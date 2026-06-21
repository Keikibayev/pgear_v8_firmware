// ============================================================================
// pgear_coproc_ads1256.ino  —  P.GEAR v8 torque-sensor co-processor  [Phase 3]
// ----------------------------------------------------------------------------
// One ESP32 + one ADS1256 (24-bit SPI ADC, PGA up to 64) channel-muxed over 4
// FUTEK load-cell bridges, replacing the old 4xHX711 coproc. Outputs CALIBRATED
// NEWTONS per cell; the main board does moment-arm -> torque. Framed UART up to
// the main ESP32-S3 (UART2 / GPIO43/44 on the main side).
//
// Wire contract: structs MUST stay byte-identical to pgear_main/protocol.h
// (static_assert guards below).
//   UP   : SensorPacketV2 (0xAA55, paced ~200 Hz) + CalStatePacket (0xAA77)
//   DOWN : DownstreamPacket (0xBB66: estop/fault + tareSeq) + CalAdjust (0xBB55)
//
// Assumes a CLASSIC ESP32 coproc board (VSPI). On an ESP32-S3 coproc, avoid
// GPIO19/20 (native USB) and re-map the SPI pins below.
// ============================================================================
#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>

#define PG_NJOINTS 4
#define UART_BAUD  460800

// ---- coproc pinout (classic ESP32) -----------------------------------------
#define PIN_UART_TX   17   // -> main UART2 RX (GPIO44)
#define PIN_UART_RX   16   // <- main UART2 TX (GPIO43)
// ADS1256 lines on ONE board-edge row (single connector). PDWN/RESET are NOT
// wired: this module doesn't break them out (it ties them HIGH internally), so
// we reset over SPI (0xFE) instead of a hardware pulse. Connector = SCLK, DIN,
// DOUT, CS, DRDY + 3V3 + GND.
#define PIN_ADS_SCLK  18
#define PIN_ADS_MOSI  23   // DIN
#define PIN_ADS_MISO  19   // DOUT
#define PIN_ADS_CS     5
#define PIN_ADS_DRDY  21   // input (active low)   — moved from 25
#define PIN_LED        2   // onboard heartbeat/status LED

// ---- ADS1256 commands / registers ------------------------------------------
#define ADS_CMD_WAKEUP  0x00
#define ADS_CMD_RDATA   0x01
#define ADS_CMD_SDATAC  0x0F
#define ADS_CMD_RREG    0x10
#define ADS_CMD_WREG    0x50
#define ADS_CMD_SELFCAL 0xF0
#define ADS_CMD_SYNC    0xFC
#define ADS_CMD_RESET   0xFE
#define ADS_REG_STATUS  0x00
#define ADS_REG_MUX     0x01
#define ADS_REG_ADCON   0x02
#define ADS_REG_DRATE   0x03
// PGA gain code (ADCON low 3 bits): 6 = x64 (load-cell bridges)
#define ADS_PGA_CODE    0x06
#define ADS_DRATE_1000  0xA1   // 1000 SPS
// AINCOM-less differential mux per cell: (AINP<<4)|AINN
static const uint8_t MUX_CH[PG_NJOINTS] = { 0x01, 0x23, 0x45, 0x67 };

// ADS1256 SPI: mode 1 (CPOL=0, CPHA=1), ~1.7 MHz (<= fCLKIN/4)
static SPISettings ADS_SPI(1700000, MSBFIRST, SPI_MODE1);

// ---- calibration / tare (per cell) -----------------------------------------
// cal = counts per Newton. force_N = (raw - tare) / cal. PLACEHOLDER defaults —
// calibrate with the pgear_tools/coproc tools (hang a known load).
#define DEFAULT_CAL  100000.0f
static float    g_cal[PG_NJOINTS]  = { DEFAULT_CAL, DEFAULT_CAL, DEFAULT_CAL, DEFAULT_CAL };
static int32_t  g_tare[PG_NJOINTS] = { 0, 0, 0, 0 };
static int32_t  g_raw[PG_NJOINTS]  = { 0, 0, 0, 0 };
static Preferences g_nvs;

// ---- packet structs (sync with pgear_main/protocol.h) ----------------------
struct __attribute__((packed)) SensorPacketV2 {
  uint8_t start0, start1, seq, flags;
  float   forceN[PG_NJOINTS];
  uint16_t rateHz, crc;
};
static_assert(sizeof(SensorPacketV2) == 24, "SensorPacketV2 size");

struct __attribute__((packed)) CalStatePacket {
  uint8_t start0, start1, reserved, reasonCode;
  float   cal[PG_NJOINTS];
  uint16_t crc;
};
static_assert(sizeof(CalStatePacket) == 22, "CalStatePacket size");

struct __attribute__((packed)) DownstreamPacket {
  uint8_t start0, start1, flags, reserved;
  uint16_t tareSeq, crc;
};
static_assert(sizeof(DownstreamPacket) == 8, "DownstreamPacket size");

struct __attribute__((packed)) CalAdjustPacket {
  uint8_t start0, start1, joint, reserved;
  float   ratio;
  uint16_t seq, crc;
};
static_assert(sizeof(CalAdjustPacket) == 12, "CalAdjustPacket size");

#define SENSOR_FLAG_HEARTBEAT (1 << 7)
#define DS_FLAG_ESTOP         (1 << 7)

static uint16_t crc16_ccitt(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// ---- ADS1256 low level -----------------------------------------------------
static inline void ads_cs(bool on) { digitalWrite(PIN_ADS_CS, on ? LOW : HIGH); }

static bool ads_wait_drdy(uint32_t timeout_us = 5000) {
  uint32_t t0 = micros();
  while (digitalRead(PIN_ADS_DRDY) == HIGH)
    if (micros() - t0 > timeout_us) return false;
  return true;
}

static void ads_wreg(uint8_t reg, uint8_t val) {
  SPI.transfer(ADS_CMD_WREG | reg);
  SPI.transfer(0x00);          // write 1 register (count-1)
  SPI.transfer(val);
}

static int32_t ads_read_channel(uint8_t mux) {
  SPI.beginTransaction(ADS_SPI);
  ads_cs(true);
  ads_wreg(ADS_REG_MUX, mux);
  SPI.transfer(ADS_CMD_SYNC);
  SPI.transfer(ADS_CMD_WAKEUP);
  bool ok = ads_wait_drdy();
  SPI.transfer(ADS_CMD_RDATA);
  delayMicroseconds(7);        // t6: >= 50 * tCLKIN
  uint8_t b2 = SPI.transfer(0xFF);
  uint8_t b1 = SPI.transfer(0xFF);
  uint8_t b0 = SPI.transfer(0xFF);
  ads_cs(false);
  SPI.endTransaction();
  if (!ok) return 0x7FFFFFFF;  // sentinel: not ready
  int32_t v = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  if (v & 0x800000) v |= 0xFF000000;   // sign-extend 24->32
  return v;
}

static void ads_init() {
  pinMode(PIN_ADS_CS, OUTPUT);   ads_cs(false);
  pinMode(PIN_ADS_DRDY, INPUT);
  delay(50);                            // allow the chip's power-on reset

  SPI.begin(PIN_ADS_SCLK, PIN_ADS_MISO, PIN_ADS_MOSI, PIN_ADS_CS);

  // Software reset over SPI (no hardware RESET pin on this module).
  SPI.beginTransaction(ADS_SPI);
  ads_cs(true);
  SPI.transfer(ADS_CMD_RESET);          // 0xFE
  ads_cs(false);
  SPI.endTransaction();
  delay(50);                            // reset completes; DRDY then toggles

  SPI.beginTransaction(ADS_SPI);
  ads_cs(true);
  SPI.transfer(ADS_CMD_SDATAC);
  ads_wreg(ADS_REG_STATUS, 0x06);       // MSB first, auto-cal on, buffer on
  ads_wreg(ADS_REG_ADCON, ADS_PGA_CODE);// clkout off, PGA x64
  ads_wreg(ADS_REG_DRATE, ADS_DRATE_1000);
  SPI.transfer(ADS_CMD_SELFCAL);
  ads_cs(false);
  SPI.endTransaction();
  delay(10);
}

// ---- tare / cal persistence ------------------------------------------------
static void do_tare_all(int n = 16) {
  for (int i = 0; i < PG_NJOINTS; i++) {
    int64_t acc = 0; int got = 0;
    for (int k = 0; k < n; k++) {
      int32_t v = ads_read_channel(MUX_CH[i]);
      if (v != 0x7FFFFFFF) { acc += v; got++; }
    }
    if (got) g_tare[i] = (int32_t)(acc / got);
  }
  Serial.println("[coproc] tare done");
}

static void cal_load() {
  g_nvs.begin("pgearcal", true);
  for (int i = 0; i < PG_NJOINTS; i++) {
    char key[8]; snprintf(key, sizeof(key), "cal%d", i);
    g_cal[i] = g_nvs.getFloat(key, DEFAULT_CAL);
  }
  g_nvs.end();
}
static void cal_save_one(int i) {
  g_nvs.begin("pgearcal", false);
  char key[8]; snprintf(key, sizeof(key), "cal%d", i);
  g_nvs.putFloat(key, g_cal[i]);
  g_nvs.end();
}

// ---- UART up ---------------------------------------------------------------
static uint8_t  g_seq = 0;
static uint16_t g_dsTareSeq = 0; bool g_dsTareInit = false;
static uint8_t  g_dsFlags = 0;
static uint16_t g_calAdjSeq = 0; bool g_calAdjInit = false;

static void send_sensor() {
  SensorPacketV2 p;
  p.start0 = 0xAA; p.start1 = 0x55; p.seq = g_seq++;
  static bool hb = false; hb = !hb;
  p.flags = hb ? SENSOR_FLAG_HEARTBEAT : 0;
  for (int i = 0; i < PG_NJOINTS; i++) {
    float n = (float)(g_raw[i] - g_tare[i]) / g_cal[i];
    p.forceN[i] = n;
    if (g_raw[i] == 0x7FFFFFFF) p.flags |= (1 << i);   // channel invalid
  }
  p.rateHz = 1000;
  p.crc = crc16_ccitt((uint8_t*)&p, sizeof(p) - 2);
  Serial1.write((uint8_t*)&p, sizeof(p));
}

static void send_calstate(uint8_t reason) {
  CalStatePacket p;
  p.start0 = 0xAA; p.start1 = 0x77; p.reserved = 0; p.reasonCode = reason;
  for (int i = 0; i < PG_NJOINTS; i++) p.cal[i] = g_cal[i];
  p.crc = crc16_ccitt((uint8_t*)&p, sizeof(p) - 2);
  Serial1.write((uint8_t*)&p, sizeof(p));
}

// ---- UART down parser (dispatch on start1) ---------------------------------
static uint8_t g_rx[16]; static size_t g_rxLen = 0; static int g_rxWant = 0;

static void handle_downstream() {
  DownstreamPacket* p = (DownstreamPacket*)g_rx;
  if (crc16_ccitt(g_rx, sizeof(DownstreamPacket) - 2) != p->crc) return;
  g_dsFlags = p->flags;
  if (!g_dsTareInit) { g_dsTareSeq = p->tareSeq; g_dsTareInit = true; }
  else if (p->tareSeq != g_dsTareSeq) { g_dsTareSeq = p->tareSeq; do_tare_all(); }
}
static void handle_caladjust() {
  CalAdjustPacket* p = (CalAdjustPacket*)g_rx;
  if (crc16_ccitt(g_rx, sizeof(CalAdjustPacket) - 2) != p->crc) return;
  if (!g_calAdjInit) { g_calAdjSeq = p->seq; g_calAdjInit = true; }
  else if (p->seq == g_calAdjSeq) return;            // dedup
  g_calAdjSeq = p->seq;
  if (p->joint >= PG_NJOINTS) return;
  float r = p->ratio;
  if (fabsf(r) < 0.01f || fabsf(r) > 100.0f) return; // reject out of range
  g_cal[p->joint] *= r;
  cal_save_one(p->joint);
  send_calstate(2);                                  // reason: after-adjust
}

static void poll_downstream() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (g_rxWant == 0) {                 // hunting for start pair
      g_rx[0] = g_rx[1]; g_rx[1] = b; g_rxLen = 2;
      if (g_rx[0] == 0xBB && g_rx[1] == 0x66) g_rxWant = sizeof(DownstreamPacket);
      else if (g_rx[0] == 0xBB && g_rx[1] == 0x55) g_rxWant = sizeof(CalAdjustPacket);
    } else {
      g_rx[g_rxLen++] = b;
      if ((int)g_rxLen >= g_rxWant) {
        if (g_rx[1] == 0x66) handle_downstream();
        else                 handle_caladjust();
        g_rxWant = 0; g_rxLen = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  Serial.println("\n[pgear_coproc_ads1256] boot");
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  cal_load();
  ads_init();
  do_tare_all();
  send_calstate(1);                       // reason: boot
  Serial.println("[coproc] running");
}

void loop() {
  static uint32_t nextTx = 0, lastCalState = 0, lastBlink = 0;
  uint32_t now = millis();

  // round-robin read all 4 cells
  for (int i = 0; i < PG_NJOINTS; i++) g_raw[i] = ads_read_channel(MUX_CH[i]);

  poll_downstream();

  if (now >= nextTx) { send_sensor(); nextTx = now + 5; }       // ~200 Hz
  if (now - lastCalState >= 5000) { lastCalState = now; send_calstate(3); }
  if (now - lastBlink >= 500) { lastBlink = now; digitalWrite(PIN_LED, !digitalRead(PIN_LED)); }
}
