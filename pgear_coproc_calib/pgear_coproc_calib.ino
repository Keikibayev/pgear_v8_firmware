// ============================================================================
// pgear_coproc_calib.ino — load-cell CALIBRATION helper (standalone).
// ----------------------------------------------------------------------------
// Flash this to the coproc INSTEAD of pgear_coproc_ads1256.ino while calibrating.
// It reads the 4 ADS1256 channels and prints tared counts over USB serial as
// CSV, so the PC calibration GUI can show live cell values while you load each
// joint with a dynamometer at a known torque.
//
//   Serial @ 115200, line format:   c0,c1,c2,c3   (tared ADC counts, ~20 Hz)
//   Send 't' (any line containing 't') -> tare/zero all channels at rest.
//   Send 'r' -> clear the tare (back to raw counts).
//
// Same ADS1256 wiring as pgear_coproc_ads1256.ino (SCLK18/DIN23/DOUT19/CS5/
// DRDY21; PDWN tied 3V3 on the LC Tech v1.1 module; software reset via 0xFE).
// When done, put the measured counts-per-Nm into pgear_coproc_ads1256.ino
// (g_cal / DEFAULT_CAL) and set MOMENT_ARM_M[]=1.0 on the main board.
// ============================================================================
#include <Arduino.h>
#include <SPI.h>

#define NCH 4
#define PIN_SCLK 18
#define PIN_MOSI 23   // DIN
#define PIN_MISO 19   // DOUT
#define PIN_CS    5
#define PIN_DRDY 21

#define CMD_RDATA 0x01
#define CMD_SDATAC 0x0F
#define CMD_WREG  0x50
#define CMD_SELFCAL 0xF0
#define CMD_SYNC  0xFC
#define CMD_WAKEUP 0x00
#define CMD_RESET 0xFE
#define REG_STATUS 0x00
#define REG_MUX    0x01
#define REG_ADCON  0x02
#define REG_DRATE  0x03
#define PGA_CODE   0x06     // gain x64 (load-cell bridge)
#define DRATE_1000 0xA1
static const uint8_t MUX_CH[NCH] = { 0x01, 0x23, 0x45, 0x67 };  // 4 differential pairs
static SPISettings ADS_SPI(1700000, MSBFIRST, SPI_MODE1);

static int32_t g_tare[NCH] = {0, 0, 0, 0};

static inline void cs(bool on) { digitalWrite(PIN_CS, on ? LOW : HIGH); }
static bool waitDrdy(uint32_t to_us = 5000) {
  uint32_t t0 = micros();
  while (digitalRead(PIN_DRDY) == HIGH) if (micros() - t0 > to_us) return false;
  return true;
}
static void wreg(uint8_t reg, uint8_t val) {
  SPI.transfer(CMD_WREG | reg); SPI.transfer(0x00); SPI.transfer(val);
}
static int32_t readChannel(uint8_t mux) {
  SPI.beginTransaction(ADS_SPI); cs(true);
  wreg(REG_MUX, mux);
  SPI.transfer(CMD_SYNC); SPI.transfer(CMD_WAKEUP);
  bool ok = waitDrdy();
  SPI.transfer(CMD_RDATA); delayMicroseconds(7);
  uint8_t b2 = SPI.transfer(0xFF), b1 = SPI.transfer(0xFF), b0 = SPI.transfer(0xFF);
  cs(false); SPI.endTransaction();
  if (!ok) return 0;
  int32_t v = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  if (v & 0x800000) v |= 0xFF000000;     // sign-extend 24->32
  return v;
}

static int32_t g_raw[NCH] = {0,0,0,0};
static void readAll() { for (int i = 0; i < NCH; i++) g_raw[i] = readChannel(MUX_CH[i]); }

static void doTare(int n = 32) {
  int64_t acc[NCH] = {0,0,0,0};
  for (int k = 0; k < n; k++) { readAll(); for (int i = 0; i < NCH; i++) acc[i] += g_raw[i]; }
  for (int i = 0; i < NCH; i++) g_tare[i] = (int32_t)(acc[i] / n);
  Serial.println("# tared");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT); cs(false);
  pinMode(PIN_DRDY, INPUT);
  delay(50);
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);
  SPI.beginTransaction(ADS_SPI); cs(true); SPI.transfer(CMD_RESET); cs(false); SPI.endTransaction();
  delay(50);
  SPI.beginTransaction(ADS_SPI); cs(true);
  SPI.transfer(CMD_SDATAC);
  wreg(REG_STATUS, 0x06);          // MSB first, auto-cal, buffer on
  wreg(REG_ADCON, PGA_CODE);       // PGA x64
  wreg(REG_DRATE, DRATE_1000);
  SPI.transfer(CMD_SELFCAL);
  cs(false); SPI.endTransaction();
  delay(10);
  Serial.println("# pgear_coproc_calib — CSV c0,c1,c2,c3 (tared counts). 't'=tare 'r'=raw");
  doTare();
}

void loop() {
  static uint32_t next = 0;
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch == 't' || ch == 'T') doTare();
    else if (ch == 'r' || ch == 'R') { for (int i = 0; i < NCH; i++) g_tare[i] = 0; Serial.println("# raw"); }
  }
  readAll();
  uint32_t now = millis();
  if (now >= next) {                 // ~20 Hz
    next = now + 50;
    Serial.printf("%ld,%ld,%ld,%ld\n",
                  (long)(g_raw[0] - g_tare[0]), (long)(g_raw[1] - g_tare[1]),
                  (long)(g_raw[2] - g_tare[2]), (long)(g_raw[3] - g_tare[3]));
  }
}
