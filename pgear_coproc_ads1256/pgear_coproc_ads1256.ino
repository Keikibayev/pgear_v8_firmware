// ============================================================================
// pgear_coproc_ads1256.ino  —  P.GEAR v8 torque-sensor co-processor
// ----------------------------------------------------------------------------
// PHASE 3 (stub). One ESP32 + one ADS1256 (24-bit SPI ADC) channel-muxed over
// 4 FUTEK load cells, replacing the old 4xHX711 coproc. Outputs CALIBRATED N
// (per-cell scale + tare on-board; cal authority lives at the cells, reusing
// the pgear_tools/coproc calibration tools). Framed UART up to the ESP32 main.
//
// Wire contract: keep these structs byte-identical to pgear_main/protocol.h.
//   UP   : SensorPacketV2 (0xAA55, ~200 Hz) + CalStatePacket (0xAA77)
//   DOWN : DownstreamPacket (0xBB66, estop/fault + tareSeq) + CalAdjust (0xBB55)
//
// ADS1256 vs HX711: single SPI bus, DRDY interrupt, mux+PGA settle between
// channels. Service on its own core so it never stalls the UART pacing.
// ============================================================================
#include <Arduino.h>

// ---- Wire contract (MUST match pgear_main/protocol.h; static_assert guards) --
#define PG_NJOINTS 4

static inline uint16_t pg_crc16_ccitt(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

struct __attribute__((packed)) SensorPacketV2 {
  uint8_t start0, start1, seq, flags;
  float   forceN[PG_NJOINTS];
  uint16_t rateHz, crc;
};
static_assert(sizeof(SensorPacketV2) == 24, "SensorPacketV2 size (sync w/ protocol.h)");

// TODO P3:
//   - ADS1256 SPI driver: reset, RDATAC/RDATA, channel mux, PGA, DRDY ISR
//   - per-cell tare + calibrated-N scale (CalAdjust applies; persist to NVS)
//   - DownstreamPacket RX parser: estop/fault LEDs + tare-on-tareSeq-change
//   - paced SensorPacketV2 TX; CalStatePacket on boot/after-adjust/5 s
//   - independent fault LEDs + link-timeout (the coproc's safety value)

void setup() {
  Serial.begin(115200);
  Serial.println("[pgear_coproc_ads1256] boot — Phase 3 stub");
  // Serial1.begin(460800, SERIAL_8N1, RX, TX);  // to main board (verify pins)
  // ads1256_init();
}

void loop() {
  // TODO P3: service ADS1256, send SensorPacketV2 at the paced rate.
}
