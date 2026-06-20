// ============================================================================
// coproc_link.cpp — UART link to the ADS1256 coproc.  [Phase 3]
// ============================================================================
#include "coproc_link.h"
#include "protocol.h"
#include <Arduino.h>

#define COPROC_UART Serial1

static CoprocData    s_data;
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t      s_lastMs = 0;

// downstream tare sequence
static uint16_t s_tareSeq = 0;
static uint16_t s_calAdjSeq = 0;

// RX parser state
static uint8_t s_buf[32];
static size_t  s_len = 0;
static int     s_want = 0;     // 0 = hunting for start pair

void coproc_link_init() {
  COPROC_UART.begin(PG_COPROC_UART_BAUD, SERIAL_8N1, COPROC_RX_PIN, COPROC_TX_PIN);
  Serial.printf("[coproc] UART up @%lu TX=%d RX=%d\n",
                (unsigned long)PG_COPROC_UART_BAUD, COPROC_TX_PIN, COPROC_RX_PIN);
}

static void apply_sensor(const SensorPacketV2* p) {
  uint32_t now = millis();
  taskENTER_CRITICAL(&s_mux);
  for (int i = 0; i < PG_NJOINTS; i++) {
    s_data.forceN[i]   = p->forceN[i];
    s_data.torqueNm[i] = p->forceN[i] * MOMENT_ARM_M[i];
  }
  s_data.sensorFlags = p->flags;
  s_data.online = true;
  s_lastMs = now;
  taskEXIT_CRITICAL(&s_mux);
}

static void apply_calstate(const CalStatePacket* p) {
  taskENTER_CRITICAL(&s_mux);
  for (int i = 0; i < PG_NJOINTS; i++) s_data.cal[i] = p->cal[i];
  taskEXIT_CRITICAL(&s_mux);
}

void coproc_link_poll() {
  while (COPROC_UART.available()) {
    uint8_t b = COPROC_UART.read();
    if (s_want == 0) {
      s_buf[0] = s_buf[1]; s_buf[1] = b; s_len = 2;
      if (s_buf[0] == 0xAA && s_buf[1] == 0x55) s_want = sizeof(SensorPacketV2);
      else if (s_buf[0] == 0xAA && s_buf[1] == 0x77) s_want = sizeof(CalStatePacket);
    } else {
      if (s_len < sizeof(s_buf)) s_buf[s_len] = b;
      s_len++;
      if ((int)s_len >= s_want) {
        if (s_buf[1] == 0x55) {
          SensorPacketV2* p = (SensorPacketV2*)s_buf;
          if (pg_crc16_ccitt(s_buf, sizeof(SensorPacketV2) - 2) == p->crc) apply_sensor(p);
          else { s_data.crcFails++; s_data.resyncs++; }
        } else {
          CalStatePacket* p = (CalStatePacket*)s_buf;
          if (pg_crc16_ccitt(s_buf, sizeof(CalStatePacket) - 2) == p->crc) apply_calstate(p);
          else s_data.crcFails++;
        }
        s_want = 0; s_len = 0;
      }
    }
  }
}

void coproc_get(CoprocData* out) {
  if (!out) return;
  uint32_t now = millis();
  taskENTER_CRITICAL(&s_mux);
  *out = s_data;
  uint32_t age = now - s_lastMs;
  out->ageMs = (s_lastMs == 0) ? 9999 : age;
  out->online = (s_lastMs != 0) && (age < COPROC_LINK_TIMEOUT_MS);
  taskEXIT_CRITICAL(&s_mux);
}

// ---- downstream TX ---------------------------------------------------------
void coproc_send_downstream(uint8_t fault_flags, bool estop) {
  DownstreamPacket p;
  p.start0 = 0xBB; p.start1 = 0x66;
  p.flags = fault_flags | (estop ? DS_FLAG_ESTOP : 0);
  p.reserved = 0;
  p.tareSeq = s_tareSeq;
  p.crc = pg_crc16_ccitt((uint8_t*)&p, sizeof(p) - 2);
  COPROC_UART.write((uint8_t*)&p, sizeof(p));
}

void coproc_request_tare() { s_tareSeq++; coproc_send_downstream(0, false); }

void coproc_send_caladjust(uint8_t joint, float ratio) {
  CalAdjustPacket p;
  p.start0 = 0xBB; p.start1 = 0x55; p.joint = joint; p.reserved = 0;
  p.ratio = ratio; p.seq = ++s_calAdjSeq;
  p.crc = pg_crc16_ccitt((uint8_t*)&p, sizeof(p) - 2);
  COPROC_UART.write((uint8_t*)&p, sizeof(p));
}
