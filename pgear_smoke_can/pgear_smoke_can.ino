// ============================================================================
// pgear_smoke_can.ino  —  Phase 1 CAN bring-up smoke test (SELF-CONTAINED)
// ----------------------------------------------------------------------------
// Validates the ESP32-S3 <-> ODrive CAN path WITHOUT the control firmware:
//   1. installs TWAI @ 250 kbps on the CAN transceiver pins,
//   2. forces all 4 axes to IDLE + clears errors (SAFE — never closed loop),
//   3. prints every decoded frame (heartbeat / encoder / iq) per node,
//   4. prints a per-node "seen" summary each second.
//
// Mirrors the intent of pi_gui smoke_test_can.py. If you see heartbeats +
// encoder estimates from all four node ids, CAN is good and Phase 2 can start.
//
// ⚠ Pins: CAN shares GPIO19/20 with native USB via the CH422G EXIO5 switch.
// This sketch sets EXIO5=HIGH (CAN mode) at boot, so flash/serial must use the
// SEPARATE "USB TO UART" CH343P port (set Arduino "USB CDC On Boot: DISABLED").
// Requires the Waveshare ESP_IOExpander library.
// ============================================================================
#include <Arduino.h>
#include "driver/twai.h"
#include <ESP_IOExpander_Library.h>

#define EXIO_USB_SEL 5   // CH422G: HIGH = CAN mode

#ifndef CAN_RX_PIN
#define CAN_RX_PIN 19
#endif
#ifndef CAN_TX_PIN
#define CAN_TX_PIN 20
#endif

static const int      CMD_BITS = 5;
static const int      CMD_MASK = 0x1F;
static const uint8_t  NODE_IDS[4] = { 10, 11, 2, 3 };  // HR, KR, HL, KL
static const char*    NODE_NAME[4] = { "HR", "KR", "HL", "KL" };

// ODrive cmd ids
enum { C_HEARTBEAT=0x001, C_SET_AXIS_STATE=0x007, C_GET_ENC_EST=0x009,
       C_GET_IQ=0x014, C_CLEAR_ERRORS=0x018 };
enum { AXIS_IDLE = 1 };

static uint32_t seen[4]   = {0,0,0,0};
static uint8_t  state[4]  = {0,0,0,0};
static float    pos[4]    = {0,0,0,0};
static float    iqm[4]    = {0,0,0,0};

static int node_idx(uint8_t node) {
  for (int i = 0; i < 4; i++) if (NODE_IDS[i] == node) return i;
  return -1;
}

static void tx(uint8_t node, uint8_t cmd, const uint8_t* d, uint8_t len) {
  twai_message_t m = {};
  m.identifier = ((uint32_t)node << CMD_BITS) | cmd;
  m.extd = 0; m.rtr = 0; m.data_length_code = len;
  for (uint8_t i = 0; i < len && i < 8; i++) m.data[i] = d[i];
  twai_transmit(&m, pdMS_TO_TICKS(10));
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[smoke_can] Phase 1 CAN bring-up test");

  // Route GPIO19/20 to the CAN transceiver via CH422G EXIO5 (HIGH = CAN).
  ESP_IOExpander* exp = new ESP_IOExpander_CH422G(
      (i2c_port_t)I2C_NUM_0, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS, 9, 8);
  exp->init();
  exp->begin();
  exp->pinMode(EXIO_USB_SEL, OUTPUT);
  exp->digitalWrite(EXIO_USB_SEL, HIGH);
  Serial.println("[smoke_can] EXIO5=HIGH -> CAN mode");

  twai_general_config_t g =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN,
                                  TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("[smoke_can] TWAI init FAILED — check transceiver + pins");
    return;
  }
  Serial.printf("[smoke_can] TWAI up @250k TX=%d RX=%d. Forcing IDLE...\n",
                CAN_TX_PIN, CAN_RX_PIN);

  uint8_t z[8] = {0};
  for (int i = 0; i < 4; i++) {
    tx(NODE_IDS[i], C_CLEAR_ERRORS, z, 8);
    uint8_t d[8] = {0}; uint32_t s = AXIS_IDLE; memcpy(d, &s, 4);
    tx(NODE_IDS[i], C_SET_AXIS_STATE, d, 8);
  }
  Serial.println("[smoke_can] listening (expect heartbeats from HR/KR/HL/KL)...");
}

void loop() {
  static uint32_t lastReport = 0;
  twai_message_t m;
  while (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) {
    if (m.rtr) continue;
    int i = node_idx((uint8_t)(m.identifier >> CMD_BITS));
    if (i < 0) continue;
    uint8_t cmd = m.identifier & CMD_MASK;
    seen[i]++;
    if (cmd == C_HEARTBEAT && m.data_length_code >= 5) state[i] = m.data[4];
    else if (cmd == C_GET_ENC_EST && m.data_length_code >= 8) memcpy(&pos[i], &m.data[0], 4);
    else if (cmd == C_GET_IQ && m.data_length_code >= 8) memcpy(&iqm[i], &m.data[4], 4);
  }

  uint32_t now = millis();
  if (now - lastReport >= 1000) {
    lastReport = now;
    twai_status_info_t st; twai_get_status_info(&st);
    Serial.printf("[smoke_can] bus_err=%lu  ", (unsigned long)st.bus_error_count);
    for (int i = 0; i < 4; i++) {
      Serial.printf("%s[n%u f=%lu st=%u pos=%.2f iq=%.2f] ",
                    NODE_NAME[i], NODE_IDS[i], (unsigned long)seen[i],
                    state[i], pos[i], iqm[i]);
      seen[i] = 0;
    }
    Serial.println();
  }
}
