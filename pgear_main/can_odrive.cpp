// ============================================================================
// can_odrive.cpp — TWAI + ODrive CAN protocol implementation.  [Phase 1]
// Port of can_odrive.py. RX runs in its own task pinned to core 0 so it never
// competes with the real-time control loop on core 1.
// ============================================================================
#include "can_odrive.h"
#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"

// ---- shared telemetry, guarded by a spinlock (cross-core RX -> control) -----
static AxisTelemetry s_tel[PG_NJOINTS];
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_running = false;
static TaskHandle_t  s_rxTask = nullptr;
static const uint32_t HB_TIMEOUT_MS = 500;

// Desired axis state per joint (what WE commanded). The watchdog re-asserts
// THIS, not the heartbeat-reported state — otherwise a transient (e.g. just
// after arm, or a FULL_CAL request) gets clobbered back to the stale state.
// Only IDLE / CLOSED_LOOP are tracked; transient states (FULL_CAL) are not, so
// the watchdog never re-triggers them.
static uint8_t s_desired[PG_NJOINTS] = { AXIS_IDLE, AXIS_IDLE, AXIS_IDLE, AXIS_IDLE };

// ---- id helpers ------------------------------------------------------------
static inline uint32_t can_id(uint8_t node_id, uint8_t cmd) {
  return ((uint32_t)node_id << CAN_CMD_BITS) | cmd;
}

// ---- low-level TX (standard 11-bit data frame) -----------------------------
static void tx(int idx, uint8_t cmd, const uint8_t* data, uint8_t len) {
  if (!s_running || idx < 0 || idx >= PG_NJOINTS) return;
  twai_message_t m = {};
  m.identifier = can_id(JOINTS[idx].node_id, cmd);
  m.extd = 0;
  m.rtr  = 0;
  m.data_length_code = len;
  for (uint8_t i = 0; i < len && i < 8; i++) m.data[i] = data[i];
  twai_transmit(&m, pdMS_TO_TICKS(5));   // best-effort; RT loop must not block
}

// little-endian packers
static inline void put_f32(uint8_t* p, float v)  { memcpy(p, &v, 4); }
static inline void put_u32(uint8_t* p, uint32_t v){ memcpy(p, &v, 4); }
static inline void put_i16(uint8_t* p, int16_t v){ memcpy(p, &v, 2); }

// ---- per-axis command ------------------------------------------------------
void can_set_input_pos(int idx, float pos_turns, float vel_ff, float torque_ff) {
  // payload: pos f32, vel_ff i16 (x1e-3), torque_ff i16 (x1e-3)
  int32_t vff = (int32_t)(vel_ff * 1000.0f);
  int32_t tff = (int32_t)(torque_ff * 1000.0f);
  if (vff > 32767) vff = 32767; if (vff < -32768) vff = -32768;
  if (tff > 32767) tff = 32767; if (tff < -32768) tff = -32768;
  uint8_t d[8] = {0};
  put_f32(d, pos_turns);
  put_i16(d + 4, (int16_t)vff);
  put_i16(d + 6, (int16_t)tff);
  tx(idx, CMD_SET_INPUT_POS, d, 8);
}

void can_set_input_torque(int idx, float torque_nm) {
  uint8_t d[8] = {0};
  put_f32(d, torque_nm);
  tx(idx, CMD_SET_INPUT_TORQUE, d, 8);
}

void can_set_axis_state(int idx, uint32_t state) {
  uint8_t d[8] = {0};
  put_u32(d, state);
  tx(idx, CMD_SET_AXIS_STATE, d, 8);
  // Track only steady states the watchdog should hold; NOT transient ones like
  // FULL_CALIBRATION_SEQUENCE (so the watchdog won't re-trigger or abort them).
  if (idx >= 0 && idx < PG_NJOINTS &&
      (state == AXIS_IDLE || state == AXIS_CLOSED_LOOP)) {
    s_desired[idx] = (uint8_t)state;
  }
}

void can_set_control_mode(int idx, uint32_t control_mode, uint32_t input_mode) {
  uint8_t d[8] = {0};
  put_u32(d, control_mode);
  put_u32(d + 4, input_mode);
  tx(idx, CMD_SET_CTRL_MODES, d, 8);
}

void can_set_limits(int idx, float vel_lim, float cur_lim) {
  uint8_t d[8] = {0};
  put_f32(d, vel_lim);
  put_f32(d + 4, cur_lim);
  tx(idx, CMD_SET_LIMITS, d, 8);
}

void can_clear_errors(int idx) {
  uint8_t d[8] = {0};
  tx(idx, CMD_CLEAR_ERRORS, d, 8);
}

// ---- bus-wide --------------------------------------------------------------
void can_idle_all() {
  for (int i = 0; i < PG_NJOINTS; i++) can_set_axis_state(i, AXIS_IDLE);
}

void can_feed_watchdog() {
  // Re-assert the DESIRED state (what we commanded), not the heartbeat-reported
  // one — so feeding never clobbers a just-requested transition.
  for (int i = 0; i < PG_NJOINTS; i++) {
    uint8_t st = s_desired[i];
    if (st == AXIS_IDLE || st == AXIS_CLOSED_LOOP) can_set_axis_state(i, st);
  }
}

// ---- telemetry snapshot ----------------------------------------------------
void can_snapshot(BusTelemetry* out) {
  if (!out) return;
  uint32_t now = millis();
  taskENTER_CRITICAL(&s_mux);
  for (int i = 0; i < PG_NJOINTS; i++) {
    out->j[i] = s_tel[i];
    uint32_t age = now - s_tel[i].last_frame_ms;
    out->j[i].fb_valid = (s_tel[i].last_frame_ms != 0) && (age < HB_TIMEOUT_MS);
  }
  taskEXIT_CRITICAL(&s_mux);
  out->bus_up = s_running;
}

// ---- RX frame decode -------------------------------------------------------
static void handle_frame(const twai_message_t& m) {
  if (m.rtr) return;
  uint8_t node = (uint8_t)(m.identifier >> CAN_CMD_BITS);
  uint8_t cmd  = (uint8_t)(m.identifier & CAN_CMD_MASK);
  int idx = joint_for_node(node);
  if (idx < 0) return;

  taskENTER_CRITICAL(&s_mux);
  AxisTelemetry& t = s_tel[idx];
  t.last_frame_ms = millis();
  switch (cmd) {
    case CMD_HEARTBEAT:
      if (m.data_length_code >= 5) {
        memcpy(&t.axis_error, &m.data[0], 4);
        t.axis_state = m.data[4];
      }
      break;
    case CMD_GET_ENC_EST:
      if (m.data_length_code >= 8) {
        memcpy(&t.pos_turns,   &m.data[0], 4);
        memcpy(&t.vel_turns_s, &m.data[4], 4);
      }
      break;
    case CMD_GET_IQ:
      if (m.data_length_code >= 8) {
        memcpy(&t.iq_setpoint_a, &m.data[0], 4);
        memcpy(&t.iq_measured_a, &m.data[4], 4);
      }
      break;
    case CMD_MOTOR_ERROR:
      if (m.data_length_code >= 4) memcpy(&t.motor_error, &m.data[0], 4);
      break;
    case CMD_ENC_ERROR:
      if (m.data_length_code >= 4) memcpy(&t.encoder_error, &m.data[0], 4);
      break;
    case CMD_CONTROLLER_ERR:
      if (m.data_length_code >= 4) memcpy(&t.controller_error, &m.data[0], 4);
      break;
    default: break;
  }
  taskEXIT_CRITICAL(&s_mux);
}

static void rxTask(void* arg) {
  (void)arg;
  twai_message_t m;
  while (s_running) {
    if (twai_receive(&m, pdMS_TO_TICKS(100)) == ESP_OK) {
      if (!m.rtr) handle_frame(m);
    }
  }
  vTaskDelete(nullptr);
}

// ---- lifecycle -------------------------------------------------------------
bool can_odrive_init() {
  twai_general_config_t g =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN,
                                  TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("[can] driver install FAILED");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[can] start FAILED");
    twai_driver_uninstall();
    return false;
  }
  s_running = true;
  xTaskCreatePinnedToCore(rxTask, "can_rx", 4096, nullptr,
                          configMAX_PRIORITIES - 3, &s_rxTask, 0);
  Serial.printf("[can] up @ %lu bps, TX=%d RX=%d\n",
                (unsigned long)CAN_BITRATE, CAN_TX_PIN, CAN_RX_PIN);
  return true;
}

void can_odrive_stop() {
  if (!s_running) return;
  can_idle_all();
  s_running = false;
  vTaskDelay(pdMS_TO_TICKS(150));   // let rxTask exit
  twai_stop();
  twai_driver_uninstall();
}
