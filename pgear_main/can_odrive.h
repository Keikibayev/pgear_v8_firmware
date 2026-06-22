// ============================================================================
// can_odrive.h — TWAI (CAN) + ODrive command protocol.  [Phase 1]
// Port of pgear_tools/pi_gui/pgear_pi/transport/can_odrive.py.
//   - 250 kbps; id = (node_id<<5)|cmd_id, little-endian payloads.
//   - RX task (core 0) caches per-axis pos/vel/iq/state/errors + frame age.
//   - TX from any core; twai_transmit is serialized by the driver.
//   - Watchdog: re-assert axis state ~10 Hz (ODrive resets its 0.5 s wd on any
//     frame for that axis — no dedicated feed cmd).
// ============================================================================
#pragma once
#include <stdint.h>
#include "constants.h"

struct AxisTelemetry {
  float    pos_turns      = 0.0f;
  float    vel_turns_s    = 0.0f;
  float    iq_measured_a  = 0.0f;
  float    iq_setpoint_a  = 0.0f;
  uint32_t axis_error     = 0;
  uint32_t motor_error    = 0;
  uint32_t encoder_error  = 0;
  uint32_t controller_error = 0;
  uint8_t  axis_state     = AXIS_IDLE;
  uint32_t last_frame_ms  = 0;     // millis() of last frame for this axis
  bool     fb_valid       = false; // encoder estimate seen within hb timeout
};

struct BusTelemetry {
  AxisTelemetry j[PG_NJOINTS];
  bool     bus_up = false;
  uint16_t crc_fails = 0;          // driver bus-error proxy (diagnostics)
};

// lifecycle
bool can_odrive_init();            // install+start TWAI, spawn RX task
void can_odrive_stop();

// per-axis command (hot path)
void can_set_input_pos(int idx, float pos_turns, float vel_ff = 0.0f, float torque_ff = 0.0f);
void can_set_input_torque(int idx, float torque_nm);
// per-axis config (cold path)
void can_set_axis_state(int idx, uint32_t state);
void can_set_control_mode(int idx, uint32_t control_mode, uint32_t input_mode);
void can_set_limits(int idx, float vel_lim, float cur_lim);
void can_clear_errors(int idx);
// bus-wide
void can_idle_all();
void can_feed_watchdog();
// telemetry (thread-safe copy)
void can_snapshot(BusTelemetry* out);
// diagnostics: print TWAI controller state + TX/RX/bus error counters
void can_dump_status(const char* tag);
