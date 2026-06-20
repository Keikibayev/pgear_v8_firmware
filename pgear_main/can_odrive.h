// can_odrive.h — TWAI (CAN) + ODrive command protocol.  [Phase 1]
// Port of pgear_tools/pi_gui/pgear_pi/transport/can_odrive.py; reuse the CAN
// bring-up from Exoskeleton_control_v7.1.5/waveshare_twai_port.cpp.
//   - 250 kbps; node ids R-knee=11, R-hip=10, L-knee=3, L-hip=2
//   - cmds: set_input_pos / set_input_torque / set_axis_state / set_control_mode
//           / set_limits / clear_errors ; 0.5 s ODrive watchdog fed ~10 Hz
//   - RX task caches per-axis pos/vel/iq/heartbeat into a telemetry snapshot
#pragma once
#include "protocol.h"
// TODO P1: void can_odrive_init(); snapshot accessor; command senders.
