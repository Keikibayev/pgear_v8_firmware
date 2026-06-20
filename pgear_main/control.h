// control.h — the two control-law policies over one shared core.  [Phase 6 / 6b]
// A controlMode field (OP_SET_MODE; LP_FLAG_TORQUE_MODE) selects per tick.
//
// Position mode (Phase 6, port of worker.py _update_aan etc.):
//   POS_FILTER set_input_pos + AAN-by-iq PHASE-YIELDING. Patient torque =
//   iq_meas - empty.predict() [- passive.predict() / - trim]. Worst-joint
//   global yield -> cps_modifier; assist-level scales thresholds; asymmetric
//   LPF (engage 100 ms / release 30 ms).
//
// Torque mode (Phase 6b, port of labs/torque_gait.py TorqueGaitController):
//   tau = tau_grav(theta) + k_assist*sat(theta_ref-theta) - B*theta_dot
//         + tau_rom(theta,theta_dot);  capped (pain limit) + slewed.
//   Phase advance gated by cooperation gate g (patient leads pace/reversal).
#pragma once
#include "protocol.h"
// TODO P6:  step_position(dt, telem) -> {setpoints, cps_modifier}
// TODO P6b: step_torque(dt, telem)   -> {motor_input_torque}
