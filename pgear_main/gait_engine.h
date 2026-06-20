// gait_engine.h — IDLE / INIT_SETTLE / GAIT / HOMING state machine.  [Phase 2]
// Port of pgear_tools/pi_gui/pgear_pi/control/gait_engine.py. Owns gait phase
// and per-joint motor-turn setpoints; lerp-from-current ramps (anti-jerk fix:
// HOMING starts from last_ref_turns, not last_pos_turns).
#pragma once
// TODO P2: GaitEngine struct: start/stop/home, tick(dt)->setpoints, homing checks.
