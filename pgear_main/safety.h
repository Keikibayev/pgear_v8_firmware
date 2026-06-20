// safety.h — safety supervisor.  [Phase 5]
// Port of the guards scattered through worker.py + the v7.1.5 sensor watchdog:
//   - e-stop (latched) + reset
//   - ODrive heartbeat watchdog (per-axis age; feed at ~10 Hz)
//   - ROM enforcement (soft, deg-frame) over hardware GPIO endstops (hard)
//   - cross-check fault (iq vs measured torque divergence)
//   - sensor watchdog: ~20 Nm/s rate cap + glitch-rate -> E-STOP
//   - drift watcher: EWMA(iq_meas - predict) warns when > residStdA
#pragma once
#include "protocol.h"
// TODO P5: safety_tick(...) -> updates g_estop + crossCheckFault/hbErrorByte bits.
