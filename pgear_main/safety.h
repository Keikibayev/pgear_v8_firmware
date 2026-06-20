// ============================================================================
// safety.h — safety supervisor.  [Phase 5]
// NO physical endstops in this setup, so the firmware owns ALL limits:
//   - hardware E-STOP button on a GPIO (latched) — plus a HW motor-power cutoff
//     that does NOT depend on firmware (see docs/TODO.md / WIRING.md)
//   - ODrive heartbeat watchdog: an enabled+armed axis going stale -> trip
//   - motor-turn ENVELOPE clamp on every commanded position (sole hard ROM net)
//   - sensor watchdog: torque rate cap + glitch-rate -> trip
//   - cross-check: iq-effort vs FUTEK torque divergence -> warn (bit), no trip
// ============================================================================
#pragma once
#include <stdint.h>
#include "can_odrive.h"
#include "coproc_link.h"

void  safety_init();

// Evaluate all guards for this tick. Returns true if a trip condition is active
// (caller latches e-stop). Fills the fault bitmasks (bit i = joint i).
bool  safety_tick(bool armed, bool running,
                  const BusTelemetry* snap, const CoprocData* cd,
                  uint8_t* out_crossCheck, uint8_t* out_hbErr);

// Hard-clamp a commanded motor position to the joint's mechanical envelope.
// This is the ONLY hard ROM limit (no endstops) — always apply before TX.
float safety_clamp_turns(int idx, float turns);

bool  safety_estop_pressed();   // raw button read
