// ============================================================================
// modes.h — setup modes: Jog / Teach-ROM / Observe.  [Phase 7]
// Ports of pi_gui worker._jog_tick / _teach_tick / _observe_capture. Mutually
// exclusive with gait/torque (and with each other). Run at GAIT_STEP_HZ.
//   JOG     : armed + IDLE; move each joint to a ROM-clamped target deg.
//   TEACH   : armed; zero-stiffness follow (command current pos) + capture
//             min/max as the therapist moves the leg by hand.
//   OBSERVE : NOT armed (motors off); read-only min/max capture.
// ============================================================================
#pragma once
#include "gait_engine.h"
#include "can_odrive.h"

enum SetupMode : uint8_t { SETUP_NONE = 0, SETUP_JOG, SETUP_TEACH, SETUP_OBSERVE };

struct ModeState {
  float jog_target_deg[PG_NJOINTS] = {0};
  bool  jog_has_target[PG_NJOINTS] = {false};
  float cap_min_deg[PG_NJOINTS]    = {0};   // teach/observe captured range
  float cap_max_deg[PG_NJOINTS]    = {0};
  bool  cap_valid[PG_NJOINTS]      = {false};
};

void modes_reset_capture(ModeState* st);
void modes_set_jog_target(ModeState* st, int idx, float deg, const GaitEngine* eng);

// per-tick (caller selects by active mode). `armed` gates motor commands.
void modes_jog_tick(const GaitEngine* eng, ModeState* st, bool armed);
void modes_teach_tick(const GaitEngine* eng, const BusTelemetry* snap, ModeState* st, bool armed);
void modes_observe_tick(const GaitEngine* eng, const BusTelemetry* snap, ModeState* st);
