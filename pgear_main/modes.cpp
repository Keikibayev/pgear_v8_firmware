// ============================================================================
// modes.cpp — Jog / Teach-ROM / Observe.  [Phase 7]
// ============================================================================
#include "modes.h"
#include "gait.h"
#include "safety.h"

void modes_reset_capture(ModeState* st) {
  for (int i = 0; i < PG_NJOINTS; i++) st->cap_valid[i] = false;
}

void modes_set_jog_target(ModeState* st, int idx, float deg, const GaitEngine* eng) {
  if (idx < 0 || idx >= PG_NJOINTS) return;
  // hard-clamp to the joint's ROM in joint-frame deg before it can be commanded
  deg = gait_clamp_to_rom(deg, eng->joints[idx].rom_min_deg, eng->joints[idx].rom_max_deg);
  st->jog_target_deg[idx] = deg;
  st->jog_has_target[idx] = true;
}

static void capture(ModeState* st, int i, float deg) {
  if (!st->cap_valid[i]) { st->cap_min_deg[i] = st->cap_max_deg[i] = deg; st->cap_valid[i] = true; }
  else { if (deg < st->cap_min_deg[i]) st->cap_min_deg[i] = deg;
         if (deg > st->cap_max_deg[i]) st->cap_max_deg[i] = deg; }
}

void modes_jog_tick(const GaitEngine* eng, ModeState* st, bool armed) {
  if (!armed) return;
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!eng->joints[i].enabled || !st->jog_has_target[i]) continue;
    float deg = gait_clamp_to_rom(st->jog_target_deg[i],
                                  eng->joints[i].rom_min_deg, eng->joints[i].rom_max_deg);
    float turns = gait_deg_to_motor_turns(deg, eng->joints[i].direction);
    can_set_input_pos(i, safety_clamp_turns(i, turns));
  }
}

void modes_teach_tick(const GaitEngine* eng, const BusTelemetry* snap, ModeState* st, bool armed) {
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!eng->joints[i].enabled || !snap->j[i].fb_valid) continue;
    float pos = snap->j[i].pos_turns;
    // zero-stiffness follow: command the motor's current position each tick
    if (armed) can_set_input_pos(i, safety_clamp_turns(i, pos));
    float deg = eng->joints[i].direction * turns_to_deg(pos);
    capture(st, i, deg);
  }
}

void modes_observe_tick(const GaitEngine* eng, const BusTelemetry* snap, ModeState* st) {
  // motors OFF — read-only capture only, NO commands
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!eng->joints[i].enabled || !snap->j[i].fb_valid) continue;
    float deg = eng->joints[i].direction * turns_to_deg(snap->j[i].pos_turns);
    capture(st, i, deg);
  }
}
