// ============================================================================
// gait_engine.cpp — gait state machine.  Port of gait_engine.py.  [Phase 2]
// ============================================================================
#include "gait_engine.h"
#include "gait.h"
#include <Arduino.h>

void GaitEngine::init_defaults() {
  for (int i = 0; i < PG_NJOINTS; i++) {
    bool hip = (JOINTS[i].kind == KIND_HIP);
    joints[i].direction      = JOINTS[i].default_dir;
    joints[i].enabled        = true;
    joints[i].rom_min_deg    = hip ? ROM_HIP_MIN_DEG  : ROM_KNEE_MIN_DEG;
    joints[i].rom_max_deg    = hip ? ROM_HIP_MAX_DEG  : ROM_KNEE_MAX_DEG;
    joints[i].init_pos_deg   = INIT_POS_DEG[i];
    joints[i].home_offset_deg = 0.0f;
    joints[i].last_ref_turns = 0.0f;
    joints[i].last_pos_turns = 0.0f;
  }
}

void GaitEngine::start_gait() {
  phase = PH_INIT_SETTLE;
  phase01 = 0.0f;
  phase_started_ms_ = millis();
  // snapshot current pos so INIT_SETTLE ramps smoothly to the init target
  for (int i = 0; i < PG_NJOINTS; i++)
    init_from_turns_[i] = joints[i].last_pos_turns;
}

void GaitEngine::stop_gait() { phase = PH_IDLE; }

void GaitEngine::start_homing() {
  phase = PH_HOMING;
  phase_started_ms_ = millis();
  // ramp from LAST COMMANDED pos (not measured) — avoids stepping the command
  // backward by the POS_FILTER follow-error and jerking the leg.
  for (int i = 0; i < PG_NJOINTS; i++)
    homing_from_turns_[i] = joints[i].last_ref_turns;
}

void GaitEngine::go_idle() { phase = PH_IDLE; }

void GaitEngine::tick(float dt_s, float out_turns[PG_NJOINTS], bool out_has[PG_NJOINTS]) {
  for (int i = 0; i < PG_NJOINTS; i++) out_has[i] = false;
  uint32_t now = millis();

  if (phase == PH_INIT_SETTLE) {
    float elapsed = (float)(now - phase_started_ms_);
    float u = elapsed / (float)INIT_SETTLE_MS; if (u > 1.0f) u = 1.0f;
    for (int i = 0; i < PG_NJOINTS; i++) {
      JointState& s = joints[i];
      if (!s.enabled) continue;
      float target = gait_init_target_turns(s.init_pos_deg, s.home_offset_deg,
                                            s.rom_min_deg, s.rom_max_deg, s.direction);
      float start = init_from_turns_[i];
      float t = start + u * (target - start);
      s.last_ref_turns = t;
      out_turns[i] = t; out_has[i] = true;
    }
    if (elapsed >= (float)INIT_SETTLE_MS) { phase = PH_GAIT; phase01 = 0.0f; }

  } else if (phase == PH_GAIT) {
    phase01 = gait_advance_phase(phase01, dt_s, cps * cps_modifier);
    for (int i = 0; i < PG_NJOINTS; i++) {
      JointState& s = joints[i];
      if (!s.enabled) continue;
      float amp = joint_is_left(i) ? amp_l : amp_r;
      float raw = gait_target_deg(JOINTS[i].kind, joint_is_left(i), phase01, amp);
      float clamped = gait_clamp_to_rom(raw, s.rom_min_deg, s.rom_max_deg);
      float t = gait_deg_to_motor_turns(clamped, s.direction);
      s.last_ref_turns = t;
      out_turns[i] = t; out_has[i] = true;
    }

  } else if (phase == PH_HOMING) {
    float elapsed = (float)(now - phase_started_ms_);
    float u = elapsed / (float)HOMING_RAMP_MS; if (u > 1.0f) u = 1.0f;
    for (int i = 0; i < PG_NJOINTS; i++) {
      JointState& s = joints[i];
      if (!s.enabled) continue;
      float target = gait_home_target_turns(s.home_offset_deg,
                                            s.rom_min_deg, s.rom_max_deg, s.direction);
      float start = homing_from_turns_[i];
      float t = start + u * (target - start);
      s.last_ref_turns = t;
      out_turns[i] = t; out_has[i] = true;
    }
  }
}

bool GaitEngine::all_home_arrived() const {
  for (int i = 0; i < PG_NJOINTS; i++) {
    const JointState& s = joints[i];
    if (!s.enabled) continue;
    float target = gait_home_target_turns(s.home_offset_deg,
                                          s.rom_min_deg, s.rom_max_deg, s.direction);
    if (fabsf(s.last_pos_turns - target) > HOMING_TOL_TURNS) return false;
  }
  return true;
}

bool GaitEngine::homing_timed_out() const {
  return (millis() - phase_started_ms_) >= HOMING_TIMEOUT_MS;
}

void GaitEngine::update_pos(int idx, float pos_turns) {
  if (idx >= 0 && idx < PG_NJOINTS) joints[idx].last_pos_turns = pos_turns;
}
