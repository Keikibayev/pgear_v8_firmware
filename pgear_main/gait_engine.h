// ============================================================================
// gait_engine.h — gait state machine + per-joint setpoints.  [Phase 2]
// Port of pgear_tools/pi_gui/pgear_pi/control/gait_engine.py.
//   IDLE -> (start) -> INIT_SETTLE (2s ramp) -> GAIT
//   any  -> (home)  -> HOMING (ramp) -> IDLE
// tick() is called at GAIT_STEP_HZ; caller pushes the returned motor-turn
// setpoints over CAN. Uses millis() for INIT/HOMING timing.
// ============================================================================
#pragma once
#include "constants.h"

enum GaitPhaseE : uint8_t { PH_IDLE = 0, PH_INIT_SETTLE, PH_GAIT, PH_HOMING };

struct JointState {
  int8_t  direction      = 1;
  bool    enabled        = true;
  float   rom_min_deg    = 0.0f;
  float   rom_max_deg    = 0.0f;
  float   init_pos_deg   = 0.0f;
  float   home_offset_deg = 0.0f;
  float   last_ref_turns = 0.0f;
  float   last_pos_turns = 0.0f;   // latest telemetry, for homing-arrival check
};

class GaitEngine {
 public:
  GaitPhaseE phase = PH_IDLE;
  float phase01      = 0.0f;
  float cps          = DEFAULT_GAIT_CPS;
  float amp_r        = DEFAULT_AMP_R;
  float amp_l        = DEFAULT_AMP_L;
  float cps_modifier = 1.0f;        // AAN sets this (1=normal, 0=paused)
  JointState joints[PG_NJOINTS];

  void init_defaults();             // ROM/dir/init-pose per joint from constants

  // transitions
  void start_gait();
  void stop_gait();
  void start_homing();
  void go_idle();

  // per-tick: fills out_turns[i] + out_has[i]=true for enabled joints w/ a target.
  void tick(float dt_s, float out_turns[PG_NJOINTS], bool out_has[PG_NJOINTS]);

  bool all_home_arrived() const;
  bool homing_timed_out() const;
  void update_pos(int idx, float pos_turns);

 private:
  uint32_t phase_started_ms_ = 0;
  float    init_from_turns_[PG_NJOINTS]   = {0};
  float    homing_from_turns_[PG_NJOINTS] = {0};
};
