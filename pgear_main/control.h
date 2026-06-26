// ============================================================================
// control.h — the two control-law policies over one shared core.  [Phase 6/6b]
// A controlMode field (g_controlMode; LP_FLAG_TORQUE_MODE) selects per tick.
//
// Position mode (Phase 6, port of worker.py): POS_FILTER + AAN-by-iq
//   phase-yielding. patient_nm = (iq_meas - predict_empty - predict_passive)
//   * JOINT_NM_PER_A; resistance = -gait_dir * patient_nm; worst-joint global
//   yield -> cps_modifier; assist-level scales thresholds; asymmetric LPF.
//
// Torque mode (Phase 6b, port of labs/torque_gait.py): moving-reference
//   impedance + cooperation gate; outputs ODrive motor input_torque per joint.
// ============================================================================
#pragma once
#include "gait_engine.h"
#include "can_odrive.h"
#include "coproc_link.h"

// Per-joint patient torque [Nm] (+ = resisting the gait). For telemetry/AAN.
struct PatientTorque {
  float nm[PG_NJOINTS]   = {0};
  bool  valid[PG_NJOINTS] = {false};
};

// Compute patient torque from iq residual vs the downloaded models.
void control_patient_torque(const BusTelemetry* snap, const CoprocData* cd,
                            const GaitEngine* eng, PatientTorque* out);

// Position-mode AAN: update eng->cps_modifier from patient resistance + assist.
// `aan_on` gates it (off -> modifier ramps back to 1.0). Returns the factor.
float control_pos_aan(float dt_s, const GaitEngine* eng, const PatientTorque* pt,
                      bool aan_on, float assist_level);

// Torque-mode impedance step. Advances its own phase via the cooperation gate;
// fills out_motor_nm[i] (ODrive input_torque) for enabled joints. `started`
// gates phase advance (false -> gravity-comp/float only). `free_run` makes the
// phase self-advance at cps (gate ignored, BENCH self-walk). `assist_gain`
// scales the assist spring K_assist (the "go"); 1.0 = config defaults.
struct TorqueState {
  float phase01 = 0.0f;
  float g_filt  = 1.0f;
  float adapt   = 1.0f;   // adaptive-assist factor [FLOOR,1] (AAN); 1 = full gain
  float prev_tau[PG_NJOINTS] = {0};
  float rev_timer    = 0.0f;  // how long the reverse condition has held [s]
  float mean_lag_deg = 0.0f;  // last tick's mean |ref-pos| tracking lag (position error)
};
// `aan_on` enables adaptive assist-as-needed. `st->adapt` (which scales the
// therapist gain ceiling) grows on the GREATER of gait-tracking lag (error) and
// the patient's effort deficit (from `pt`), and fades as the patient keeps up
// AND contributes. When AAN is on the device also DRIVES a passive patient
// (advances the phase by effort-need). `cap_mult` scales the per-joint torque
// caps (live GUI control). When AAN is off, adapt is held at 1.0 (manual gain).
// `limb_hip_nm` = manual hip limb-weight feed-forward (peak Nm), therapist-set,
// for patients who can't be characterized. Cancels the constant limb weight.
// `knee_assist_nm` = knee swing-assist feed-forward (constant Nm kick in the gait
// swing direction; no knee gravity model needed).
void control_torque_step(float dt_s, bool started, bool free_run, bool allow_reverse,
                         bool aan_on,
                         float assist_gain, float cap_mult, float vel_limit_mult,
                         float limb_hip_nm,
                         float knee_assist_nm, float cps_base,
                         const BusTelemetry* snap, const CoprocData* cd,
                         const PatientTorque* pt, const GaitEngine* eng,
                         TorqueState* st,
                         float out_motor_nm[PG_NJOINTS], bool out_has[PG_NJOINTS]);
