// ============================================================================
// control.cpp — position-mode AAN + torque-mode impedance.  [Phase 6/6b]
// Ports of pi_gui worker._update_aan and labs/torque_gait.TorqueGaitController.
// ============================================================================
#include "control.h"
#include "gait.h"
#include "calib.h"
#include <Arduino.h>
#include <math.h>

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline float signf(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }

// joint-frame deg/vel from motor telemetry (direction applied)
static inline float jdeg(const GaitEngine* e, const AxisTelemetry& a, int i) {
  return e->joints[i].direction * turns_to_deg(a.pos_turns);
}
static inline float jvel(const GaitEngine* e, const AxisTelemetry& a, int i) {
  return e->joints[i].direction * turns_to_deg(a.vel_turns_s);
}

// ============================================================================
// Shared: patient torque = (iq_meas - empty - passive) * JOINT_NM_PER_A
// ============================================================================
void control_patient_torque(const BusTelemetry* snap, const CoprocData* cd,
                            const GaitEngine* eng, PatientTorque* out) {
  (void)cd;
  for (int i = 0; i < PG_NJOINTS; i++) {
    out->valid[i] = false; out->nm[i] = 0.0f;
    if (!eng->joints[i].enabled) continue;
    const AxisTelemetry& a = snap->j[i];
    if (!a.fb_valid) continue;
    if (!calib_has(i, COEFF_EMPTY)) continue;       // need a baseline to subtract
    // Model convention (matches calibrator.py + the JSON baselines + worker.py):
    //   deg = joint-frame degrees (direction applied); vel = RAW motor turns/s
    //   (NOT direction-applied, NOT deg/s). The fit and predict must agree on this.
    float deg = jdeg(eng, a, i);
    float vel = a.vel_turns_s;                       // raw turns/s, per the fit
    float iq_pred = calib_predict(i, COEFF_EMPTY, deg, vel)
                  + calib_predict(i, COEFF_PATIENT_PASSIVE, deg, vel);
    out->nm[i] = (a.iq_measured_a - iq_pred) * JOINT_NM_PER_A;
    out->valid[i] = true;
  }
}

// ============================================================================
// Position-mode AAN — worst-joint phase-yielding (port of _update_aan)
// ============================================================================
float control_pos_aan(float dt_s, const GaitEngine* eng, const PatientTorque* pt,
                      bool aan_on, float assist_level) {
  static float s_factor = 1.0f;

  // assist-level -> (lo, hi) yield thresholds
  float a = clampf(assist_level, 0.0f, 1.0f);
  float lo = AAN_THRESHOLD_AT_MIN_ASSIST_NM
           + a * (AAN_THRESHOLD_AT_MAX_ASSIST_NM - AAN_THRESHOLD_AT_MIN_ASSIST_NM);
  float hi = lo + AAN_YIELD_SPAN_NM;

  float target = 1.0f;
  if (aan_on) {
    // factor yields off the WORST (highest-resistance) joint == min per-joint factor
    for (int i = 0; i < PG_NJOINTS; i++) {
      if (!pt->valid[i]) continue;
      bool L = joint_is_left(i);
      float slope = gait_ref_vel_deg_s(JOINTS[i].kind, L, eng->phase01, 1.0f, 1.0f);
      int gait_dir = (slope >= 0.0f) ? 1 : -1;
      float resistance = -(float)gait_dir * pt->nm[i];   // + = fighting the gait
      float f;
      if (resistance <= lo)      f = 1.0f;
      else if (resistance >= hi) f = 0.0f;
      else                       f = 1.0f - (resistance - lo) / (hi - lo);
      if (f < target) target = f;
    }
  }

  // asymmetric LPF: slow to yield (target<factor), quick to resume
  float tc = (target < s_factor) ? AAN_FACTOR_ENGAGE_TC_S : AAN_FACTOR_RELEASE_TC_S;
  float alpha = (tc > 0.0f) ? (1.0f - expf(-dt_s / tc)) : 1.0f;
  s_factor += alpha * (target - s_factor);
  return s_factor;
}

// ============================================================================
// Torque-mode impedance (port of TorqueGaitController.step)
// ============================================================================
static float tq_cap(int i, float mult) {
  float base = (JOINTS[i].kind == KIND_HIP) ? MAX_HIP_TORQUE_NM : MAX_KNEE_TORQUE_NM;
  return base * mult;     // mult = live GUI "torque cap x" (1.0 = safe defaults)
}

// static gravity-hold torque (joint frame): full exo + alpha*patient passive +
// manual therapist limb-weight feed-forward.
static float tq_grav_nm(const GaitEngine* eng, int i, float deg, float limb_hip_nm) {
  int dir = eng->joints[i].direction;
  float tau;
  if (calib_has(i, COEFF_EMPTY))
    tau = dir * JOINT_NM_PER_A * calib_predict(i, COEFF_EMPTY, deg, 0.0f);
  else {
    float k = (JOINTS[i].kind == KIND_HIP) ? TQ_K_HIP_FALLBACK_NM : TQ_K_KNEE_FALLBACK_NM;
    tau = k * sinf(deg * (float)M_PI / 180.0f);
  }
  if (calib_has(i, COEFF_PATIENT_PASSIVE))
    tau += dir * JOINT_NM_PER_A * TQ_PATIENT_ASSIST_FRAC
         * calib_predict(i, COEFF_PATIENT_PASSIVE, deg, 0.0f);
  // Manual limb-weight feed-forward (peak Nm at horizontal, therapist-set): for
  // CP patients who can't relax to be characterized. Cancels the *constant* limb
  // weight so the assist spring can reach the target; tone is left for the
  // compliant AAN. HIP ONLY -- the knee's gravity couples to the hip angle
  // (2-link), so sin(knee) alone is wrong; needs a coupled model (TODO).
  if (JOINTS[i].kind == KIND_HIP)
    tau += dir * limb_hip_nm * sinf(deg * (float)M_PI / 180.0f);
  return tau;
}

static float tq_rom_nm(const GaitEngine* eng, int i, float deg, float vel) {
  float lo = eng->joints[i].rom_min_deg, hi = eng->joints[i].rom_max_deg;
  float m = TQ_ROM_MARGIN_DEG, k = TQ_KWALL_NM_DEG, b = TQ_BWALL_NM_S_DEG;
  if (deg > hi - m) return -k * (deg - (hi - m)) - b * (vel > 0 ? vel : 0.0f);
  if (deg < lo + m) return  k * ((lo + m) - deg) - b * (vel < 0 ? vel : 0.0f);
  return 0.0f;
}

void control_torque_step(float dt_s, bool started, bool free_run, bool aan_on,
                         float assist_gain, float cap_mult, float limb_hip_nm,
                         float cps_base, const BusTelemetry* snap, const CoprocData* cd,
                         const PatientTorque* pt, const GaitEngine* eng,
                         TorqueState* st,
                         float out_motor_nm[PG_NJOINTS], bool out_has[PG_NJOINTS]) {
  (void)cd;
  for (int i = 0; i < PG_NJOINTS; i++) out_has[i] = false;

  // any enabled joint with stale feedback -> zero all (safe)
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (eng->joints[i].enabled && !snap->j[i].fb_valid) {
      for (int j = 0; j < PG_NJOINTS; j++) st->prev_tau[j] = 0.0f;
      return;
    }
  }

  // cooperation gate g = sum_hips(vel*sign(v_ref)) / sum_hips(|v_ref|)
  float num = 0.0f, den = 0.0f;
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!eng->joints[i].enabled || JOINTS[i].kind != KIND_HIP) continue;
    bool L = joint_is_left(i);
    float v_ref = gait_ref_vel_deg_s(KIND_HIP, L, st->phase01,
                                     L ? eng->amp_l : eng->amp_r, cps_base);
    if (fabsf(v_ref) < TQ_GATE_VREF_FLOOR_DEG_S) continue;
    float vel = jvel(eng, snap->j[i], i);
    num += vel * signf(v_ref);
    den += fabsf(v_ref);
  }
  float g_raw = (den > 0.0f) ? clampf(num / den, TQ_GATE_MIN, 1.0f) : st->g_filt;
  // asymmetric LPF (slow to yield, quick to resume)
  float tc = (g_raw < st->g_filt) ? AAN_FACTOR_ENGAGE_TC_S : AAN_FACTOR_RELEASE_TC_S;
  float alpha = (tc > 0.0f) ? (1.0f - expf(-dt_s / tc)) : 1.0f;
  st->g_filt += alpha * (g_raw - st->g_filt);

  // cooperation factor: 1 when cooperating/forward, ->0 as the patient drives
  // AGAINST the gait. Gates both the assist ramp and the device-drive below.
  float coop = clampf((st->g_filt - TQ_REVERSE_ENABLE) / (0.0f - TQ_REVERSE_ENABLE),
                      0.0f, 1.0f);

  // EFFORT-based need: how little the patient contributes in the gait direction.
  // help_i = gait_dir * patient_torque (>0 = pushing with the gait); effort_need
  // is 1 when passive (no help), 0 when contributing >= TQ_EFFORT_SCALE_NM.
  float help_sum = 0.0f; int help_n = 0;
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!eng->joints[i].enabled || !pt->valid[i]) continue;
    bool L = joint_is_left(i);
    float v_ref = gait_ref_vel_deg_s(JOINTS[i].kind, L, st->phase01,
                                     L ? eng->amp_l : eng->amp_r, cps_base);
    float gait_dir = (v_ref >= 0.0f) ? 1.0f : -1.0f;
    // help>0 = the patient REDUCES the motor's required torque (genuinely
    // assisting). A passive load / resistance INCREASES motor torque -> reads
    // negative -> effort_need high -> the device assists MORE (correct).
    help_sum -= gait_dir * pt->nm[i];
    help_n++;
  }
  // No baseline on any enabled joint -> can't measure effort -> fall back to
  // error-only (effort_need 0), safer than assuming the patient is passive.
  float effort_need = 0.0f;
  if (help_n > 0)
    effort_need = clampf(1.0f - (help_sum / (float)help_n) / TQ_EFFORT_SCALE_NM,
                         0.0f, 1.0f);

  // advance shared phase (hold band; reverse only on sustained backward motion).
  // free_run (BENCH) marches at full cadence. With AAN on, the device also DRIVES
  // a passive patient: advance up to TQ_DRIVE_MAX*cadence in proportion to how
  // passive they are (effort_need), gated by cooperation so it never drives into
  // a deviation. As the patient contributes, effort_need falls and they lead.
  if (started) {
    float g = st->g_filt;
    float gate_phase = free_run ? 1.0f
                  : ((g > TQ_GATE_HOLD_BAND) ? g : (g < TQ_REVERSE_ENABLE ? g : 0.0f));
    float drive = (aan_on && !free_run) ? (effort_need * coop * TQ_DRIVE_MAX) : 0.0f;
    // Drive only ADDS forward motion; never override a reversal (negative gate).
    float g_phase = gate_phase;
    if (gate_phase >= 0.0f && drive > gate_phase) g_phase = drive;
    st->phase01 += dt_s * cps_base * g_phase;
    while (st->phase01 >= 1.0f) st->phase01 -= 1.0f;
    while (st->phase01 < 0.0f)  st->phase01 += 1.0f;
  }

  // Effective assist gain = therapist ceiling * adaptive factor (last tick's;
  // a 1-tick lag at 50 Hz is negligible). adapt is updated below from this
  // tick's mean tracking lag.
  float eff_gain = assist_gain * st->adapt;

  // per-joint torque law (accumulate the gait-tracking lag for the AAN update)
  float lag_sum = 0.0f; int lag_n = 0;
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!eng->joints[i].enabled) continue;
    bool L = joint_is_left(i);
    float deg = jdeg(eng, snap->j[i], i);
    float vel = jvel(eng, snap->j[i], i);
    float ref = gait_target_deg(JOINTS[i].kind, L, st->phase01, L ? eng->amp_l : eng->amp_r);

    float t_grav = tq_grav_nm(eng, i, deg, limb_hip_nm);
    float err = clampf(ref - deg, -TQ_ASSIST_SAT_DEG, TQ_ASSIST_SAT_DEG);
    lag_sum += fabsf(err); lag_n++;
    float kA = (JOINTS[i].kind == KIND_HIP) ? TQ_KASSIST_HIP_NM_DEG : TQ_KASSIST_KNEE_NM_DEG;
    float t_assist = eff_gain * kA * err;      // adaptive (or full) assist spring
    float bD = (JOINTS[i].kind == KIND_HIP) ? TQ_BDAMP_HIP_NM_S_DEG : TQ_BDAMP_KNEE_NM_S_DEG;
    float t_damp = -bD * vel;
    float t_rom = tq_rom_nm(eng, i, deg, vel);

    float tau = t_grav + t_assist + t_damp + t_rom;
    float cap = tq_cap(i, cap_mult);
    tau = clampf(tau, -cap, cap);
    // slew-rate limit
    float step = TQ_TAU_RATE_NM_S * dt_s;
    tau = st->prev_tau[i] + clampf(tau - st->prev_tau[i], -step, step);
    st->prev_tau[i] = tau;

    out_motor_nm[i] = eng->joints[i].direction * tau * DRIVE_NM_PER_JOINT_NM;
    out_has[i] = true;
  }

  // Adaptive assist-as-needed (with forgetting): grow `adapt` when the leg lags
  // the gait pattern, fade it when the patient keeps up. When AAN is off, hold
  // at 1.0 so the therapist's gain is used directly.
  if (aan_on && started) {
    float lag = (lag_n > 0) ? (lag_sum / (float)lag_n) : 0.0f;
    float e_norm = clampf(lag / TQ_ADAPT_ERR_SCALE_DEG, 0.0f, 1.0f);
    // Need = the greater of "lagging the pattern" (error-based) and "not
    // contributing" (effort-based). coop (above) suppresses growth during a
    // deviation; the forget term still fades the assist when the patient leads.
    float need = fmaxf(e_norm, effort_need);
    st->adapt += dt_s * (TQ_ADAPT_K_UP * need * coop - TQ_ADAPT_K_FORGET * st->adapt);
    st->adapt = clampf(st->adapt, TQ_ADAPT_FLOOR, 1.0f);
  } else {
    st->adapt = 1.0f;
  }
}
