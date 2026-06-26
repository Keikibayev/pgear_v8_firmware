// ============================================================================
// safety.cpp — safety supervisor.  [Phase 5]
// ============================================================================
#include "safety.h"
#include <Arduino.h>
#include <math.h>

// sensor-watchdog state
static float    s_prevTorque[PG_NJOINTS] = {0};
static bool     s_prevValid = false;
static uint32_t s_prevMs = 0;
static uint32_t s_glitchWindowStart = 0;
static int      s_glitchCount = 0;

void safety_init() {
#if USE_ESTOP_BUTTON
#if ESTOP_ACTIVE_LOW
  pinMode(ESTOP_GPIO, INPUT_PULLUP);
#else
  pinMode(ESTOP_GPIO, INPUT_PULLDOWN);
#endif
  Serial.printf("[safety] init: HW e-stop on GPIO%d; NO endstops (firmware envelope is the hard limit)\n",
                ESTOP_GPIO);
#else
  Serial.println("[safety] init: HW e-stop button DISABLED (USE_ESTOP_BUTTON=0)");
#endif
#if !SAFETY_AUTO_ESTOP
  Serial.println("[safety] *** AUTO E-STOP DISABLED (SAFETY_AUTO_ESTOP=0, dev only) *** "
                 "no auto-trips; set 1 + wire e-stop before patient use");
#endif
  s_prevMs = millis();
  s_glitchWindowStart = millis();
}

bool safety_estop_pressed() {
  int v = digitalRead(ESTOP_GPIO);
#if ESTOP_ACTIVE_LOW
  // NC button to GND with pullup: closed (released) = LOW; open (pressed) = HIGH.
  return (v == HIGH);
#else
  return (v == HIGH);
#endif
}

float safety_clamp_turns(int idx, float turns) {
  if (idx < 0 || idx >= PG_NJOINTS) return turns;
  float lo, hi;
  joint_turn_limits(idx, &lo, &hi);   // direction-aware (mirrors for R-leg)
  if (turns < lo) return lo;
  if (turns > hi) return hi;
  return turns;
}

bool safety_tick(bool armed, bool running,
                 const BusTelemetry* snap, const CoprocData* cd,
                 uint8_t* out_crossCheck, uint8_t* out_hbErr) {
  uint8_t hbErr = 0, crossCheck = 0;
  bool trip = false;

  // 1) hardware E-STOP button (highest priority) — dev-disablable
#if USE_ESTOP_BUTTON
  if (safety_estop_pressed()) trip = true;
#endif

  // 2) ODrive heartbeat watchdog + position-envelope overrun — once armed.
  if (armed && snap) {
    for (int i = 0; i < PG_NJOINTS; i++) {
      if (!snap->j[i].fb_valid) { hbErr |= (1 << i); trip = true; }
      float lo, hi;
      joint_turn_limits(i, &lo, &hi);   // direction-aware (mirrors for R-leg)
      lo -= ENV_OVERRUN_MARGIN_TURNS;
      hi += ENV_OVERRUN_MARGIN_TURNS;
      if (snap->j[i].fb_valid &&
          (snap->j[i].pos_turns < lo || snap->j[i].pos_turns > hi)) trip = true;
    }
  }

  // 3) sensor watchdog (torque rate cap + glitch-rate) — only if coproc online
  uint32_t now = millis();
  if (cd && cd->online) {
    float dt = (now - s_prevMs) / 1000.0f;
    if (s_prevValid && dt > 0.0f) {
      for (int i = 0; i < PG_NJOINTS; i++) {
        float rate = fabsf(cd->torqueNm[i] - s_prevTorque[i]) / dt;
        if (rate > SENSOR_MAX_RATE_NM_PER_S) s_glitchCount++;
      }
    }
    for (int i = 0; i < PG_NJOINTS; i++) s_prevTorque[i] = cd->torqueNm[i];
    s_prevValid = true;
  } else {
    s_prevValid = false;
  }
  s_prevMs = now;
  if (now - s_glitchWindowStart >= 1000) {
    if (s_glitchCount > SENSOR_GLITCH_ESTOP_PER_S) trip = true;
    s_glitchCount = 0;
    s_glitchWindowStart = now;
  }

  // 4) cross-check iq-effort vs load cell torque — warn only (no trip)
  if (cd && cd->online && snap) {
    for (int i = 0; i < PG_NJOINTS; i++) {
      float iqTorque = snap->j[i].iq_measured_a * JOINT_NM_PER_A;
      if (fabsf(iqTorque - cd->torqueNm[i]) > CROSSCHECK_DIVERGE_NM)
        crossCheck |= (1 << i);
    }
  }

  if (out_crossCheck) *out_crossCheck = crossCheck;
  if (out_hbErr)      *out_hbErr = hbErr;
#if !SAFETY_AUTO_ESTOP
  trip = false;   // dev: automatic e-stop disabled (manual GUI/CMD e-stop still works)
#endif
  return trip;
}
