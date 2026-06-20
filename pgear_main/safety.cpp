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
#if ESTOP_ACTIVE_LOW
  pinMode(ESTOP_GPIO, INPUT_PULLUP);
#else
  pinMode(ESTOP_GPIO, INPUT_PULLDOWN);
#endif
  s_prevMs = millis();
  s_glitchWindowStart = millis();
  Serial.printf("[safety] init (estop GPIO%d, NO endstops -> firmware envelope is the hard limit)\n",
                ESTOP_GPIO);
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
  bool hip = (JOINTS[idx].kind == KIND_HIP);
  float lo = hip ? HIP_TURN_MIN : KNEE_TURN_MIN;
  float hi = hip ? HIP_TURN_MAX : KNEE_TURN_MAX;
  if (turns < lo) return lo;
  if (turns > hi) return hi;
  return turns;
}

bool safety_tick(bool armed, bool running,
                 const BusTelemetry* snap, const CoprocData* cd,
                 uint8_t* out_crossCheck, uint8_t* out_hbErr) {
  uint8_t hbErr = 0, crossCheck = 0;
  bool trip = false;

  // 1) hardware E-STOP button (highest priority)
  if (safety_estop_pressed()) trip = true;

  // 2) ODrive heartbeat watchdog — only meaningful once armed
  if (armed && snap) {
    for (int i = 0; i < PG_NJOINTS; i++) {
      // (enabled-ness lives in the engine; treat any stale axis as a fault here)
      if (!snap->j[i].fb_valid) { hbErr |= (1 << i); trip = true; }
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

  // 4) cross-check iq-effort vs FUTEK torque — warn only (no trip)
  if (cd && cd->online && snap) {
    for (int i = 0; i < PG_NJOINTS; i++) {
      float iqTorque = snap->j[i].iq_measured_a * JOINT_NM_PER_A;
      if (fabsf(iqTorque - cd->torqueNm[i]) > CROSSCHECK_DIVERGE_NM)
        crossCheck |= (1 << i);
    }
  }

  if (out_crossCheck) *out_crossCheck = crossCheck;
  if (out_hbErr)      *out_hbErr = hbErr;
  return trip;
}
