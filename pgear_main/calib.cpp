// ============================================================================
// calib.cpp — store + evaluate downloaded joint models.  [Phase 4]
// ⚠ The sin/cos basis MUST match calibrator.py's fit convention. Verify deg-vs-
// radians against fit_joint_model() before trusting AAN in Phase 6; coefficients
// are meaningless if the basis differs. Implemented here as sin/cos of RADIANS.
// ============================================================================
#include "calib.h"
#include <math.h>

static JointCoeffs s_models[PG_NJOINTS][2];   // [joint][COEFF_EMPTY | COEFF_PATIENT_PASSIVE]
static bool        s_has[PG_NJOINTS][2] = {{false}};

static inline int kindIdx(PgCoeffKind k) { return (k == COEFF_PATIENT_PASSIVE) ? 1 : 0; }

void calib_load_coeffs(const JointCoeffs* c) {
  if (!c || c->joint >= PG_NJOINTS) return;
  int k = kindIdx((PgCoeffKind)c->kind);
  s_models[c->joint][k] = *c;
  s_has[c->joint][k] = true;
}

bool calib_has(int joint, PgCoeffKind kind) {
  if (joint < 0 || joint >= PG_NJOINTS) return false;
  return s_has[joint][kindIdx(kind)];
}

float calib_predict(int joint, PgCoeffKind kind, float deg, float vel) {
  if (!calib_has(joint, kind)) return 0.0f;
  const float* a = s_models[joint][kindIdx(kind)].coef;   // a,b,c,d,e
  float r = deg * (float)M_PI / 180.0f;
  float s = (vel > 0.0f) ? 1.0f : (vel < 0.0f ? -1.0f : 0.0f);
  return a[0] * sinf(r) + a[1] * cosf(r) + a[2] * vel + a[3] * s + a[4];
}

float calib_resid_std(int joint, PgCoeffKind kind) {
  if (!calib_has(joint, kind)) return 0.0f;
  return s_models[joint][kindIdx(kind)].residStdA;
}
