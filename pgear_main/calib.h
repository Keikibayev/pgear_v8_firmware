// ============================================================================
// calib.h — JointCalibration evaluation ONLY (no fitting on-chip).  [Phase 4/6]
// The PC fits (numpy least-squares) and downloads coefficients via
// OP_LOAD_COEFFS (JointCoeffs in protocol.h). Here we only STORE + EVALUATE:
//   predict(deg,vel) = a*sin(deg) + b*cos(deg) + c*vel + d*sign(vel) + e
// Two models per joint: empty-exo baseline + optional patient passive-load.
// (Storage + predict land in Phase 4 so LOAD_COEFFS works end-to-end; the AAN
//  consumer is Phase 6.)
// ============================================================================
#pragma once
#include "protocol.h"

void  calib_load_coeffs(const JointCoeffs* c);   // store one model (from PC)
bool  calib_has(int joint, PgCoeffKind kind);
float calib_predict(int joint, PgCoeffKind kind, float deg, float vel);
float calib_resid_std(int joint, PgCoeffKind kind);
