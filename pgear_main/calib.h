// calib.h — JointCalibration evaluation ONLY (no fitting on-chip).  [Phase 6]
// The PC fits (numpy least-squares) and downloads coefficients via OP_LOAD_COEFFS
// (JointCoeffs in protocol.h). Here we only evaluate:
//   predict(deg, vel) = a*sin(deg) + b*cos(deg) + c*vel + d*sign(vel) + e
// Two models per joint: empty-exo baseline + optional patient passive-load.
#pragma once
#include "protocol.h"
// TODO P6: store JointCoeffs[PG_NJOINTS][2]; float predict(idx, kind, deg, vel).
