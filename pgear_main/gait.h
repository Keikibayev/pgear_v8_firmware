// ============================================================================
// gait.h — gait reference trajectory + phase math (pure functions).  [Phase 2]
// Port of pgear_tools/pi_gui/pgear_pi/gait.py. Bit-identical 50-point pediatric
// reference (Exoskeleton_control_v4.txt hipAngles[]/kneeAngles[]); L-leg is the
// R-leg arrays phase-shifted half a cycle. Allocation-free hot path.
// ============================================================================
#pragma once
#include "constants.h"

static constexpr int PHASE_STEPS = 50;
static constexpr int LEFT_PHASE_OFFSET = 25;   // half cycle

extern const float HIP_DEG[PHASE_STEPS];
extern const float KNEE_DEG[PHASE_STEPS];

// Linear-interp lookup at fractional phase in [0,50). Matches pos_only lerpTraj.
float gait_lerp_traj(const float* arr, float phase_units);

// Global phase [0,1) -> per-joint fractional trajectory index (+L offset).
float gait_joint_phase_units(bool is_left, float phase01);

// Sample one joint's reference [joint-frame deg], amp-scaled. NOT yet ROM-
// clamped or direction-applied (caller applies ROM then direction).
float gait_target_deg(JointKind kind, bool is_left, float phase01, float amp);

// d(theta_ref)/dt at the given phase [deg/s], central difference * cps. Used by
// the AAN gait-direction sign and the torque-mode cooperation gate.
float gait_ref_vel_deg_s(JointKind kind, bool is_left, float phase01, float amp, float cps);

// Soft per-joint ROM clamp (joint-frame deg).
float gait_clamp_to_rom(float deg, float rom_min, float rom_max);

// Final stage: joint-frame deg -> signed motor turns (direction applied last).
float gait_deg_to_motor_turns(float deg, int dir);

// Advance phase by dt at cps cycles/s, wrap at 1.0.
float gait_advance_phase(float phase01, float dt_s, float cps);

// Home / init pose targets [motor turns] (ROM-clamped, fixes firmware bypass).
float gait_home_target_turns(float home_offset_deg, float rom_min, float rom_max, int dir);
float gait_init_target_turns(float init_pos_deg, float home_offset_deg,
                             float rom_min, float rom_max, int dir);
