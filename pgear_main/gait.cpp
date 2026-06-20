// ============================================================================
// gait.cpp — gait trajectory + phase math.  Port of gait.py.  [Phase 2]
// ============================================================================
#include "gait.h"

// 50-point reference, joint-frame degrees, R-leg phase (gait.py:19-33).
const float HIP_DEG[PHASE_STEPS] = {
   10.08f,  8.65f,  7.44f,  6.38f,  5.42f,  4.39f,  3.40f,  2.55f,  1.77f,  1.16f,
    0.65f,  0.11f, -0.56f, -1.47f, -2.49f, -3.47f, -4.28f, -4.99f, -5.58f, -6.16f,
   -6.64f, -7.01f, -7.16f, -7.00f, -6.53f, -5.84f, -4.93f, -3.79f, -2.47f, -1.01f,
    0.37f,  1.57f,  2.60f,  3.61f,  4.67f,  5.78f,  6.97f,  8.15f,  9.30f, 10.52f,
   11.69f, 12.78f, 13.62f, 14.14f, 14.35f, 14.31f, 14.14f, 13.85f, 13.28f, 12.29f,
};

const float KNEE_DEG[PHASE_STEPS] = {
   10.62f,  9.94f,  9.46f,  9.18f,  9.11f,  9.05f,  8.85f,  8.45f,  7.85f,  7.05f,
    6.11f,  5.18f,  4.37f,  3.75f,  3.29f,  2.97f,  2.77f,  2.58f,  2.40f,  2.21f,
    2.05f,  1.97f,  2.05f,  2.42f,  3.08f,  4.20f,  5.75f,  7.69f,  9.90f, 12.33f,
   14.81f, 17.17f, 19.23f, 20.86f, 22.06f, 22.70f, 22.93f, 22.81f, 22.42f, 21.91f,
   21.31f, 20.70f, 20.05f, 19.32f, 18.41f, 17.30f, 16.10f, 14.83f, 13.47f, 12.21f,
};

float gait_lerp_traj(const float* arr, float phase_units) {
  int i0 = (int)phase_units;
  if (i0 < 0) i0 = 0;
  if (i0 > 49) i0 = 49;
  int i1 = (i0 + 1) % PHASE_STEPS;
  float frac = phase_units - (float)i0;
  return arr[i0] * (1.0f - frac) + arr[i1] * frac;
}

float gait_joint_phase_units(bool is_left, float phase01) {
  float units = phase01 * PHASE_STEPS;
  if (is_left) {
    units += LEFT_PHASE_OFFSET;
    while (units >= PHASE_STEPS) units -= PHASE_STEPS;
  }
  return units;
}

float gait_target_deg(JointKind kind, bool is_left, float phase01, float amp) {
  const float* arr = (kind == KIND_HIP) ? HIP_DEG : KNEE_DEG;
  float u = gait_joint_phase_units(is_left, phase01);
  return amp * gait_lerp_traj(arr, u);
}

float gait_ref_vel_deg_s(JointKind kind, bool is_left, float phase01, float amp, float cps) {
  const float eps = 0.01f;
  float p1 = phase01 + eps; while (p1 >= 1.0f) p1 -= 1.0f;
  float p0 = phase01 - eps; while (p0 < 0.0f)  p0 += 1.0f;
  float r1 = gait_target_deg(kind, is_left, p1, amp);
  float r0 = gait_target_deg(kind, is_left, p0, amp);
  return ((r1 - r0) / (2.0f * eps)) * cps;
}

float gait_clamp_to_rom(float deg, float rom_min, float rom_max) {
  if (deg < rom_min) return rom_min;
  if (deg > rom_max) return rom_max;
  return deg;
}

float gait_deg_to_motor_turns(float deg, int dir) {
  return (float)dir * deg_to_turns(deg);
}

float gait_advance_phase(float phase01, float dt_s, float cps) {
  phase01 += dt_s * cps;
  while (phase01 >= 1.0f) phase01 -= 1.0f;
  return phase01;
}

float gait_home_target_turns(float home_offset_deg, float rom_min, float rom_max, int dir) {
  float c = gait_clamp_to_rom(home_offset_deg, rom_min, rom_max);
  return gait_deg_to_motor_turns(c, dir);
}

float gait_init_target_turns(float init_pos_deg, float home_offset_deg,
                             float rom_min, float rom_max, int dir) {
  float deg = gait_clamp_to_rom(init_pos_deg + home_offset_deg, rom_min, rom_max);
  return gait_deg_to_motor_turns(deg, dir);
}
