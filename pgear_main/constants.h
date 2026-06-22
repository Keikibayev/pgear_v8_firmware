// ============================================================================
// constants.h — firmware-mirror constants (port of pi_gui constants.py)
// Single source of truth for mechanical scaling, CAN protocol, ODrive enums,
// node ids, and the joint table. Reconciled against v7.1.5 / pos_only firmware.
// ============================================================================
#pragma once
#include <stdint.h>
#include <math.h>
#include "protocol.h"   // PG_NJOINTS

// ---- Mechanical scaling (pos_only.ino:131-135) -----------------------------
static constexpr float GEAR_RATIO   = 64.0f;
static constexpr float KEF          = 1.5f;
static constexpr float TURNS_PER_DEG = GEAR_RATIO * KEF / 360.0f;  // ~0.2667
static constexpr float DEG_PER_TURN  = 1.0f / TURNS_PER_DEG;       // 3.75
static inline float deg_to_turns(float d) { return d * TURNS_PER_DEG; }
static inline float turns_to_deg(float t) { return t / TURNS_PER_DEG; }

// ---- CAN bus ---------------------------------------------------------------
static constexpr uint32_t CAN_BITRATE   = 250000;   // 250 kbps (TWAI_TIMING_CONFIG_250KBITS)
static constexpr int      CAN_CMD_BITS  = 5;        // id = (node<<5)|cmd
static constexpr int      CAN_CMD_MASK  = 0x1F;

// RESOLVED (verified against Waveshare docs): on the ESP32-S3-Touch-LCD-7 the
// CAN transceiver shares GPIO19/20 (the native-USB D-/D+ pins) via an
// FSUSB42UMX switch selected by CH422G EXIO5 (USB_SEL). EXIO5=HIGH -> CAN.
// We use CAN on 19/20 (see board_io.cpp) and run the PC link over the board's
// SEPARATE "USB TO UART" CH343P port (UART0, GPIO43/44) — so set the Arduino
// option "USB CDC On Boot: DISABLED" so `Serial` maps to that port, and flash
// over it too (native USB is off while EXIO5=CAN).
#ifndef CAN_RX_PIN
#define CAN_RX_PIN 19    // CANRX (Waveshare pinout)
#endif
#ifndef CAN_TX_PIN
#define CAN_TX_PIN 20    // CANTX (Waveshare pinout)
#endif

static constexpr float    ODRIVE_WD_TIMEOUT_S = 0.5f;
static constexpr float    ODRIVE_WD_FEED_HZ   = 10.0f;

// ---- ODrive CAN command IDs (subset we use) --------------------------------
enum CanCmd : uint8_t {
  CMD_HEARTBEAT        = 0x001,  // RX: axis_error u32, axis_state u8
  CMD_MOTOR_ERROR      = 0x003,  // RX: error u32
  CMD_ENC_ERROR        = 0x004,  // RX: error u32
  CMD_SET_AXIS_STATE   = 0x007,  // TX: state u32
  CMD_GET_ENC_EST      = 0x009,  // RX: pos f32, vel f32
  CMD_SET_CTRL_MODES   = 0x00B,  // TX: control_mode u32, input_mode u32
  CMD_SET_INPUT_POS    = 0x00C,  // TX: pos f32, vel_ff i16, torque_ff i16
  CMD_SET_INPUT_TORQUE = 0x00E,  // TX: torque f32
  CMD_SET_LIMITS       = 0x00F,  // TX: vel_lim f32, cur_lim f32
  CMD_SET_TRAJ_VEL     = 0x011,  // TX: traj_vel f32
  CMD_SET_TRAJ_ACCEL   = 0x012,  // TX: accel f32, decel f32
  CMD_GET_IQ           = 0x014,  // RX: iq_setpoint f32, iq_measured f32
  CMD_CLEAR_ERRORS     = 0x018,  // TX: (empty)
  CMD_CONTROLLER_ERR   = 0x01D,  // RX: error u32
};

// ---- ODrive enums ----------------------------------------------------------
enum AxisState : uint32_t { AXIS_IDLE = 1, AXIS_FULL_CAL = 3, AXIS_CLOSED_LOOP = 8 };
enum ControlMode : uint32_t { CM_TORQUE = 1, CM_VELOCITY = 2, CM_POSITION = 3 };
enum InputMode : uint32_t { IM_PASSTHROUGH = 1, IM_POS_FILTER = 3, IM_TRAP_TRAJ = 5 };

// ---- Joint table (canonical order: 0=R-hip,1=R-knee,2=L-hip,3=L-knee) ------
enum JointKind : uint8_t { KIND_HIP = 0, KIND_KNEE = 1 };
struct JointDef { uint8_t node_id; JointKind kind; const char* shortName; int8_t default_dir; };

// node ids: R-hip=10, R-knee=11, L-hip=2, L-knee=3 (v7.1.5.ino:77-80)
static constexpr JointDef JOINTS[PG_NJOINTS] = {
  { 10, KIND_HIP,  "HR", -1 },
  { 11, KIND_KNEE, "KR", -1 },
  {  2, KIND_HIP,  "HL", +1 },
  {  3, KIND_KNEE, "KL", +1 },
};

// reverse-lookup joint index by CAN node id; -1 if not ours.
static inline int joint_for_node(uint8_t node_id) {
  for (int i = 0; i < PG_NJOINTS; i++)
    if (JOINTS[i].node_id == node_id) return i;
  return -1;
}

// ---- Safety envelopes (motor turns) ----------------------------------------
static constexpr float HIP_TURN_MIN = -6.0f, HIP_TURN_MAX = 8.0f;
static constexpr float KNEE_TURN_MIN = -2.0f, KNEE_TURN_MAX = 10.0f;
static constexpr float ABS_VEL_LIM = 25.0f, ABS_ACC_LIM = 200.0f, ABS_CUR_LIM = 20.0f;
static constexpr float MAX_HIP_TORQUE_NM = 3.0f, MAX_KNEE_TORQUE_NM = 2.5f;

// ---- FUTEK force -> joint torque (moment arm, applied on the MAIN side) -----
// Coproc sends calibrated N per cell; main multiplies by the moment arm to get
// joint torque. Order HR/KR/HL/KL. Tune to the mechanical mounting.
static constexpr float MOMENT_ARM_M[PG_NJOINTS] = { 0.15f, 0.12f, 0.15f, 0.12f };

// Coproc link timeout: no valid SensorPacket for this long -> sensors offline.
static constexpr uint32_t COPROC_LINK_TIMEOUT_MS = 300;

// ---- iq -> joint torque ----------------------------------------------------
static constexpr float HR_TORQUE_CONSTANT_NM_A = 0.119f;
static constexpr float GEAR_EFFICIENCY = 0.70f;
static constexpr float JOINT_NM_PER_A =
    HR_TORQUE_CONSTANT_NM_A * GEAR_RATIO * KEF * GEAR_EFFICIENCY;

// ---- ROM defaults (joint-frame deg) per kind (v7.1.5 anatomical) -----------
static constexpr float ROM_HIP_MIN_DEG  = -18.0f, ROM_HIP_MAX_DEG  = 25.0f;
static constexpr float ROM_KNEE_MIN_DEG =  -6.0f, ROM_KNEE_MAX_DEG = 31.0f;

// ---- Init pose (joint-frame deg) per joint idx HR/KR/HL/KL ------------------
static constexpr float INIT_POS_DEG[PG_NJOINTS] = { 10.0f, 8.5f, -6.0f, 9.5f };

// ---- Gait engine -----------------------------------------------------------
static constexpr float    DEFAULT_GAIT_CPS = 0.36f;   // cycles/s (firmware speed=2)
static constexpr float    DEFAULT_AMP_R = 0.50f, DEFAULT_AMP_L = 0.50f;
static constexpr uint32_t INIT_SETTLE_MS = 2000;      // hold init pose before walking
static constexpr float    POS_FILTER_BANDWIDTH_HZ = 7.5f;  // provisioned via odrivetool
                                                           // (CAN can't set at runtime)
// ---- Homing ----------------------------------------------------------------
static constexpr float    HOMING_TOL_TURNS = 0.05f;
static constexpr uint32_t HOMING_TIMEOUT_MS = 4000;
static constexpr uint32_t HOMING_RAMP_MS    = 1500;

// canonical order guarantees idx 0,1 = Right leg; 2,3 = Left leg
static inline bool joint_is_left(int idx) { return idx >= 2; }

// ---- AAN (position-mode phase-yielding overlay) ----------------------------
static constexpr float AAN_DETECTION_THRESHOLD_NM = 3.0f;   // ~95th-pct noise
static constexpr float AAN_YIELD_FULL_NM          = 6.0f;
static constexpr float AAN_FACTOR_ENGAGE_TC_S     = 0.10f;  // slow to yield
static constexpr float AAN_FACTOR_RELEASE_TC_S    = 0.03f;  // quick to resume
// assist-level knob [0..1] scales the yield threshold (0.5 == legacy 3/6)
static constexpr float AAN_THRESHOLD_AT_MAX_ASSIST_NM = 1.0f;  // assist=1 -> light push
static constexpr float AAN_THRESHOLD_AT_MIN_ASSIST_NM = 5.0f;  // assist=0 -> work hard
static constexpr float AAN_YIELD_SPAN_NM              = 3.0f;  // hi = lo + span

// ---- Torque-mode impedance (port of labs/torque_gait.py defaults) ----------
static constexpr float TQ_PATIENT_ASSIST_FRAC   = 0.5f;   // alpha: fraction of patient passive carried
static constexpr float TQ_K_HIP_FALLBACK_NM     = 2.0f;   // grav gain w/o a model
static constexpr float TQ_K_KNEE_FALLBACK_NM    = 0.5f;
static constexpr float TQ_KASSIST_HIP_NM_DEG    = 0.08f;  // soft tracking spring
static constexpr float TQ_KASSIST_KNEE_NM_DEG   = 0.05f;
static constexpr float TQ_ASSIST_SAT_DEG        = 12.0f;
static constexpr float TQ_BDAMP_HIP_NM_S_DEG    = 0.02f;
static constexpr float TQ_BDAMP_KNEE_NM_S_DEG   = 0.02f;
static constexpr float TQ_ROM_MARGIN_DEG        = 4.0f;   // virtual-wall band
static constexpr float TQ_KWALL_NM_DEG          = 0.4f;
static constexpr float TQ_BWALL_NM_S_DEG        = 0.06f;
static constexpr float TQ_GATE_MIN              = -1.0f;  // allow full reverse
static constexpr float TQ_GATE_VREF_FLOOR_DEG_S = 2.0f;
static constexpr float TQ_GATE_HOLD_BAND        = 0.05f;
static constexpr float TQ_REVERSE_ENABLE        = -0.15f;
static constexpr float TQ_TAU_RATE_NM_S         = 40.0f;  // joint torque slew
// joint torque [Nm] -> ODrive motor input_torque [Nm] (inverse of the gear path)
static constexpr float DRIVE_NM_PER_JOINT_NM = 1.0f / (GEAR_RATIO * KEF * GEAR_EFFICIENCY);

// ---- Safety ----------------------------------------------------------------
// NO physical ODrive endstops in this setup -> the firmware motor-turn envelope
// clamp (HIP/KNEE_TURN_MIN/MAX above) is the ONLY hard ROM limit. Every
// commanded position is clamped to it before TX, regardless of the soft deg ROM.
#ifndef ESTOP_GPIO
#define ESTOP_GPIO 6        // <-- VERIFY a free broken-out GPIO; NC button to GND
#endif
#define ESTOP_ACTIVE_LOW 1  // INPUT_PULLUP; NC button: released=LOW, pressed/open=HIGH
// DEV: with no E-STOP button wired, the INPUT_PULLUP pin floats HIGH and reads as
// "pressed" -> latched e-stop. Keep 0 until an NC button is wired to GND; then set
// 1 (an unwired/severed e-stop then fails safe to STOPPED). MUST be 1 for patient use.
#ifndef USE_ESTOP_BUTTON
#define USE_ESTOP_BUTTON 0
#endif
static constexpr float SENSOR_MAX_RATE_NM_PER_S   = 20.0f;  // glitch if exceeded
static constexpr int   SENSOR_GLITCH_ESTOP_PER_S  = 50;     // glitches/s -> e-stop
static constexpr float CROSSCHECK_DIVERGE_NM      = 8.0f;   // iq vs FUTEK warn
// Measured position beyond the envelope by this margin -> e-stop. The position
// clamp can't catch a torque-mode runaway (we command torque, not position), so
// this guards the no-endstop torque path.
static constexpr float ENV_OVERRUN_MARGIN_TURNS   = 0.5f;

// ---- Loop rates ------------------------------------------------------------
static constexpr uint32_t TELEM_HZ = 100;
static constexpr uint32_t GAIT_STEP_HZ = 50;
