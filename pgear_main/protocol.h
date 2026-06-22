// ============================================================================
// protocol.h  —  P.GEAR v8 wire contract (the single source of truth)
// ----------------------------------------------------------------------------
// Three links, all little-endian, all __attribute__((packed)), all framed with
// CRC-16/CCITT over every byte except the trailing 2-byte CRC. Packet families
// are disambiguated by the (start0, start1) pair.
//
//   ① Coproc  <-> ESP32-S3   over UART  (ADS1256 calibrated-N front-end)
//   ② ESP32-S3 -> PC         telemetry  (USB-CDC AND WiFi-UDP, same bytes)
//   ③ PC       -> ESP32-S3   commands   (USB-CDC)
//
// The coproc sketch keeps its OWN matching copies of the ① structs (the old
// firmware did the same) and guards them with static_assert on sizeof — keep
// the two in sync by size + field order.
// ============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---- Link parameters --------------------------------------------------------
#define PG_COPROC_UART_BAUD   460800
#define PG_UDP_LOG_PORT       47000          // broadcast 255.255.255.255:47000
#define PG_LOGPACKET_VERSION  3              // wire-format version (v7.1.5-compatible)

// ---- Joint order (canonical everywhere) ------------------------------------
//   0 = R-hip, 1 = R-knee, 2 = L-hip, 3 = L-knee
#define PG_NJOINTS  4

// ============================================================================
// CRC-16/CCITT  (poly 0x1021, init 0xFFFF) — identical to v7.1.5 / coproc
// ============================================================================
static inline uint16_t pg_crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ============================================================================
// ① COPROC LINK
// ============================================================================

// ---- Upstream: coproc -> ESP32 ---------------------------------------------

// 0xAA 0x55 — per-cell calibrated force, ~200 Hz. (Redesign of the old 24-byte
// SensorPacket: ADS1256, N instead of pre-multiplied torque; ESP32 does the
// moment-arm -> torque step so all joint geometry lives in one place.)
struct __attribute__((packed)) SensorPacketV2 {
  uint8_t  start0;        // 0xAA
  uint8_t  start1;        // 0x55
  uint8_t  seq;
  uint8_t  flags;         // bit7=heartbeat, bit0..3 = channel saturated/invalid
  float    forceN[PG_NJOINTS];   // calibrated N, joint order above
  uint16_t rateHz;        // actual ADS1256 sample rate
  uint16_t crc;
};
static_assert(sizeof(SensorPacketV2) == 24, "SensorPacketV2 size");

// 0xAA 0x77 — live cal scale per channel, so the PC can show the active CAL.
// Sent on boot, after each CalAdjust, and every 5 s.
struct __attribute__((packed)) CalStatePacket {
  uint8_t  start0;        // 0xAA
  uint8_t  start1;        // 0x77
  uint8_t  reserved;
  uint8_t  reasonCode;    // 1=boot, 2=after-adjust, 3=periodic
  float    cal[PG_NJOINTS];
  uint16_t crc;
};
static_assert(sizeof(CalStatePacket) == 22, "CalStatePacket size");

#define SENSOR_FLAG_CH0_BAD   (1 << 0)
#define SENSOR_FLAG_CH1_BAD   (1 << 1)
#define SENSOR_FLAG_CH2_BAD   (1 << 2)
#define SENSOR_FLAG_CH3_BAD   (1 << 3)
#define SENSOR_FLAG_HEARTBEAT (1 << 7)

// ---- Downstream: ESP32 -> coproc -------------------------------------------

// 0xBB 0x66 — per-axis error flags + tare command, ~10 Hz.
struct __attribute__((packed)) DownstreamPacket {
  uint8_t  start0;        // 0xBB
  uint8_t  start1;        // 0x66
  uint8_t  flags;         // bit0=R-hip err,1=R-knee,2=L-hip,3=L-knee, bit7=estop
  uint8_t  reserved;
  uint16_t tareSeq;       // coproc re-tares all cells when this changes
  uint16_t crc;
};
static_assert(sizeof(DownstreamPacket) == 8, "DownstreamPacket size");

// 0xBB 0x55 — multiplicative cal adjust: newCal = currentCal * ratio.
struct __attribute__((packed)) CalAdjustPacket {
  uint8_t  start0;        // 0xBB
  uint8_t  start1;        // 0x55
  uint8_t  joint;         // 0..3
  uint8_t  reserved;
  float    ratio;         // |ratio| in [0.01, 100]; sign flips convention
  uint16_t seq;           // de-dup re-sends
  uint16_t crc;
};
static_assert(sizeof(CalAdjustPacket) == 12, "CalAdjustPacket size");

#define DS_FLAG_HIPR_ERR  (1 << 0)
#define DS_FLAG_KNEER_ERR (1 << 1)
#define DS_FLAG_HIPL_ERR  (1 << 2)
#define DS_FLAG_KNEEL_ERR (1 << 3)
#define DS_FLAG_ESTOP     (1 << 7)

// ============================================================================
// ② TELEMETRY  (ESP32 -> PC, identical bytes over USB-CDC and WiFi-UDP)
//    206-byte LogPacket, v7.1.5-compatible so the existing pgear_udp_logger.py
//    decodes it unchanged. start 0xBB 0x66 (distinct LINK from DownstreamPacket).
// ============================================================================
struct __attribute__((packed)) LogPacket {
  // HEADER (8)
  uint8_t  start0;        // 0xBB
  uint8_t  start1;        // 0x66
  uint8_t  version;       // PG_LOGPACKET_VERSION (3)
  uint8_t  reserved0;
  uint16_t seq;
  uint16_t headerCrc;     // CRC-16 of bytes [0..5]
  // TIMING (4)
  uint32_t timeMs;
  // GAIT / TOP-LEVEL STATE (8)
  uint8_t  gaitPhase;     // PH_IDLE..PH_GAIT..PH_RAMP_DOWN
  uint8_t  stepIdx;       // 0..49
  uint8_t  profileSlot;   // 0xFF = none
  uint8_t  sensorHealth;  // bit0=R-hip..bit3=L-knee
  uint16_t flags;         // see LP_FLAG_* below
  uint16_t linkAgeMs;     // ms since last good coproc packet
  // JOINT ARRAYS (136), joint order above
  float    refPos[PG_NJOINTS];      // turns
  float    pos[PG_NJOINTS];         // turns
  float    vel[PG_NJOINTS];         // turns/s
  float    cmdTorque[PG_NJOINTS];   // Nm (torque mode; 0 in pos mode)
  float    measTorque[PG_NJOINTS];  // Nm (from FUTEK via coproc)
  float    gravTerm[PG_NJOINTS];    // Nm (gravity-comp contribution)
  float    ffTerm[PG_NJOINTS];      // Nm (feedforward / assist-spring contribution)
  float    iqMeasured[PG_NJOINTS];  // A  (q-axis current)
  float    motorEffort[PG_NJOINTS]; // Nm (iq-derived torque estimate)
  uint16_t hbAgeMs[PG_NJOINTS];     // ms (ODrive heartbeat age; 0xFFFF=never)
  // THERAPIST TUNABLES (24)
  float    assistR, assistL;
  float    deadzoneR, deadzoneL;
  float    ampR, ampL;
  // DIAGNOSTICS (8)
  uint16_t ctrlLoopUs;    // last control-loop exec time
  uint16_t linkCrcFails;  // coproc-UART CRC failures since boot
  uint16_t linkResyncs;   // coproc-UART resyncs since boot
  uint8_t  crossCheckFault; // bit i = joint i latched cross-check fault
  uint8_t  hbErrorByte;     // bit i = joint i heartbeat error
  // TRAILER (2)
  uint16_t crc;           // CRC-16/CCITT over bytes [0..203]
};
static_assert(sizeof(LogPacket) == 206, "LogPacket size (must stay 206 for udp logger)");

// LogPacket.flags bit map (v7.1.5-compatible)
#define LP_FLAG_RUN            (1 << 0)
#define LP_FLAG_ESTOP          (1 << 1)
#define LP_FLAG_SENSOR_ONLINE  (1 << 2)
#define LP_FLAG_FF_ENABLED     (1 << 3)
#define LP_FLAG_AAN_ENABLED    (1 << 4)   // (was fuzzyEnabled) — AAN active
#define LP_FLAG_FFP_TRIPPED    (1 << 5)
#define LP_FLAG_SEG_GRAV_MODEL (1 << 6)
#define LP_FLAG_CROSSCHECK_FLT (1 << 7)
#define LP_FLAG_GAIT_AUTOPROG  (1 << 8)
#define LP_FLAG_TORQUE_MODE    (1 << 9)   // 0 = position mode, 1 = torque mode

// Captured ROM from Teach/Observe (ESP32 -> PC, UDP, same port). 0xBB 0x77.
// Smaller than LogPacket so the old pgear_udp_logger.py (len>=206 check) ignores it.
struct __attribute__((packed)) CaptureRangePacket {
  uint8_t  start0;        // 0xBB
  uint8_t  start1;        // 0x77
  uint8_t  mode;          // SetupMode (1=jog,2=teach,3=observe)
  uint8_t  validMask;     // bit i = joint i has a captured range
  float    minDeg[PG_NJOINTS];
  float    maxDeg[PG_NJOINTS];
  uint16_t crc;
};
static_assert(sizeof(CaptureRangePacket) == 38, "CaptureRangePacket size");

// ============================================================================
// ③ COMMANDS  (PC -> ESP32, USB-CDC)
//    0xCC 0x33. CRC-16/CCITT covers bytes [0 .. 4+len-1] (header + payload).
//    Payload layout is per-opcode (see notes). Parser caps len at PG_CMD_MAXPAYLOAD.
// ============================================================================
#define PG_CMD_START0       0xCC
#define PG_CMD_START1       0x33
#define PG_CMD_MAXPAYLOAD   48

enum PgOpcode : uint8_t {
  OP_NOP = 0,
  // lifecycle / run state
  OP_ARM, OP_DISARM, OP_RUN, OP_STOP, OP_ESTOP, OP_ESTOP_RESET, OP_IDLE, OP_HOME,
  // mode + tunables (payload = float, unless noted)
  OP_SET_MODE,        // payload: uint8 PgControlMode
  OP_SET_CPS,         // float cycles/s
  OP_SET_AMP_R,       // float
  OP_SET_AMP_L,       // float
  OP_SET_BW,          // float Hz (POS_FILTER bandwidth)
  OP_SET_ASSIST,      // float [0..1]
  OP_SET_AAN,         // uint8 0/1
  OP_SET_ROM,         // joint=idx, payload: float romMinDeg, float romMaxDeg
  OP_SET_ENABLE,      // joint=idx, payload: uint8 0/1
  OP_SET_DIR,         // joint=idx, payload: int8 (-1/+1)
  // setup modes
  OP_JOG_ENABLE,      // uint8 0/1
  OP_JOG_TARGET,      // joint=idx, payload: float deg
  OP_TEACH_START, OP_TEACH_STOP,
  OP_OBSERVE_START, OP_OBSERVE_STOP,
  OP_CAL_START, OP_CAL_CANCEL,
  OP_TRIM_START, OP_TRIM_CANCEL, OP_TRIM_CLEAR,
  OP_SWEEP_START, OP_SWEEP_CANCEL,
  OP_TARE,            // re-tare FUTEK cells (bumps coproc tareSeq)
  // calibration coefficient download (the numpy-split boundary)
  OP_LOAD_COEFFS,     // payload: JointCoeffs (see below)
  OP_FULL_CAL,        // ODrive FULL_CALIBRATION_SEQUENCE on enabled axes (IDLE only)
  // torque-mode tunables (Phase 6b)
  OP_SET_TORQUE_ASSIST, // float: multiplier on the assist spring K_assist (the
                        // "go"); 1.0 = defaults, clamped [0,5]
  OP_SET_FREE_RUN,    // uint8 0/1: 1 = torque-mode phase self-advances (BENCH
                      // self-walk, gate ignored); 0 = patient-led (default)
};

enum PgControlMode : uint8_t { MODE_POSITION = 0, MODE_TORQUE = 1 };

// Payload for OP_LOAD_COEFFS — one fitted model for one joint.
// Mirrors calibrator.JointCalibration: iq = a*sin(deg)+b*cos(deg)+c*vel+d*sign(vel)+e
enum PgCoeffKind : uint8_t { COEFF_EMPTY = 0, COEFF_PATIENT_PASSIVE = 1 };
struct __attribute__((packed)) JointCoeffs {
  uint8_t joint;        // 0..3
  uint8_t kind;         // PgCoeffKind
  uint8_t reserved[2];
  float   coef[5];      // a, b, c, d, e
  float   residStdA;    // fit residual std [A] (drift-watcher threshold)
  float   calCps;       // cadence the fit was taken at
  float   calAmp;       // amplitude the fit was taken at
};
static_assert(sizeof(JointCoeffs) == 36, "JointCoeffs size");

struct __attribute__((packed)) CommandPacket {
  uint8_t  start0;      // 0xCC
  uint8_t  start1;      // 0x33
  uint8_t  opcode;      // PgOpcode
  uint8_t  joint;       // 0..3, or 0xFF = all / not-applicable
  uint8_t  len;         // payload byte count (<= PG_CMD_MAXPAYLOAD)
  uint8_t  payload[PG_CMD_MAXPAYLOAD];
  // NOTE: crc is NOT at a fixed offset — it follows payload[len].
  // TX: write crc16 over bytes [0 .. 4+len-1] at &payload[len].
  // RX: read 5-byte header, then len payload bytes, then 2 crc bytes.
};
