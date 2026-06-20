# P.GEAR v8 — build plan

Pivot 2026-06-20: Pi dropped **as controller** (not real-time). ESP32-S3 is the
hard-real-time controller; PC runs the PySide6 supervisor (Option C). See the
parent memory `project_esp32_return.md` and `../README.md`.

## Repo location
`f:\ArduinoProjects\Arduino\pgear_v8\` — new git repo, sibling to the (now
bibliography) `Exoskeleton_control_v7.x` trees. `pgear_tools` stays the
host-side Python home (GUI + UDP logger + coproc calibration tools).

## Control modes (BOTH ported — shared core, two `step()` laws)
1. **Position mode** (baseline, `worker.py`+`gait_engine.py`): POS_FILTER
   `set_input_pos` + AAN-by-iq phase-yielding. ROM-clamped pos is bounded by
   construction → the safe path to prove the hardware. Port FIRST.
2. **Torque mode** (`labs/torque_gait.py`): moving-reference impedance +
   cooperation gate (patient leads pace/direction). Port SECOND (6b); needs
   per-axis Kt + bench tuning of caps/gains.

`controlMode` field (PC `OP_SET_MODE`, mirrored in `LP_FLAG_TORQUE_MODE`)
selects per tick; the ODrive control-mode switch happens on mode change.

## The numpy split
PC fits (`calibrator.fit_joint_model[_robust]`, least-squares) → downloads
coefficients via `OP_LOAD_COEFFS` (`JointCoeffs`). ESP32 only evaluates
`predict(deg,vel)`. No linear algebra on-chip.

## Packet contracts — see `pgear_main/protocol.h`
- ① Coproc UART: `SensorPacketV2`/`CalStatePacket` up, `DownstreamPacket`/
  `CalAdjustPacket` down. CRC-16/CCITT, tare-seq, live cal-adjust.
- ② Telemetry: 206-byte `LogPacket` (v3, v7.1.5-compatible) — same bytes over
  USB-CDC and WiFi-UDP `:47000`; old `pgear_udp_logger.py` decodes unchanged.
- ③ Commands: `CommandPacket` (0xCC33) opcode+payload; `OP_LOAD_COEFFS` is the
  numpy-split boundary.

## Phase order (motion-deterministic first)
| Phase | Deliverable | Proves |
|---|---|---|
| **0** ✅ | repo skeleton; headless bring-up; dual-core; USB-CDC echo; `protocol.h` | toolchain + board |
| **1** 🔨 | `can_odrive.cpp` TWAI port (RX task core 0, TX, watchdog, snapshot) + standalone `pgear_smoke_can` test | **deterministic CAN** — bench-verify pending |
| **2** | `gait` + `gait_engine` → empty exo through gait in pos mode | smooth motion, no jitter |
| **3** | ADS1256 coproc + UART link + fusion; `LogPacket` out (USB+UDP) | torque sensing + logger |
| **4** | PC supervisor: strip control loop from pi_gui; `CommandPacket`; profiles → coeffs | host = pure pendant |
| **5** | safety supervisor (estop, watchdog, ROM enforce, sensor rate-cap/glitch, drift) | safe to strap a patient |
| **6** | position-mode AAN + trim + passive-model eval | assist-as-needed on-chip |
| **6b** | torque-mode impedance law (`TorqueGaitController`) | compliant mode |
| **7** | Jog / Teach-ROM / Observe modes | setup workflows |
| **8** | status-screen polish; WiFi-UDP logger; kiosk/boot | field-ready |

## Open hardware items
- ⚠ **CAN vs native-USB pin conflict (found Phase 1):** v7.1.5 wired the CAN
  transceiver to **GPIO 19/20**, which are the ESP32-S3 **native-USB D-/D+**
  pins. If the 7" board keeps that wiring, the PC link CANNOT be native USB-CDC
  — route it over the board's **UART-bridge USB-C** (CH343) instead, or move
  CAN to free pins. Pins are `#define CAN_RX_PIN/CAN_TX_PIN` in `constants.h`.
  Resolve this before/with bench-verifying Phase 1.
- Verify **ESP32-S3-Touch-LCD-7** GPIO map (v7.1.5 config was the 4.3" variant).
- Confirm per-axis `Kt` (torque mode depends on it); KR/HL/KL screw tightness.
- Coproc UART pins on the 7" board.

## Bench-verify Phase 1
Flash `pgear_smoke_can` to the ESP32-S3, wire CAN_H/CAN_L to the ODrive bus
(ODrives powered, configured for 250 kbps + cyclic heartbeat/encoder/iq
broadcast). Expect the 1 Hz report to show rising frame counts and live
`pos`/`iq` for all four nodes HR(10)/KR(11)/HL(2)/KL(3), `bus_err` flat at 0.
