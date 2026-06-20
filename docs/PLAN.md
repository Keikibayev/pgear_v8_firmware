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
| **2** 🔨 | `gait` + `gait_engine` ports + control-task integration + temp serial bench keys (a/r/s/d/e) | smooth motion, no jitter — bench-verify pending |
| **3** 🔨 | ADS1256 coproc firmware + `coproc_link` (UART2/43/44) + force→torque fusion; wiring doc | torque sensing — bench-verify pending (`LogPacket` emit moves to P4/WiFi) |
| **4** | PC supervisor: strip control loop from pi_gui; `CommandPacket`; profiles → coeffs | host = pure pendant |
| **5** | safety supervisor (estop, watchdog, ROM enforce, sensor rate-cap/glitch, drift) | safe to strap a patient |
| **6** | position-mode AAN + trim + passive-model eval | assist-as-needed on-chip |
| **6b** | torque-mode impedance law (`TorqueGaitController`) | compliant mode |
| **7** | Jog / Teach-ROM / Observe modes | setup workflows |
| **8** | status-screen polish; WiFi-UDP logger; kiosk/boot | field-ready |

## CAN / USB routing — RESOLVED (verified vs Waveshare docs, Phase 1)
On the ESP32-S3-Touch-LCD-7, CAN (TX=GPIO20, RX=GPIO19) **shares the native-USB
D-/D+ pins** through an **FSUSB42UMX** switch selected by **CH422G EXIO5**
(USB_SEL): EXIO5=HIGH → CAN, LOW → native USB. They cannot run at once. The
board has a **2nd USB-C port** = "USB TO UART" via a **CH343P** bridge (UART0,
GPIO43/44), independent of GPIO19/20. **Decision:** keep CAN on 19/20
(EXIO5=HIGH, set in `board_io.cpp`); the PC link + flashing use the CH343P
port. Arduino: set **USB CDC On Boot: DISABLED** so `Serial` = that port.

## Host link — DECIDED (2026-06-20)
PC link over **WiFi, station mode** — the ESP32-S3 (and the PC) join an
**external hotspot** the user provides. Coproc takes the UART2 pins (43/44),
so there is no wired-USB PC link in normal operation (USB only for flashing).
Phase 4 host link: **UDP** for telemetry/logging (loss-tolerant `LogPacket`)
+ a **reliable channel (TCP or UDP+ACK)** for commands + a **link-loss
watchdog** (PC heartbeat gone ~1 s → ESP32 auto-ramps to safe hold). Safety
note: real E-STOP + limits are HARDWARE (button + ODrive GPIO endstops), never
dependent on WiFi.

## Bench-verify Phase 2
With CAN verified (Phase 1) and the empty exo on the bench: open the serial
console (CH343P port), press `a` (arm → closed-loop POS_FILTER), then `r`
(run → 2 s init-settle ramp, then walking). `s` ramps home and idles; `e` is a
software e-stop (hardware e-stop comes in Phase 5). Watch the status line:
`ph` should go IDLE→INIT(1)→GAIT(2), `p01` cycling 0..1. Confirm the legs walk
smoothly with no jerk at the GAIT→HOMING transition.

## Open hardware items
- Verify **ESP32-S3-Touch-LCD-7** GPIO map (v7.1.5 config was the 4.3" variant).
- Confirm per-axis `Kt` (torque mode depends on it); KR/HL/KL screw tightness.
- Coproc UART pins on the 7" board.

## Bench-verify Phase 1
Flash `pgear_smoke_can` to the ESP32-S3, wire CAN_H/CAN_L to the ODrive bus
(ODrives powered, configured for 250 kbps + cyclic heartbeat/encoder/iq
broadcast). Expect the 1 Hz report to show rising frame counts and live
`pos`/`iq` for all four nodes HR(10)/KR(11)/HL(2)/KL(3), `bus_err` flat at 0.
