# pgear_v8 — P.GEAR exoskeleton firmware (ESP32-S3 main controller)

Clean re-architecture (2026-06-20): the Raspberry Pi is dropped **as the
controller** (Linux isn't real-time — CAN jitter + ODrive USB-stream lag).
The **ESP32-S3** is the hard-real-time controller again; a **PC** runs the
PySide6 supervisor GUI (`../../../pgear_tools/pi_gui`) as a pure pendant.

This is a fresh line, **not** `Exoskeleton_control_v7.x` — that lineage
carries the on-device LVGL GUI + SD logging we deliberately drop. The
v7.1.5 tree stays as bibliography (its packet protocol is lifted here).

## Architecture (Option C)

```
   load cell x4 ──▶ ADS1256 ──▶ [coproc ESP32] ──UART(calibrated N)──▶ [ESP32-S3 main]
                                                                      │  ▲
                                                  CAN/TWAI (250 kbps) │  │ telemetry
                                                                      ▼  │
                                                              2x ODrive (4 axes)

   [ESP32-S3 main] ──USB-CDC (commands + LogPacket telemetry)──▶ [PC PySide6 GUI]
                   └─WiFi-UDP :47000 LogPacket (telemetry-only)─▶ [PC udp logger]
```

- **ESP32-S3** owns the reactive loop (≈250 Hz) **and** the AAN / trim /
  assist policies → the host is never in a feedback path.
- **No LVGL.** The attached LCD (if present) shows a minimal **text** status
  page only; all interaction is on the PC.
- **Coproc kept**: isolates fussy ADS1256 timing + independent safety LEDs.
  Outputs **calibrated N** (cal authority at the cells; reuses the existing
  `pgear_tools/coproc` calibration tools).

## Two control modes (both ported — shared core, two `step()` laws)

| Mode | ODrive | Patient interaction |
|---|---|---|
| **Position** (baseline) | POS_FILTER `set_input_pos` | AAN-by-iq **phase-yielding** (resistance slows the gait clock) |
| **Torque** (compliant) | torque `set_input_torque` | **moving-reference impedance** + **cooperation gate** (patient leads pace/direction) |

A `controlMode` field (PC `SET_MODE` command, mirrored in the `LogPacket`
`torqueMode` flag) selects which law runs per tick.

## The numpy split

Calibration **fit** (least-squares) stays in Python on the PC; the ESP32
only **evaluates** the resulting 5-coef polynomial (`predict(deg,vel)`).
The PC downloads coefficients via the `LOAD_COEFFS` command. No linear
algebra on the microcontroller.

## Layout

```
pgear_main/                  ESP32-S3 main controller (Arduino sketch)
  pgear_main.ino             setup/loop, dual-core task spawn (control→core1, comms→core0)
  protocol.h                 ALL packet structs + CRC-16/CCITT (the wire contract)
  can_odrive.h               TWAI + ODrive cmd protocol      (port of can_odrive.py)   [Phase 1]
  gait.h                     trajectory + phase math          (port of gait.py)         [Phase 2]
  gait_engine.h              IDLE/INIT/GAIT/HOMING state mc    (port of gait_engine.py)  [Phase 2]
  calib.h                    JointCalibration::predict (eval only)                       [Phase 6]
  control.h                  pos-mode AAN + torque-mode impedance (both step() laws)     [Phase 6/6b]
  safety.h                   estop, watchdog, ROM enforce, cross-check, drift            [Phase 5]
  coproc_link.h              UART RX/TX to ADS1256 coproc                                [Phase 3]
  host_link.h                USB-CDC command parse + LogPacket; WiFi-UDP log             [Phase 3/4]
  status_screen.h            minimal text status panel (panel init, NO LVGL)            [Phase 0/8]
pgear_coproc_ads1256/
  pgear_coproc_ads1256.ino   ADS1256 → calibrated N → framed UART                        [Phase 3]
docs/PLAN.md                 full plan: phases, packet contracts, port list
```

## Build

Arduino-ESP32 (same framework as v7.1.5). Board: **Waveshare
ESP32-S3-Touch-LCD-7**. CAN shares GPIO19/20 with native USB via a CH422G
switch, so we run CAN there (`board_io.cpp` sets EXIO5=HIGH) and use the
board's **second USB-C "USB TO UART" (CH343P) port** for the PC link and
flashing — set Arduino **USB CDC On Boot: DISABLED** so `Serial` maps to that
port. Needs the Waveshare **ESP_IOExpander** library. Each subfolder is its
own sketch. (⚠ verify the 7" full GPIO map — v7.1.5's config was the 4.3".)

## Status

**Phase 0 scaffold.** `pgear_main.ino` brings up the board headless, spawns
the dual-core skeleton, prints a text status line, and echoes USB-CDC.
`protocol.h` is the complete wire contract. Everything else is a stub
header to be filled per `docs/PLAN.md`.
