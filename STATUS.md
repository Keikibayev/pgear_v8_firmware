# P.GEAR v8 — STATUS / read-me-first

One-screen orientation for anyone (or any new chat) opening this repo. Deep
detail: `docs/PLAN.md` (phases), `docs/API.md` (interface), `docs/WIRING.md`,
`docs/TODO.md`.

## What this is
ESP32-S3 real-time exoskeleton controller. **Pivot 2026-06-20:** the Raspberry
Pi was dropped *as the controller* (not real-time); the ESP32-S3 runs the
hard-real-time loop and a PC runs the GUI as a supervisor over WiFi (Option C).
This is a fresh line — **not** the `Exoskeleton_control_v7.x` trees (those are
bibliography; their packet protocol + CAN bring-up were lifted here).

## Current state (2026-06-21)
**Firmware Phases 0–8 are CODE-COMPLETE but NOT bench-verified** — nothing has
run on hardware yet. Treat all field values/timing as the contract, expect
first-light tuning. The wire protocol + the PC link were verified headless.

| ✅ done | 0 scaffold · 7 modes (Jog/Teach/Observe) |
|---|---|
| 🔨 code-complete, bench-verify pending | 1 CAN · 2 gait · 3 coproc · 4 host-link · 5 safety · 6/6b control law · 8 status screen |

## What's left
1. **Bench-verify** the whole stack (see `docs/TODO.md` ladder: CAN smoke test →
   walk → coproc → WiFi → AAN/torque).
2. **pi_gui GUI rewiring** — wire the existing PySide6 GUI onto the device
   (via the bridge or `esp32_link` directly); today it still uses the old path.
3. **Operator setup** — ODrive provisioning, FUTEK cal, `MOMENT_ARM_M`,
   per-axis `Kt`, envelope-vs-mechanical-stops, e-stop HW cutoff (`docs/TODO.md`).
4. Optional: enable status screen (Phase 8), bridge cosmetic strings, mock device.

## Cross-repo map
- **This repo** `pgear_v8_firmware` (`github.com/Keikibayev/pgear_v8_firmware`,
  branch `main`): `pgear_main/` (controller), `pgear_coproc_ads1256/` (coproc),
  `pgear_smoke_can/` (CAN test). Wire contract = `pgear_main/protocol.h`.
- **Host side** `pgear_tools` (`github.com/Keikibayev/pgear_tools`, branch
  `feature/patient-profiles-cp-support`):
  - `pi_gui/pgear_pi/transport/esp32_link.py` — Python device client (this protocol).
  - `pi_gui/pgear_pi/bridge.py` + `pi_gui/BRIDGE.md` — WebSocket bridge for
    multiple control heads (GUI + browser/ROS), hosts calibration/patient-torque.
  - `pi_gui/pgear_pi/labs/` — torque-mode controller + bench lab (`labs/README.md`).
  - `pi_gui/pgear_pi/` — the PySide6 GUI + `calibrator.py` (the fit, reused by the bridge).

## Key decisions (don't relitigate without reason)
- **Control split (Option C):** ESP32 owns inner loop + AAN/trim/assist; PC is a
  pure supervisor (never in a control loop).
- **Two control modes:** position (POS_FILTER + AAN phase-yield) and torque
  (moving-reference impedance + cooperation gate); `controlMode` selects.
- **numpy split:** PC fits the 5-coef models; ESP32 only evaluates `predict()`.
  Coefficients pushed via `OP_LOAD_COEFFS`.
- **WiFi station mode** to an external hotspot `PGEAR_TUNING`/`pgearpgear`;
  telemetry UDP :47000 (broadcast), commands TCP :47001 (single client → bridge).
- **CAN on GPIO19/20** (shares native-USB pins via CH422G EXIO5=HIGH); PC link +
  flashing use the **CH343P "USB TO UART"** port (USB CDC On Boot: Disabled).
- **Coproc** = ESP32 + ADS1256 (LC Tech v1.1) on UART2 (43/44); ADS lines on one
  connector (SCLK18/DIN23/DOUT19/CS5/DRDY21); PDWN→3V3 on module, RESET via SPI.
- **NO ODrive endstops** → the firmware motor-turn envelope clamp is the ONLY
  hard ROM limit; e-stop is a hardware button + motor-power cutoff.

## ⚠ Note on persistent memory
The assistant's project memory is scoped to the **`f:\pgear_tools`** folder, not
this one. A chat opened *here* won't auto-load that history — this repo's docs
(STATUS/PLAN/API/WIRING/TODO) are the durable record. For the full rationale,
run the assistant from `f:\pgear_tools`.
