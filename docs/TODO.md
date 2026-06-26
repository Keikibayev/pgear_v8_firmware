# P.GEAR v8 — TODO for the operator (hardware / config / bench)

Things **you** need to do that the firmware can't. Grouped by when they bite.
Code-side phase status lives in `PLAN.md`.

## Before first power-on
- [ ] Install the Arduino **ESP_IOExpander** library (CH422G) — used by
      `board_io.cpp` and the smoke test.
- [ ] Arduino IDE: select the ESP32-S3 board, set **USB CDC On Boot: Disabled**
      so `Serial` maps to the CH343P "USB TO UART" port.
- [ ] Edit **`pgear_main/wifi_config.h`** — set `WIFI_SSID` / `WIFI_PASSWORD`
      to your hotspot.
- [ ] Confirm a **free GPIO for E-STOP** (`ESTOP_GPIO` in `constants.h`, default
      6) is actually broken out and unused on the 7" board; change if not.
- [ ] Wire everything per **`docs/WIRING.md`**; tie all grounds common.

## Board / flashing nuances
- [ ] Set the board's **UART1/UART2 switch**: UART1/CH343P for flashing +
      console (coproc unplugged); **UART2 for coproc** at run time.
- [ ] Verify the full **ESP32-S3-Touch-LCD-7 GPIO map** against its schematic
      (our pin notes came from the 4.3" config + Waveshare docs).

## ODrive provisioning (odrivetool) — once per ODrive
- [ ] Set **CAN node IDs**: R-hip=10, R-knee=11, L-hip=2, L-knee=3
      (hip=axis1, knee=axis0 on each board).
- [ ] Set **CAN bitrate 250 kbps**; enable cyclic heartbeat / encoder / iq
      broadcast so the main board's telemetry is populated.
- [ ] Provision **`input_filter_bandwidth ≈ 7.5 Hz`** (POS_FILTER) — the CAN
      path cannot set it at runtime.
- [ ] Set sane **vel / current limits** on each axis.
- [ ] **120 Ω CAN termination** at both bus ends (board jumper + far ODrive).

## Calibration / mechanical
- [ ] **Load-cell scale (counts-per-Nm).** The coproc `g_cal`/`DEFAULT_CAL` is a
      placeholder. Procedure:
      1. Flash `pgear_v8/pgear_coproc_calib/` to the coproc (streams tared cell
         counts over USB serial).
      2. Run `python -m pgear_pi.labs.loadcell_calib_gui` (from `pi_gui/`).
      3. Isolate the cell (motor off / limb free), **Tare** at no load.
      4. Apply a KNOWN torque `τ = F·r` with a dynamometer at distance `r` from
         the joint axis (pull perpendicular); record CELL counts. Repeat at 2–3
         levels, both directions.
      5. Slope of counts-vs-τ = **`k` (counts per Nm)** per cell.
      6. Write each `k` into `pgear_coproc_ads1256.ino` `g_cal[]` and set
         **`MOMENT_ARM_M[]=1.0`** in `constants.h` (the scale now yields Nm
         directly — no separate moment arm needed). Reflash.
      - ⚠️ NVS caveat: stored `cal0..3` override the hardcoded `g_cal` — clear
        NVS for a fresh hardcoded value. (Live PC-driven scale-adjust is not
        wired yet; only `OP_TARE` is.)
      - Note: load cells are READ-ONLY (cross-check/telemetry); control uses iq,
        so this isn't required for torque mode / AAN.
- [ ] Confirm **per-axis `Kt`** (torque_constant) in odrivetool; if axes differ,
      promote `JOINT_NM_PER_A` to per-joint. (Torque mode / AAN depend on it.)
- [ ] Tighten **KR / HL / KL joint screws** (were loose) before trusting their
      calibration fits.
- [ ] **No endstops:** verify the motor-turn envelope matches the mechanical hard
      stops on EACH leg — it is now the ONLY hard ROM limit. It is direction-aware
      (`joint_turn_limits`): left leg HIP −6..+8 / KNEE −2..+10, right leg mirrored
      HIP −8..+6 / KNEE −10..+2. Drive each knee to full flexion and confirm both
      reach the same joint-angle limit (the old global [−2,+10] capped KR at ~7.5°).

## Safety wiring (do not skip)
- [ ] E-STOP button → ESP32 GPIO **and** a hardware motor-power cutoff
      (contactor / ODrive power) that works without the MCU.
- [ ] Verify the firmware envelope clamp + soft ROM on the bench (drive toward a
      limit, confirm it plateaus) BEFORE any patient session.

## Bench-verify sequence (per PLAN.md)
- [ ] **Phase 1:** flash `pgear_smoke_can` → frame counts + pos/iq from all 4
      nodes @250k, `bus_err`=0.
- [ ] **Phase 2:** flash `pgear_main`, serial `a`(arm)→`r`(run) → empty exo
      walks (ph IDLE→INIT→GAIT, p01 cycling), `s` homes smoothly, no jerk.
- [ ] **Phase 3:** set `USE_COPROC 1`, switch to UART2; confirm per-joint torque
      in telemetry and that `OP_TARE` zeroes the cells. (Scale is set by the
      load-cell calibration above; cells are read-only — control uses iq.)
- [ ] **Phase 4:** join hotspot; confirm `pgear_udp_logger.py` sees LogPacket on
      :47000; GUI commands over TCP :47001; pull the link while running →
      controller auto-homes (watchdog).

## Optional: on-device status screen (Phase 8, default OFF)
- [ ] To enable: set `USE_STATUS_SCREEN 1`, and copy into the sketch from
      v7.1.5: `esp_panel_board_custom_conf.h`, `lvgl_v8_port.{h,cpp}`; install
      the **ESP32_Display_Panel** + **lvgl** libraries. Device runs fine headless
      without it.

## Known code follow-ups (for the developer)
- [x] Verified `calib.cpp` sin/cos basis matches `calibrator.py` (both radians).
- [ ] pi_gui supervisor adaptation: strip the control loop; send CommandPackets
      over TCP :47001; decode LogPacket over UDP :47000 + CaptureRangePacket
      (0xBB77, 38 B) for Teach/Observe ROM; push profile coeffs via LOAD_COEFFS.
