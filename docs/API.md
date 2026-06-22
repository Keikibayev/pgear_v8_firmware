# P.GEAR v8 — External Control & Telemetry API (ICD)

For integrating a **second control head** (e.g. a ROS / WebSocket / browser app
with a PostgreSQL store) alongside the standalone Python GUI. Source of truth
for the wire format is [`pgear_main/protocol.h`](../pgear_main/protocol.h); a
reference Python client is
[`esp32_link.py`](https://github.com/Keikibayev/pgear_tools/blob/feature/patient-profiles-cp-support/pi_gui/pgear_pi/transport/esp32_link.py).

---

## 0. The one decision you must make first: arbitration

The ESP32-S3 is the real-time controller (WiFi **station** on `PGEAR_TUNING`).
It exposes two channels with **very different multiplicity**:

| Channel | Transport | Multiplicity | Use |
|---|---|---|---|
| **Telemetry (TM)** | **UDP broadcast** `:47000` | **many readers** | read device state; record sessions |
| **Commands (TC)** | **TCP** `:47001` | **one client** | control the device |

So **reading is free** — the ROS app can listen to UDP `:47000` directly and
store everything in Postgres with zero coordination. **Writing needs a single
owner**, both because the TCP server accepts one client and because two
controllers fighting over an exoskeleton is unsafe.

**Recommended topology — a single "device bridge" owns the command socket:**

```
                 UDP :47000 (broadcast TM)
   ESP32-S3 ───────────────┬───────────────┬─────────────► Python GUI (reads)
       ▲                   │               └─────────────► ROS app (reads → Postgres)
       │ TCP :47001 (TC)   │
       └───────────────────┴── DEVICE BRIDGE (sole TC owner; reuses esp32_link.py)
                                    ▲              ▲
                          WebSocket/ROS        WebSocket/ROS
                                    │              │
                              Python GUI       Browser/ROS head
```

The bridge: subscribes TM, re-publishes to all heads (ROS topic / WS broadcast),
accepts commands from heads, applies an **ownership policy**, and is the only
thing that opens TCP `:47001`. Heads never touch the ESP32 directly for control.

**Ownership policy (suggested):** exactly one head holds a "control token" at a
time; others are read-only and can *request* handover. **Exception: E-STOP must
be accepted from ANY head at ANY time** (and is *also* a hardware button —
software e-stop is convenience, not the safety mechanism).

> If you skip the bridge and let a head open `:47001` directly, only **one**
> head can control at a time (whoever holds the socket), and you own the
> arbitration yourself. The bridge is the clean path.

---

## 1. Transport details (binary level)

### Commands — TCP `:47001`, framed `CommandPacket`
```
byte:  0     1     2       3      4     5 .. 5+len-1   then 2
       0xCC  0x33  opcode  joint  len   payload[len]   crc16(LE)
```
- `joint` = 0..3 or `0xFF` (all / not-applicable). `len` ≤ 48.
- **CRC-16/CCITT** (poly `0x1021`, init `0xFFFF`) over bytes `[0 .. 4+len-1]`,
  appended little-endian. TCP is a byte stream → the receiver reassembles frames.

### Telemetry — UDP `:47000` (broadcast)
Identify by the first two bytes + length:
- `0xBB 0x66`, **206 B** → `LogPacket` (main telemetry, ~50 Hz)
- `0xBB 0x77`, **38 B**  → `CaptureRangePacket` (only during Teach/Observe)

All structs are little-endian, packed. Joint order is **always
`0=R-hip, 1=R-knee, 2=L-hip, 3=L-knee`**.

### Keepalive / link-loss
The active controller must send `OP_NOP` (or any command) **≥ ~3 Hz**. If the
device is *running* and hears nothing for **1 s**, it auto-ramps to a safe home.

---

## 2. Telecommand (TC) catalog

`D` = handled on the device firmware today. `P` = belongs to the PC/bridge side
(calibration math runs off-device; result pushed via `LOAD_COEFFS`).

### Lifecycle / run-state
| op | # | payload | precond | effect | |
|---|---|---|---|---|---|
| `NOP` | 0 | — | — | keepalive (feeds watchdog) | D |
| `ARM` | 1 | — | not e-stopped | axes → CLOSED_LOOP (POS_FILTER or TORQUE per mode) | D |
| `DISARM` | 2 | — | — | all axes → IDLE | D |
| `RUN` | 3 | — | armed | start gait (pos) / start walking (torque) | D |
| `STOP` | 4 | — | running | pos: ramp home → idle; torque: hold (gravity-comp) | D |
| `ESTOP` | 5 | — | any | latch e-stop, idle all | D |
| `ESTOP_RESET` | 6 | — | — | clear latched e-stop | D |
| `IDLE` | 7 | — | — | idle all axes (= disarm) | D |
| `HOME` | 8 | — | running | ramp to home pose (= stop) | D |

### Mode + live tunables
| op | # | payload | effect | |
|---|---|---|---|---|
| `SET_MODE` | 9 | `u8` (0=position, 1=torque) | select control law (apply before ARM) | D |
| `SET_CPS` | 10 | `f32` cycles/s | gait cadence | D |
| `SET_AMP_R` | 11 | `f32` | right-leg trajectory amplitude | D |
| `SET_AMP_L` | 12 | `f32` | left-leg amplitude | D |
| `SET_BW` | 13 | `f32` Hz | POS_FILTER bandwidth — **provisioned via odrivetool, not settable over CAN** | — |
| `SET_ASSIST` | 14 | `f32` [0..1] | therapist assist level (0.5 = legacy) | D |
| `SET_AAN` | 15 | `u8` 0/1 | enable assist-as-needed | D |

### Per-joint config (`joint` field = 0..3)
| op | # | payload | effect | |
|---|---|---|---|---|
| `SET_ROM` | 16 | `f32 min, f32 max` (joint-frame deg) | per-joint soft ROM | D |
| `SET_ENABLE` | 17 | `u8` 0/1 | include/exclude a joint | D |
| `SET_DIR` | 18 | `i8` (-1/+1) | per-joint direction (setup; apply when idle) | D |

### Setup modes (mutually exclusive with gait/torque)
| op | # | payload | effect | |
|---|---|---|---|---|
| `JOG_ENABLE` | 19 | `u8` 0/1 | enter/exit jog (armed) | D |
| `JOG_TARGET` | 20 | `f32` deg (joint field) | move one joint to a ROM-clamped target | D |
| `TEACH_START` | 21 | — | zero-stiffness follow + capture ROM (armed) | D |
| `TEACH_STOP` | 22 | — | exit teach | D |
| `OBSERVE_START` | 23 | — | read-only ROM capture, **motors off** | D |
| `OBSERVE_STOP` | 24 | — | exit observe | D |

During Teach/Observe the device streams `CaptureRangePacket` (see §3).

### Calibration / patient models (PC/bridge side; result pushed to device)
| op | # | payload | effect | |
|---|---|---|---|---|
| `CAL_START`/`CANCEL` | 25/26 | — | empty-exo baseline sweep (collect on bridge, fit, push) | P |
| `TRIM_START`/`CANCEL`/`CLEAR` | 27/28/29 | — | per-patient DC trim (fallback) | P |
| `SWEEP_START`/`CANCEL` | 30/31 | — | patient passive-load characterization sweep | P |
| `TARE` | 32 | — | re-zero the FUTEK load cells (via coproc) | D |
| `LOAD_COEFFS` | 33 | `JointCoeffs` (36 B) | **download a fitted model to the device** | D |
| `FULL_CAL` | 34 | — | ODrive `FULL_CALIBRATION_SEQUENCE` on enabled axes (**disarmed/idle only**; calibrates in RAM — `save_configuration` in odrivetool to persist) | D |

`JointCoeffs` (36 B): `u8 joint, u8 kind(0=empty,1=patient_passive), 2 pad,
f32 coef[5] (a·sinθ + b·cosθ + c·v + d·sign(v) + e, θ in radians),
f32 residStdA, f32 calCps, f32 calAmp`.

> The numpy least-squares **fit runs on the PC/bridge**; the device only
> *evaluates* the polynomial. So "calibrate patient" = collect a TM window →
> fit → `LOAD_COEFFS`. Store the resulting coefficients in Postgres as the
> patient/exo profile.

---

## 3. Telemetry (TM) catalog

### `LogPacket` — 206 B, `0xBB 0x66`, ~50 Hz, broadcast
| off | type | field | units / meaning |
|---|---|---|---|
| 4 | u16 | `seq` | packet counter |
| 8 | u32 | `timeMs` | device uptime ms |
| 12 | u8 | `gaitPhase` | 0=IDLE,1=INIT_SETTLE,2=GAIT,3=HOMING |
| 13 | u8 | `stepIdx` | 0..49 within the 50-pt trajectory |
| 15 | u8 | `sensorHealth` | bit i = joint i load-cell OK |
| 16 | u16 | `flags` | see bitmap below |
| 18 | u16 | `linkAgeMs` | ms since last good coproc packet |
| 20 | f32×4 | `refPos` | reference position **[motor turns]** |
| 36 | f32×4 | `pos` | measured position **[motor turns]** |
| 52 | f32×4 | `vel` | velocity [turns/s] |
| 68 | f32×4 | `cmdTorque` | commanded torque [Nm] (torque mode; 0 in pos) |
| 84 | f32×4 | `measTorque` | FUTEK joint torque [Nm] |
| 100 | f32×4 | `gravTerm` | gravity-comp contribution [Nm] |
| 116 | f32×4 | `ffTerm` | feedforward/spring contribution [Nm] |
| 132 | f32×4 | `iqMeasured` | motor q-axis current [A] |
| 148 | f32×4 | `motorEffort` | iq-derived torque estimate [Nm] |
| 164 | u16×4 | `hbAgeMs` | ODrive heartbeat age per axis (0xFFFF=never) |
| 172 | f32 | `assistR` / `assistL` (176) | assist level |
| 188 | f32 | `ampR` / `ampL` (192) | trajectory amplitude |
| 196 | u16 | `ctrlLoopUs` | control-loop exec time |
| 198 | u16 | `linkCrcFails` / `linkResyncs` (200) | coproc-UART diagnostics |
| 202 | u8 | `crossCheckFault` | bit i = iq-vs-FUTEK divergence on joint i |
| 203 | u8 | `hbErrorByte` | bit i = axis i heartbeat error |

**`flags` bitmap:** 0 `RUN`, 1 `ESTOP`, 2 `SENSOR_ONLINE`, 3 `FF`, 4
`AAN_ENABLED`, 5 `FFP_TRIPPED`, 6 `SEG_GRAV`, 7 `CROSSCHECK_FAULT`, 8
`GAIT_AUTOPROG`, **9 `TORQUE_MODE`** (0=position, 1=torque).

**Positions are MOTOR TURNS.** To get joint-frame degrees:
`deg = dir · turns / TURNS_PER_DEG`, where `TURNS_PER_DEG = 64·1.5/360 ≈ 0.2667`
and `dir` = `{HR:-1, KR:-1, HL:+1, KL:+1}`. (`esp32_link.py` exposes the raw
turns; convert on your side or reuse `joint.py`/`constants.py` factors.)

### `CaptureRangePacket` — 38 B, `0xBB 0x77` (only during Teach/Observe)
`u8 start0/1, u8 mode(1=jog,2=teach,3=observe), u8 validMask, f32 minDeg[4],
f32 maxDeg[4], u16 crc`. Use it to populate per-joint ROM after a Teach/Observe.

---

## 4. Functional capabilities — clinical action → TC/TM

| You want to… | Do this |
|---|---|
| **Monitor live** | Listen UDP `:47000`, decode `LogPacket`. Show pos→deg, torques, flags, faults. No control needed. |
| **Select a patient** | Push the patient profile: `LOAD_COEFFS` (empty + patient_passive per joint), `SET_ROM` per joint, `SET_ASSIST`, `SET_AAN`, `SET_MODE`, `SET_CPS`, `SET_AMP_R/L`, `SET_ENABLE` per joint. |
| **Start a session** | `ARM` → wait `RUN` flag conditions → `RUN`; record the TM stream to Postgres; `STOP` → `DISARM`. |
| **Live-tune** | `SET_CPS` / `SET_AMP_*` / `SET_ASSIST` / `SET_AAN` while running. |
| **Set ROM by hand** | `TEACH_START` (or `OBSERVE_START`), read `CaptureRangePacket`, then `SET_ROM` from the captured min/max; `TEACH_STOP`. |
| **Jog a joint** | `JOG_ENABLE 1`, `JOG_TARGET(joint, deg)`, … `JOG_ENABLE 0`. |
| **Calibrate** | Drive a sweep (`RUN` empty-exo, or `SWEEP_START` for patient), collect the TM window on the bridge, fit on the PC, `LOAD_COEFFS`. |
| **Tare load cells** | `TARE` with the limb at rest. |
| **Emergency stop** | `ESTOP` (allowed from any head) — and the hardware button. |

---

## 5. PostgreSQL data model mapping

| Postgres entity | Maps to |
|---|---|
| **Exoskeleton params** (per device) | empty-exo baseline coeffs (`JointCoeffs kind=empty` ×4), motor-turn envelopes, per-axis `Kt` |
| **Patient profile** | per-patient `JointCoeffs kind=patient_passive` ×4, per-joint ROM, `enable`, `dir`, `assist`, `aan`, `mode`, `cps`, `ampR/L` |
| **Session record** | start/end time, patient id, profile snapshot, **downsampled `LogPacket` stream** (pos/vel/torque/iq/flags) |
| **Events** | e-stop, faults (`crossCheckFault`, `hbErrorByte`), link-loss, mode changes |

Loading a patient = read their profile from Postgres → emit the corresponding
`LOAD_COEFFS` + `SET_*` commands. Saving a session = capture the TM stream.

---

## 6. Suggested ROS / WebSocket surface (bridge-exposed)

The bridge translates JSON/ROS ↔ binary. Suggested shapes:

- **TM out** — ROS topic `/pgear/telemetry` (or WS broadcast): the decoded
  `LogPacket` as JSON (+ derived joint-deg), at 50 Hz or downsampled.
- **State** — `/pgear/state`: armed/running/estop/mode/online + fault bits.
- **TC in** — ROS services/actions or WS messages, one per action:
  `arm`, `disarm`, `run`, `stop`, `estop`, `set_mode`, `set_param{name,value}`,
  `load_profile{patient_id}`, `set_rom{joint,min,max}`, `jog{joint,deg}`,
  `teach_start/stop`, `observe_start/stop`, `tare`, `request_control` /
  `release_control` (the ownership token).

Example WS command → bridge → TC:
```json
{ "type": "set_param", "name": "cps", "value": 0.36 }   // → CommandPacket OP_SET_CPS
{ "type": "estop" }                                      // → OP_ESTOP (always allowed)
```

---

## 7. Reaching FULL Python-GUI parity (host-side logic — use the bridge)

§§2–3 fully specify the **device interface** — enough to ARM/RUN/tune/jog/teach/
e-stop. But several things the Python GUI does are **host-side algorithms that
are deliberately NOT in the wire protocol**. A head built from §§2–3 alone can
control the device but cannot calibrate or show assist feedback like the GUI.

| GUI feature | On the wire? | Where the logic lives |
|---|---|---|
| Empty-exo **calibration** (sweep → 5-coef least-squares fit + R²/guards) | no | `calibrator.MultiJointCalibrator` |
| Patient **"Characterize"** (multi-cadence passive sweep + robust fit) / DC trim | no | `worker` + `calibrator` |
| Live per-joint **patient torque + status** (iq − model) | no (derivable) | `worker._emit_patient_torque` |
| **AAN factor** / **drift watcher** feedback | no | `worker` |
| Per-joint **direction** (`SET_DIR`) | ✅ now handled on device | — |

**Do NOT reimplement the 5-coef fit in JavaScript.** Use the **bridge**
([`pgear_tools/pi_gui/pgear_pi/bridge.py`](https://github.com/Keikibayev/pgear_tools/blob/feature/patient-profiles-cp-support/pi_gui/pgear_pi/bridge.py),
docs in [`pi_gui/BRIDGE.md`](https://github.com/Keikibayev/pgear_tools/blob/feature/patient-profiles-cp-support/pi_gui/BRIDGE.md)),
which hosts this logic by **reusing the production modules** and exposes it as
JSON ops. This is how a head reaches full parity.

### 7.1 Derived telemetry — already in the bridge's `telemetry` message
- `patient_torque[4]` [Nm] and `patient_status[4]`
  (`"ok"` / `"no_baseline"` / `"not_gait"`). The bridge computes `iq − model`
  per joint from the loaded coefficients — **no head-side math needed**.
- Plus everything from the LogPacket, with `pos_deg` already converted.

### 7.2 Calibration ops — bridge runs the sweep → fit → `LOAD_COEFFS` → replies
- `{"type":"calibrate","joints":[0,1,2,3],"duration":30}` — empty-exo baseline.
  **Precondition: device ARM + RUN (gait), exo EMPTY.** Reply:
  `{"type":"calibrate_result","kind":0,"joints":{idx:{coef,r2,resid_std,cps,amp,n}},"skipped":[...]}`.
- `{"type":"characterize","joints":[...],"cadences":[0.08,0.12,0.16]}` — patient
  passive-load model (multi-cadence, robust outlier-rejecting fit).
  **Precondition: device ARM + RUN, patient strapped + passive.** Same reply
  shape (`kind:1`).
- **E-STOP from any head cancels a running sweep.** Store the returned `coef`
  (+ r2/resid) in Postgres; re-apply later.

### 7.3 Apply models / profiles
- `{"type":"load_coeffs","joint":0,"kind":0|1,"coef":[a,b,c,d,e],
   "resid_std":0,"cal_cps":0,"cal_amp":0}` — one model → device **and** bridge cache.
- `{"type":"load_profile","profile":{…}}` — a full patient profile in one shot.
- `{"type":"clear_models"}` — drop the bridge's cached models.

### 7.4 Patient profile JSON schema (what to store in Postgres)
```json
{
  "mode": "position",                 // or "torque"
  "cps": 0.36, "amp_r": 0.5, "amp_l": 0.5,
  "assist": 0.5, "aan": true,
  "coeffs": [[0,0,[a,b,c,d,e]], [0,1,[a,b,c,d,e]], ...],  // [joint, kind, coef5]
  "rom":    {"0": [-18, 25], "1": [-6, 31]},              // joint -> [min,max] deg
  "enable": {"0": true, "1": true}
}
```
**Split (important):** `kind=0` (`empty`) coeffs describe the **bare
exoskeleton** → store **once per device** (exo params table). `kind=1`
(`patient_passive`) + `rom`/`assist`/`aan`/`cps`/`amp` → **per patient**.

### 7.5 So a head reaches full GUI parity by
1. reading the enriched telemetry (incl. `patient_torque`) — done for you;
2. calling `calibrate` / `characterize` — the fit runs in the bridge (reusing
   `calibrator.py`, the same code the GUI uses);
3. storing/applying profiles via `load_profile`.

Not-yet-bridged GUI extras (cosmetic, add later if needed): the exact
"Assist: easing for KR …" label text and the drift-watcher warning string.

## 8. Versioning & keeping in sync
- `protocol.h` is the contract; `LogPacket.version == 3`. Validate it.
- If a struct changes, bump the version and update both `esp32_link.py` and the
  bridge. Struct sizes are guarded by `static_assert` on the device.
- Reuse `esp32_link.py` as the bridge's device-facing client (it already
  implements every frame here and is wire-verified).
