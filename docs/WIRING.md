# P.GEAR v8 — wiring between units

Five units: **PC** (WiFi), **Main controller** (ESP32-S3-Touch-LCD-7),
**Coproc** (ESP32 + ADS1256), **2× ODrive**, **4× load cell load cell**.

```
            WiFi hotspot
            /          \
        [PC]          [Main ESP32-S3-Touch-LCD-7]
                       |   |   |
              CAN(19/20)|  |   |UART2 (43/44)
                        |  |   |
                  [ODrive R][ODrive L]   [Coproc ESP32]
                                           |  SPI
                                        [ADS1256]
                                        | | | |  (4 differential bridges)
                                     [load cell x4 load cells]
```

> Common ground everywhere: tie GND of Main, Coproc, ADS1256, ODrives, and the
> load-cell excitation return together. CAN especially needs a shared ground.

---

## 1. Main ESP32-S3-Touch-LCD-7 ↔ 2× ODrive — CAN bus

The board has an **onboard CAN transceiver**; you wire only CAN_H / CAN_L from
the board's **CAN connector** to the ODrive CAN bus. (Firmware sets CH422G
EXIO5 = HIGH so GPIO19/20 route to the transceiver, not native USB.)

| Main CAN connector | ODrive R | ODrive L |
|---|---|---|
| CAN_H | CAN_H | CAN_H (daisy-chained) |
| CAN_L | CAN_L | CAN_L (daisy-chained) |
| GND   | GND   | GND |

- **250 kbps**, both ODrives on the same bus.
- **120 Ω termination at both physical ends** of the bus: the board's CAN
  jumper at one end, an ODrive terminator at the far end.
- ODrive node IDs: **R-hip=10, R-knee=11, L-hip=2, L-knee=3** (axis1=hip,
  axis0=knee on each board). Set these in `odrivetool`.

## 2. Main ↔ Coproc — UART (UART2 connector, GPIO43/44)

3 wires + power. Cross TX↔RX.

| Main (UART2 hdr) | Coproc ESP32 |
|---|---|
| GPIO43 TXD → | GPIO16 RXD |
| GPIO44 RXD ← | GPIO17 TXD |
| GND ↔ | GND |
| 5V (opt) → | 5V (power the coproc from main, optional) |

- **460800 baud, 8N1.**
- ⚠ These pins are shared with the CH343P USB-console via the board's
  UART1/UART2 switch — set the switch to **UART2** for coproc operation. The
  text console is then unavailable (debug via WiFi). Flash with the switch on
  **UART1/CH343P** (coproc unplugged), USB CDC On Boot **Disabled**.

## 3. Coproc ESP32 ↔ ADS1256 — SPI (mode 1, ~1.7 MHz)

All ADS1256 lines are on ONE board-edge row → a single connector carries them.

| Coproc ESP32 | ADS1256 module |
|---|---|
| GPIO18 | SCLK |
| GPIO23 | DIN (MOSI) |
| GPIO19 | DOUT (MISO) |
| GPIO5  | CS |
| GPIO21 | DRDY |
| 3V3/5V | VCC (digital, per module spec) |
| GND    | DGND |

- Module = **LC Technology ADS1256 v1.1**. **`PDWN` is present → tie it to
  3.3 V on the module** (jumper to the adjacent VCC/3V3 pad) so it sits in run
  mode. No ESP GPIO is used for it — we never power the ADS down in software.
- **`RESET` is not broken out** on this module → the firmware resets over SPI
  (the `0xFE` command) at init; no hardware RESET line needed.
- So the ESP-side connector is just these **5 signal pins + 3V3 + GND**, all in
  the same header row (free pins; the row's 16/17 go to the main-board UART,
  1/3 are the USB console, 2 is the LED, 15 is a strap — keep clear of those).
- ADS1256 `SYNC` line (if broken out separately): tie HIGH.
- (Classic-ESP32 pins shown; on an ESP32-S3 coproc avoid GPIO19/20.)

## 4. ADS1256 ↔ 4× load cell load cells — analog bridges

Each load cell is a strain-gauge **Wheatstone bridge** (4-wire): EXC+, EXC-,
SIG+, SIG-. The ADS1256 PGA (×64) reads the bridge directly — no separate
amplifier. Wire each cell to one **differential pair**:

| Load cell | ADS1256 inputs |
|---|---|
| Cell 0 (R-hip)  | SIG+ → AIN0, SIG- → AIN1 |
| Cell 1 (R-knee) | SIG+ → AIN2, SIG- → AIN3 |
| Cell 2 (L-hip)  | SIG+ → AIN4, SIG- → AIN5 |
| Cell 3 (L-knee) | SIG+ → AIN6, SIG- → AIN7 |

- **Excitation (shared):** all EXC+ → ADS1256 **AVDD** (≈5 V); all EXC- →
  **AGND**. (Keep analog AVDD/AGND clean; the common Waveshare ADS1256 module
  provides a buffered 5 V analog supply.)
- Channel→joint order matches the canonical **0=R-hip, 1=R-knee, 2=L-hip,
  3=L-knee** so `MOMENT_ARM_M[]` and the GUI line up. Sign may invert per cell
  — fix with a negative `CalAdjust` ratio after calibration.
- load cell wire colors vary by model — confirm EXC/SIG from its datasheet.

## 5. PC ↔ everything — WiFi (no wires)

PC and the Main ESP32-S3 both join your **external hotspot** (station mode).
Telemetry = UDP; commands = reliable channel; link-loss watchdog on the ESP32.
(Phase 4.)

## 6. Safety wiring (Phase 5)

**No physical endstops in this setup.** The firmware **motor-turn envelope
clamp** (`safety_clamp_turns`) is therefore the **only** hard ROM limit — every
commanded position is clamped to it before TX. The envelope is **direction-aware**
(`joint_turn_limits`): the base values are the +dir (left-leg) motor frame
(HIP −6..+8 / KNEE −2..+10 turns), and the right-leg joints (dir −1, mirror-mounted)
get the **mirrored** range (HIP −8..+6 / KNEE −10..+2) so flexion has the large
range on BOTH legs. A single global [−2,+10] strangled the right knee to ~7.5°.

- **E-STOP button (NC):**
  1. → a Main ESP32 GPIO to GND (`ESTOP_GPIO`, default 6 — **verify a free
     pin**), `INPUT_PULLUP`; firmware latches e-stop and idles all axes.
  2. **AND** a hardware path that cuts motor power **without firmware** — wire
     the same button into a **contactor / the ODrive power rail** so an e-stop
     works even if the MCU hangs. This is the real emergency stop; the GPIO is
     the firmware's awareness of it.
- Do **not** rely on a WiFi command for emergency stop.
- Because there are no endstops, double-check the envelope constants match the
  mechanical hard stops before running with a patient.

---

### Quick power summary
- Main board: USB-C 5 V (or its DC input).
- ODrives: their own motor DC supply (per ODrive docs).
- Coproc + ADS1256: 5 V (from Main's 5 V or a small dedicated supply).
- **All grounds common.**
