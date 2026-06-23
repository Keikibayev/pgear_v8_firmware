// ============================================================================
// pgear_main.ino  —  P.GEAR v8 ESP32-S3 main controller
// ----------------------------------------------------------------------------
// PHASE 0 SCAFFOLD: headless board bring-up + dual-core skeleton.
//   core 1 : real-time control task (fixed-rate tick; empty for now)
//   core 0 : Arduino loop() — comms + text status + USB-CDC echo
//
// Build: Arduino-ESP32, board "ESP32-S3", enable *USB CDC On Boot* so the
// `Serial` object is the native USB-CDC link to the PC. (Verify the
// Waveshare ESP32-S3-Touch-LCD-7 GPIO map before wiring CAN/coproc/screen.)
//
// Roadmap per docs/PLAN.md:
//   P1 can_odrive.h   P2 gait/gait_engine.h   P3 coproc_link/host_link.h
//   P5 safety.h       P6 control.h + calib.h  P8 status_screen.h
// ============================================================================
#include "protocol.h"
#include "constants.h"
#include "board_io.h"       // CH422G: route GPIO19/20 to CAN (not native USB)
#include "can_odrive.h"     // [Phase 1] TWAI + ODrive protocol
#include "gait_engine.h"    // [Phase 2] gait state machine + setpoints
#include "coproc_link.h"    // [Phase 3] ADS1256 coproc over UART
#include "calib.h"          // [Phase 4] downloaded model store + predict
#include "host_link.h"      // [Phase 4] WiFi: UDP telemetry + TCP commands
#include "safety.h"         // [Phase 5] e-stop, hb watchdog, envelope clamp, sensor wd
#include "control.h"        // [Phase 6] pos-mode AAN + torque-mode impedance
#include "modes.h"          // [Phase 7] Jog / Teach-ROM / Observe
#include "status_screen.h"  // [Phase 8] optional on-device text status (default off)
#include "wifi_config.h"    // HOST_TELEM_HZ
#include <string.h>

// Coproc UART (43/44) shares pins with the CH343P text console via the board's
// UART switch — they can't both run. Set USE_COPROC=1 once the coproc is wired
// (switch on UART2) and debug moves to WiFi (Phase 4). Keep 0 for the Phase-2
// serial-console bench-key workflow.
#define USE_COPROC 0

// ---- Control task configuration --------------------------------------------
static const uint32_t CTRL_HZ        = 250;                 // real-time loop rate
static const uint32_t CTRL_PERIOD_US = 1000000UL / CTRL_HZ; // 4000 us
static const BaseType_t CTRL_CORE    = 1;                   // pin control to core 1
static const BaseType_t COMMS_CORE   = 0;                   // loop() runs on core 0
static const int      GAIT_SUBDIV    = CTRL_HZ / GAIT_STEP_HZ;   // 250/50 = 5
static const float    GAIT_DT_S      = 1.0f / (float)GAIT_STEP_HZ;

// ---- Shared state (cross-core; expand into proper structs in later phases) --
volatile uint32_t g_ctrlTicks   = 0;
volatile uint16_t g_ctrlLoopUs  = 0;   // last control-loop exec time (-> LogPacket)
volatile uint16_t g_ctrlLoopUsMax = 0; // peak control-loop time since last status (jitter probe)
volatile bool     g_quiet       = false; // 'q' mutes the periodic [status] spam
volatile bool     g_estop       = false;
volatile bool     g_armed       = false;
volatile bool     g_running     = false;
volatile uint32_t g_fullcal_until_ms = 0;   // watchdog feed paused until this (FULL_CAL)

// Pending operator command (set from comms core, consumed by control core so
// all CAN TX stays on one core). TEMP bench control — replaced by CommandPacket
// in Phase 4.
enum PgCmd { CMD_NONE = 0, CMD_ARM, CMD_RUN, CMD_STOP, CMD_DISARM, CMD_ESTOP, CMD_FULLCAL };
volatile PgCmd g_pendingCmd = CMD_NONE;

// Stored supervisor state (consumed by the control law in Phase 6).
volatile uint8_t g_controlMode = MODE_POSITION;
volatile bool    g_aanEnabled  = false;
volatile float   g_assistLevel = 0.5f;
volatile float   g_torqueAssistGain = 1.0f;   // [6b] live K_assist multiplier (the "go")
volatile bool    g_freeRun         = false;   // [6b] BENCH: torque phase self-advances
volatile float   g_torqueCapMult   = 1.0f;    // [6b] live multiplier on per-joint torque caps
volatile uint16_t g_logSeq     = 0;
volatile uint8_t g_crossCheckFault = 0;
volatile uint8_t g_hbErrorByte     = 0;

static GaitEngine   g_engine;
static TorqueState  g_torque;     // [Phase 6b] torque-mode controller state
static PatientTorque g_patient;   // shared patient-torque estimate
volatile float      g_aanFactor = 1.0f;
volatile uint8_t    g_setupMode = SETUP_NONE;   // [Phase 7] jog/teach/observe
static ModeState    g_modeState;

// ---- run-state orchestration (executed on the control core) ----------------
static void armDrives() {
  bool torque = (g_controlMode == MODE_TORQUE);
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!g_engine.joints[i].enabled) continue;
    can_clear_errors(i);
    if (torque) can_set_control_mode(i, CM_TORQUE, IM_PASSTHROUGH);
    else        can_set_control_mode(i, CM_POSITION, IM_POS_FILTER); // bw provisioned
    can_set_limits(i, ABS_VEL_LIM, 10.0f);                // bench-safe current cap
    can_set_axis_state(i, AXIS_CLOSED_LOOP);
  }
  // reset torque-mode state so it starts from the current pose
  g_torque.phase01 = 0.0f; g_torque.g_filt = 1.0f;
  for (int i = 0; i < PG_NJOINTS; i++) g_torque.prev_tau[i] = 0.0f;
  g_armed = true;
  Serial.printf("[ctrl] ARMED (%s)\n", torque ? "TORQUE" : "POS_FILTER");
}

static void disarmDrives() {
  g_running = false;
  g_setupMode = SETUP_NONE;
  g_engine.go_idle();
  can_idle_all();
  g_armed = false;
  Serial.println("[ctrl] DISARMED");
}

static void doEstop() {
  g_running = false;
  g_setupMode = SETUP_NONE;
  g_engine.go_idle();
  can_idle_all();
  g_armed = false;
  g_estop = true;
  Serial.println("[ctrl] *** E-STOP *** (all axes IDLE)");
}

// ODrive FULL_CALIBRATION_SEQUENCE on enabled axes — disarmed/idle only.
static void doFullCal() {
  if (g_estop || g_armed || g_running) {
    Serial.println("[ctrl] FULL CAL refused — disarm/idle first");
    return;
  }
  // Pause the watchdog re-assert so it can't clobber the cal request back to IDLE.
  g_fullcal_until_ms = millis() + 30000;
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!g_engine.joints[i].enabled) continue;
    can_clear_errors(i);
    can_set_axis_state(i, AXIS_FULL_CAL);     // ODrive state 3
  }
  can_dump_status("after FULL_CAL tx");   // tx_failed>0 or state!=1 => CAN TX problem
  Serial.println("[ctrl] FULL CAL: ODrive calibration started (motor moves/beeps; "
                 "watchdog paused 30s; save_configuration in odrivetool to persist)");
}

static void handlePendingCmd() {
  PgCmd c = g_pendingCmd;
  if (c == CMD_NONE) return;
  g_pendingCmd = CMD_NONE;
  switch (c) {
    case CMD_ARM:    if (!g_estop) armDrives(); break;
    case CMD_RUN:    if (g_armed) {
                       g_setupMode = SETUP_NONE;   // gait/torque preempts setup modes
                       if (g_controlMode == MODE_TORQUE) {
                         g_torque.phase01 = 0.0f; g_running = true;
                         Serial.println("[ctrl] RUN (torque)");
                       } else {
                         g_engine.start_gait(); g_running = true;
                         Serial.println("[ctrl] RUN (gait)");
                       }
                     } break;
    case CMD_STOP:   if (g_running) {
                       if (g_controlMode == MODE_TORQUE) {
                         g_running = false;   // controller holds via gravity-comp
                         Serial.println("[ctrl] STOP (torque hold)");
                       } else {
                         g_engine.start_homing();
                         Serial.println("[ctrl] STOP -> homing");
                       }
                     } break;
    case CMD_DISARM: disarmDrives(); break;
    case CMD_ESTOP:  doEstop(); break;
    case CMD_FULLCAL: doFullCal(); break;
    default: break;
  }
}

// ---- LogPacket builder (comms core) ----------------------------------------
static void build_logpacket(LogPacket* p) {
  memset(p, 0, sizeof(*p));
  p->start0 = 0xBB; p->start1 = 0x66; p->version = PG_LOGPACKET_VERSION;
  p->seq = g_logSeq++;
  p->headerCrc = pg_crc16_ccitt((const uint8_t*)p, 6);
  p->timeMs = millis();
  if (g_controlMode == MODE_TORQUE) {
    p->gaitPhase = g_running ? (uint8_t)PH_GAIT : (uint8_t)PH_IDLE;
    p->stepIdx = (uint8_t)(g_torque.phase01 * 50.0f);
  } else {
    p->gaitPhase = (uint8_t)g_engine.phase;
    p->stepIdx = (uint8_t)(g_engine.phase01 * 50.0f);
  }
  p->profileSlot = 0xFF;

  BusTelemetry snap; can_snapshot(&snap);
  CoprocData cd;
#if USE_COPROC
  coproc_get(&cd);
#endif
  uint8_t health = 0;
  for (int i = 0; i < PG_NJOINTS; i++) {
    const AxisTelemetry& a = snap.j[i];
    p->refPos[i]      = g_engine.joints[i].last_ref_turns;
    p->pos[i]         = a.pos_turns;
    p->vel[i]         = a.vel_turns_s;
    p->iqMeasured[i]  = a.iq_measured_a;
    p->motorEffort[i] = a.iq_measured_a * JOINT_NM_PER_A;
    p->measTorque[i]  = cd.torqueNm[i];
    uint32_t age = a.last_frame_ms ? (millis() - a.last_frame_ms) : 0xFFFFu;
    if (age > 0xFFFFu) age = 0xFFFFu;
    p->hbAgeMs[i]     = (uint16_t)age;
    if (cd.online && !(cd.sensorFlags & (1 << i))) health |= (1 << i);
  }
  p->sensorHealth = health;
  p->linkAgeMs = (uint16_t)(cd.ageMs > 0xFFFFu ? 0xFFFFu : cd.ageMs);

  uint16_t flags = 0;
  if (g_running)        flags |= LP_FLAG_RUN;
  if (g_estop)          flags |= LP_FLAG_ESTOP;
  if (cd.online)        flags |= LP_FLAG_SENSOR_ONLINE;
  if (g_aanEnabled)     flags |= LP_FLAG_AAN_ENABLED;
  if (g_controlMode == MODE_TORQUE) flags |= LP_FLAG_TORQUE_MODE;
  p->flags = flags;

  p->ampR = g_engine.amp_r; p->ampL = g_engine.amp_l;
  p->assistR = g_assistLevel; p->assistL = g_assistLevel;
  p->ctrlLoopUs = g_ctrlLoopUs;
  p->crossCheckFault = g_crossCheckFault;
  p->hbErrorByte = g_hbErrorByte;

  p->crc = pg_crc16_ccitt((const uint8_t*)p, sizeof(LogPacket) - 2);
}

// ---- CommandPacket dispatch (comms core) -----------------------------------
static float payload_f32(const CommandPacket& c, int off) {
  float v = 0.0f; if (off + 4 <= c.len) memcpy(&v, &c.payload[off], 4); return v;
}
static void dispatchCommand(const CommandPacket& c) {
  switch (c.opcode) {
    case OP_NOP:          break;                       // keepalive (RX ts updated)
    case OP_ARM:          g_pendingCmd = CMD_ARM;    break;
    case OP_DISARM:       g_pendingCmd = CMD_DISARM; break;
    case OP_RUN:          g_pendingCmd = CMD_RUN;    break;
    case OP_STOP:         g_pendingCmd = CMD_STOP;   break;
    case OP_ESTOP:        g_pendingCmd = CMD_ESTOP;  break;
    case OP_ESTOP_RESET:  g_estop = false;           break;
    case OP_IDLE:         g_pendingCmd = CMD_DISARM; break;   // idle all axes
    case OP_HOME:         g_pendingCmd = CMD_STOP;   break;   // ramp to home pose
    case OP_SET_MODE:     if (c.len >= 1) g_controlMode = c.payload[0]; break;
    case OP_SET_CPS:      g_engine.cps   = payload_f32(c, 0); break;
    case OP_SET_AMP_R:    g_engine.amp_r = payload_f32(c, 0); break;
    case OP_SET_AMP_L:    g_engine.amp_l = payload_f32(c, 0); break;
    case OP_SET_ASSIST:   g_assistLevel  = payload_f32(c, 0); break;
    case OP_SET_AAN:      g_aanEnabled   = (c.len >= 1 && c.payload[0]); break;
    case OP_SET_TORQUE_ASSIST: { float g = payload_f32(c, 0);
                          g_torqueAssistGain = g < 0.0f ? 0.0f : (g > 10.0f ? 10.0f : g); } break;
    case OP_SET_FREE_RUN: { bool fr = (c.len >= 1 && c.payload[0]);
                          if (!fr) g_torque.g_filt = 0.0f;  // re-earn cooperation on exit
                          g_freeRun = fr; } break;
    case OP_SET_TORQUE_CAP: { float m = payload_f32(c, 0);
                          g_torqueCapMult = m < 0.1f ? 0.1f : (m > 4.0f ? 4.0f : m); } break;
    case OP_SET_ROM:      if (c.joint < PG_NJOINTS) {
                            g_engine.joints[c.joint].rom_min_deg = payload_f32(c, 0);
                            g_engine.joints[c.joint].rom_max_deg = payload_f32(c, 4);
                          } break;
    case OP_SET_ENABLE:   if (c.joint < PG_NJOINTS && c.len >= 1)
                            g_engine.joints[c.joint].enabled = c.payload[0]; break;
    case OP_SET_DIR:      if (c.joint < PG_NJOINTS && c.len >= 1)
                            g_engine.joints[c.joint].direction = (int8_t)c.payload[0]; break;
    case OP_TARE:
#if USE_COPROC
      coproc_request_tare();
#endif
      break;
    case OP_LOAD_COEFFS:  if (c.len >= (int)sizeof(JointCoeffs)) {
                            JointCoeffs jc; memcpy(&jc, c.payload, sizeof(JointCoeffs));
                            calib_load_coeffs(&jc);
                          } break;
    case OP_FULL_CAL:     g_pendingCmd = CMD_FULLCAL; break;   // ODrive motor cal
    // ---- setup modes [Phase 7] (mutually exclusive with gait/torque) -------
    case OP_JOG_ENABLE:   g_running = false;
                          g_setupMode = (c.len >= 1 && c.payload[0]) ? SETUP_JOG : SETUP_NONE;
                          break;
    case OP_JOG_TARGET:   modes_set_jog_target(&g_modeState, c.joint, payload_f32(c, 0), &g_engine);
                          break;
    case OP_TEACH_START:  g_running = false; modes_reset_capture(&g_modeState);
                          g_setupMode = SETUP_TEACH; break;
    case OP_OBSERVE_START:g_running = false; modes_reset_capture(&g_modeState);
                          g_setupMode = SETUP_OBSERVE; break;   // motors off
    case OP_TEACH_STOP:
    case OP_OBSERVE_STOP: g_setupMode = SETUP_NONE; break;
    // OP_SET_BW/DIR, CAL_*, TRIM_*, SWEEP_* : driven from the PC; not on-chip
    default: break;
  }
}

// ============================================================================
// Real-time control task — pinned to core 1, runs at CTRL_HZ with low jitter.
// Phase 2: position-mode gait. (Coproc fusion P3, safety P5, AAN/torque P6.)
// ============================================================================
static void controlTask(void *arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / CTRL_HZ);
  int subdiv = 0;

  for (;;) {
    uint32_t t0 = micros();

    handlePendingCmd();

    // Telemetry snapshot -> feed measured positions into the engine.
    BusTelemetry snap;
    can_snapshot(&snap);
    for (int i = 0; i < PG_NJOINTS; i++)
      g_engine.update_pos(i, snap.j[i].pos_turns);

    // Safety supervisor (e-stop button, hb watchdog, sensor watchdog, cross-check).
    CoprocData cd;
#if USE_COPROC
    coproc_get(&cd);
#endif
    uint8_t cc = 0, hb = 0;
    bool trip = safety_tick(g_armed, g_running, &snap, &cd, &cc, &hb);
    g_crossCheckFault = cc; g_hbErrorByte = hb;
    if (trip && !g_estop) doEstop();

    // Control law at GAIT_STEP_HZ (every GAIT_SUBDIV ticks).
    if (++subdiv >= GAIT_SUBDIV) {
      subdiv = 0;
      control_patient_torque(&snap, &cd, &g_engine, &g_patient);  // shared estimate

      if (g_setupMode != SETUP_NONE) {
        // [Phase 7] setup modes preempt gait/torque
        switch (g_setupMode) {
          case SETUP_JOG:     modes_jog_tick(&g_engine, &g_modeState, g_armed); break;
          case SETUP_TEACH:   modes_teach_tick(&g_engine, &snap, &g_modeState, g_armed); break;
          case SETUP_OBSERVE: modes_observe_tick(&g_engine, &snap, &g_modeState); break;
          default: break;
        }
      } else if (g_controlMode == MODE_TORQUE) {
        // [6b] impedance: runs whenever ARMED (gravity-comp hold/float when not
        // running; walks when running). No homing — STOP just stops the phase.
        if (g_armed) {
          float mnm[PG_NJOINTS]; bool has[PG_NJOINTS];
          control_torque_step(GAIT_DT_S, g_running, g_freeRun, g_aanEnabled,
                              g_torqueAssistGain, g_torqueCapMult, g_engine.cps,
                              &snap, &cd, &g_patient, &g_engine, &g_torque, mnm, has);
          for (int i = 0; i < PG_NJOINTS; i++)
            if (has[i]) can_set_input_torque(i, mnm[i]);
        }
      } else {
        // [6] position-mode gait + AAN phase-yielding
        if (g_running) {
          g_aanFactor = control_pos_aan(GAIT_DT_S, &g_engine, &g_patient,
                                        g_aanEnabled, g_assistLevel);
          g_engine.cps_modifier = g_aanFactor;
          float turns[PG_NJOINTS]; bool has[PG_NJOINTS];
          g_engine.tick(GAIT_DT_S, turns, has);
          if (g_armed)
            for (int i = 0; i < PG_NJOINTS; i++)
              if (has[i])
                can_set_input_pos(i, safety_clamp_turns(i, turns[i]));  // envelope = sole hard limit
          if (g_engine.phase == PH_HOMING &&
              (g_engine.all_home_arrived() || g_engine.homing_timed_out())) {
            g_engine.go_idle(); g_running = false;
            Serial.println("[ctrl] homing done -> IDLE");
          }
        }
      }
    }
    g_ctrlTicks++;

    g_ctrlLoopUs = (uint16_t)(micros() - t0);
    if (g_ctrlLoopUs > g_ctrlLoopUsMax) g_ctrlLoopUsMax = g_ctrlLoopUs;  // catch spikes
    vTaskDelayUntil(&lastWake, period);  // fixed-rate, jitter-bounded
  }
}

// ============================================================================
void setup() {
  Serial.begin(115200);     // native USB-CDC to the PC (CDC On Boot)
  delay(200);
  Serial.println("\n[pgear_v8] main controller boot — Phase 0 scaffold");
  Serial.printf("[pgear_v8] control loop %lu Hz on core %d, comms on core %d\n",
                (unsigned long)CTRL_HZ, (int)CTRL_CORE, (int)COMMS_CORE);
  Serial.printf("[pgear_v8] LogPacket=%u B  Command max payload=%u B\n",
                (unsigned)sizeof(LogPacket), (unsigned)PG_CMD_MAXPAYLOAD);

  // --- TODO P0: status_screen_init();  (panel bring-up, NO LVGL — text only)
  // Route GPIO19/20 to the CAN transceiver (away from native USB). The PC link
  // is the SEPARATE CH343P "USB TO UART" port, so this costs us nothing.
  board_io_init();
  board_io_set_can_mode(true);
  if (can_odrive_init()) {
    can_idle_all();                  // safe default: every axis IDLE at boot
  } else {
    Serial.println("[pgear_v8] WARNING: CAN init failed — check transceiver/pins");
  }
  g_engine.init_defaults();          // [Phase 2] ROM/dir/init-pose per joint
  safety_init();                     // [Phase 5] e-stop GPIO + watchdogs
#if USE_COPROC
  coproc_link_init();                // [Phase 3] ADS1256 coproc on UART2 (43/44)
#endif
  host_link_init();                  // [Phase 4] WiFi STA: UDP telemetry + TCP cmds
  status_screen_init();              // [Phase 8] on-device text status (no-op if off)

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr,
                          configMAX_PRIORITIES - 2, nullptr, CTRL_CORE);
  Serial.println("[pgear_v8] control task started");
  Serial.println("[pgear_v8] bench keys: a=arm r=run s=stop d=disarm e=estop");
}

// ============================================================================
// loop() runs on core 0 — comms, telemetry, status. Never blocks the RT loop.
// ============================================================================
void loop() {
  static uint32_t lastStatus = 0;
  static uint32_t lastWdFeed = 0;
  uint32_t now = millis();

  // TEMP bench control over the serial console (replaced by CommandPacket /
  // WiFi in Phase 4). One key per action; consumed on the control core.
  // GUARDED: GPIO43/44 are shared between the CH343P console (UART switch =
  // UART1) and the coproc (switch = UART2). In a coproc build the coproc's
  // sensor stream lands on this same UART; reading it as keystrokes fires
  // random commands (a stray 'c' -> repeated FULL_CAL, 's'/'e'/'d' -> kills a
  // running cal). So only honor bench keys when the coproc is NOT on the bus —
  // in coproc/production builds, drive everything over WiFi instead.
#if !USE_COPROC
  while (Serial.available()) {
    int c = Serial.read();
    switch (c) {
      case 'a': g_pendingCmd = CMD_ARM;    break;
      case 'r': g_pendingCmd = CMD_RUN;    break;
      case 's': g_pendingCmd = CMD_STOP;   break;
      case 'd': g_pendingCmd = CMD_DISARM; break;
      case 'i': g_pendingCmd = CMD_DISARM; break;   // idle all axes (= disarm here)
      case 'e': g_pendingCmd = CMD_ESTOP;  break;
      case 'E': g_estop = false; Serial.println("[ctrl] estop cleared"); break;
      case 'c': g_pendingCmd = CMD_FULLCAL; break;   // firmware-driven ODrive cal
      case 'q': g_quiet = !g_quiet;                  // mute/unmute periodic [status]
                Serial.printf("[status] periodic output %s\n", g_quiet ? "OFF (q to resume)" : "ON");
                break;
      default: break;
    }
  }
#endif  // !USE_COPROC

  // ODrive watchdog feed @ ~10 Hz (re-asserts known axis state).
  if (now - lastWdFeed >= (uint32_t)(1000.0f / ODRIVE_WD_FEED_HZ)) {
    lastWdFeed = now;
    if (now >= g_fullcal_until_ms) can_feed_watchdog();   // paused during FULL_CAL
#if USE_COPROC
    coproc_send_downstream(0, g_estop);   // keep coproc fed: estop + tareSeq
#endif
  }

#if USE_COPROC
  coproc_link_poll();                      // parse coproc RX (comms core)
#endif

  // Host link: maintain WiFi/TCP, dispatch incoming commands.
  host_link_poll();
  CommandPacket cmd;
  while (host_link_get_command(&cmd)) dispatchCommand(cmd);

  // Link-loss watchdog: GUI talked then went silent while running -> safe hold.
  if (g_running && host_link_last_rx_ms() != 0 && host_link_link_lost()) {
    static uint32_t lastWarn = 0;
    if (now - lastWarn > 2000) { lastWarn = now;
      Serial.println("[host] link lost while running -> homing (safe hold)"); }
    g_pendingCmd = CMD_STOP;
  }

  // Telemetry: broadcast LogPacket at HOST_TELEM_HZ.
  static uint32_t lastTelem = 0;
  if (now - lastTelem >= (uint32_t)(1000 / HOST_TELEM_HZ)) {
    lastTelem = now;
    LogPacket lp; build_logpacket(&lp);
    host_link_send_telemetry((const uint8_t*)&lp, sizeof(lp));

    // Teach/Observe: also stream the captured ROM so the GUI can apply it.
    if (g_setupMode == SETUP_TEACH || g_setupMode == SETUP_OBSERVE) {
      CaptureRangePacket cap; cap.start0 = 0xBB; cap.start1 = 0x77;
      cap.mode = g_setupMode; cap.validMask = 0;
      for (int i = 0; i < PG_NJOINTS; i++) {
        cap.minDeg[i] = g_modeState.cap_min_deg[i];
        cap.maxDeg[i] = g_modeState.cap_max_deg[i];
        if (g_modeState.cap_valid[i]) cap.validMask |= (1 << i);
      }
      cap.crc = pg_crc16_ccitt((const uint8_t*)&cap, sizeof(cap) - 2);
      host_link_send_telemetry((const uint8_t*)&cap, sizeof(cap));
    }
  }

  if (now - lastStatus >= 500) {
    lastStatus = now;
    BusTelemetry snap;
    can_snapshot(&snap);
    // Peak control-loop time since last status (and reset). A jump here = the
    // RT loop stalled (e.g. blocked CAN TX) -> motor jitter.
    uint16_t lpmax = g_ctrlLoopUsMax; g_ctrlLoopUsMax = 0;
    // Torque mode runs off its own phase (g_torque), not the position-mode state
    // machine — report that so the status line isn't stuck at IDLE.
    bool tq = (g_controlMode == MODE_TORQUE);
    unsigned phv = tq ? (g_running ? (unsigned)PH_GAIT : (unsigned)PH_IDLE)
                      : (unsigned)g_engine.phase;
    float p01v = tq ? g_torque.phase01 : g_engine.phase01;
    if (!g_quiet) {
      Serial.printf("[status] t=%lu ticks=%lu loop=%u(max=%u)us heap=%lu "
                    "estop=%d bus=%d armed=%d run=%d ph=%u p01=%.2f adapt=%.2f\n",
                    (unsigned long)now, (unsigned long)g_ctrlTicks,
                    (unsigned)g_ctrlLoopUs, (unsigned)lpmax,
                    (unsigned long)ESP.getFreeHeap(), (int)g_estop, (int)snap.bus_up,
                    (int)g_armed, (int)g_running, phv, p01v, g_torque.adapt);
      // TWAI controller health: tx_failed / bus_err / tx_queue climbing over
      // time => CAN bus issue (termination / ACK / load), not the control loop.
      can_dump_status("bus");
#if USE_COPROC
      CoprocData cd; coproc_get(&cd);
      Serial.printf("   coproc online=%d age=%lu ms\n", (int)cd.online,
                    (unsigned long)cd.ageMs);
#endif
      for (int i = 0; i < PG_NJOINTS; i++) {
        const AxisTelemetry& a = snap.j[i];
#if USE_COPROC
        Serial.printf("   %s n%u st=%u pos=%.2f vel=%.2f iq=%.2f tau=%.2f fb=%d\n",
                      JOINTS[i].shortName, JOINTS[i].node_id, a.axis_state,
                      a.pos_turns, a.vel_turns_s, a.iq_measured_a, cd.torqueNm[i],
                      (int)a.fb_valid);
#else
        Serial.printf("   %s n%u st=%u pos=%.2f vel=%.2f iq=%.2f fb=%d\n",
                      JOINTS[i].shortName, JOINTS[i].node_id, a.axis_state,
                      a.pos_turns, a.vel_turns_s, a.iq_measured_a, (int)a.fb_valid);
#endif
      }
    }
    // On-device status text (no-op unless USE_STATUS_SCREEN).
    char sbuf[160];
    snprintf(sbuf, sizeof(sbuf),
             "P.GEAR v8\nWiFi: %s   E-STOP: %s\nstate: %s   mode: %s\nbus: %s",
             host_link_wifi_up() ? "up" : "--", g_estop ? "ACTIVE" : "clear",
             g_setupMode != SETUP_NONE ? "SETUP" : (g_running ? "RUN" : (g_armed ? "ARM" : "idle")),
             g_controlMode == MODE_TORQUE ? "TORQUE" : "POS",
             snap.bus_up ? "ok" : "--");
    status_screen_update(sbuf);
  }
}
