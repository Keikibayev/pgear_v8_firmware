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
volatile bool     g_estop       = false;
volatile bool     g_armed       = false;
volatile bool     g_running     = false;

// Pending operator command (set from comms core, consumed by control core so
// all CAN TX stays on one core). TEMP bench control — replaced by CommandPacket
// in Phase 4.
enum PgCmd { CMD_NONE = 0, CMD_ARM, CMD_RUN, CMD_STOP, CMD_DISARM, CMD_ESTOP };
volatile PgCmd g_pendingCmd = CMD_NONE;

// Stored supervisor state (consumed by the control law in Phase 6).
volatile uint8_t g_controlMode = MODE_POSITION;
volatile bool    g_aanEnabled  = false;
volatile float   g_assistLevel = 0.5f;
volatile uint16_t g_logSeq     = 0;

static GaitEngine g_engine;

// ---- run-state orchestration (executed on the control core) ----------------
static void armDrives() {
  for (int i = 0; i < PG_NJOINTS; i++) {
    if (!g_engine.joints[i].enabled) continue;
    can_clear_errors(i);
    can_set_control_mode(i, CM_POSITION, IM_POS_FILTER);  // bandwidth provisioned
    can_set_limits(i, ABS_VEL_LIM, 10.0f);                // bench-safe current cap
    can_set_axis_state(i, AXIS_CLOSED_LOOP);
  }
  g_armed = true;
  Serial.println("[ctrl] ARMED (closed-loop, POS_FILTER)");
}

static void disarmDrives() {
  g_running = false;
  g_engine.go_idle();
  can_idle_all();
  g_armed = false;
  Serial.println("[ctrl] DISARMED");
}

static void doEstop() {
  g_running = false;
  g_engine.go_idle();
  can_idle_all();
  g_armed = false;
  g_estop = true;
  Serial.println("[ctrl] *** E-STOP *** (all axes IDLE)");
}

static void handlePendingCmd() {
  PgCmd c = g_pendingCmd;
  if (c == CMD_NONE) return;
  g_pendingCmd = CMD_NONE;
  switch (c) {
    case CMD_ARM:    if (!g_estop) armDrives(); break;
    case CMD_RUN:    if (g_armed) { g_engine.start_gait(); g_running = true;
                                    Serial.println("[ctrl] RUN (gait)"); } break;
    case CMD_STOP:   if (g_running) { g_engine.start_homing();
                                      Serial.println("[ctrl] STOP -> homing"); } break;
    case CMD_DISARM: disarmDrives(); break;
    case CMD_ESTOP:  doEstop(); break;
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
  p->gaitPhase = (uint8_t)g_engine.phase;
  p->stepIdx = (uint8_t)(g_engine.phase01 * 50.0f);
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
    case OP_SET_MODE:     if (c.len >= 1) g_controlMode = c.payload[0]; break;
    case OP_SET_CPS:      g_engine.cps   = payload_f32(c, 0); break;
    case OP_SET_AMP_R:    g_engine.amp_r = payload_f32(c, 0); break;
    case OP_SET_AMP_L:    g_engine.amp_l = payload_f32(c, 0); break;
    case OP_SET_ASSIST:   g_assistLevel  = payload_f32(c, 0); break;
    case OP_SET_AAN:      g_aanEnabled   = (c.len >= 1 && c.payload[0]); break;
    case OP_SET_ROM:      if (c.joint < PG_NJOINTS) {
                            g_engine.joints[c.joint].rom_min_deg = payload_f32(c, 0);
                            g_engine.joints[c.joint].rom_max_deg = payload_f32(c, 4);
                          } break;
    case OP_SET_ENABLE:   if (c.joint < PG_NJOINTS && c.len >= 1)
                            g_engine.joints[c.joint].enabled = c.payload[0]; break;
    case OP_TARE:
#if USE_COPROC
      coproc_request_tare();
#endif
      break;
    case OP_LOAD_COEFFS:  if (c.len >= (int)sizeof(JointCoeffs)) {
                            JointCoeffs jc; memcpy(&jc, c.payload, sizeof(JointCoeffs));
                            calib_load_coeffs(&jc);
                          } break;
    // OP_SET_BW/DIR, JOG_*, TEACH_*, OBSERVE_*, CAL_*, TRIM_*, SWEEP_* : Phase 6/7
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

    // --- TODO P3: fold in coproc force -> torque
    // --- TODO P5: safety (estop / watchdog / ROM enforce / cross-check)
    // --- TODO P6: control.step()  (pos-mode AAN  OR  torque-mode impedance)

    // Gait setpoint output at GAIT_STEP_HZ (every GAIT_SUBDIV ticks).
    if (++subdiv >= GAIT_SUBDIV) {
      subdiv = 0;
      if (g_running) {
        float turns[PG_NJOINTS]; bool has[PG_NJOINTS];
        g_engine.tick(GAIT_DT_S, turns, has);
        if (g_armed) {
          for (int i = 0; i < PG_NJOINTS; i++)
            if (has[i]) can_set_input_pos(i, turns[i]);
        }
        // STOP sequence finished -> settle to IDLE.
        if (g_engine.phase == PH_HOMING &&
            (g_engine.all_home_arrived() || g_engine.homing_timed_out())) {
          g_engine.go_idle();
          g_running = false;
          Serial.println("[ctrl] homing done -> IDLE");
        }
      }
    }
    g_ctrlTicks++;

    g_ctrlLoopUs = (uint16_t)(micros() - t0);
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
#if USE_COPROC
  coproc_link_init();                // [Phase 3] ADS1256 coproc on UART2 (43/44)
#endif
  host_link_init();                  // [Phase 4] WiFi STA: UDP telemetry + TCP cmds

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
  while (Serial.available()) {
    int c = Serial.read();
    switch (c) {
      case 'a': g_pendingCmd = CMD_ARM;    break;
      case 'r': g_pendingCmd = CMD_RUN;    break;
      case 's': g_pendingCmd = CMD_STOP;   break;
      case 'd': g_pendingCmd = CMD_DISARM; break;
      case 'e': g_pendingCmd = CMD_ESTOP;  break;
      case 'E': g_estop = false; Serial.println("[ctrl] estop cleared"); break;
      default: break;
    }
  }

  // ODrive watchdog feed @ ~10 Hz (re-asserts known axis state).
  if (now - lastWdFeed >= (uint32_t)(1000.0f / ODRIVE_WD_FEED_HZ)) {
    lastWdFeed = now;
    can_feed_watchdog();
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
  }

  if (now - lastStatus >= 500) {
    lastStatus = now;
    BusTelemetry snap;
    can_snapshot(&snap);
    // --- TODO P8: render this to the on-device text status screen too.
    Serial.printf("[status] t=%lu ticks=%lu loop=%u us estop=%d bus=%d "
                  "armed=%d run=%d ph=%u p01=%.2f\n",
                  (unsigned long)now, (unsigned long)g_ctrlTicks,
                  (unsigned)g_ctrlLoopUs, (int)g_estop, (int)snap.bus_up,
                  (int)g_armed, (int)g_running, (unsigned)g_engine.phase,
                  g_engine.phase01);
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
    // --- TODO P4: build + emit LogPacket over WiFi-UDP; TCP commands.
  }
}
