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

// ---- Control task configuration --------------------------------------------
static const uint32_t CTRL_HZ        = 250;                 // real-time loop rate
static const uint32_t CTRL_PERIOD_US = 1000000UL / CTRL_HZ; // 4000 us
static const BaseType_t CTRL_CORE    = 1;                   // pin control to core 1
static const BaseType_t COMMS_CORE   = 0;                   // loop() runs on core 0

// ---- Shared state (cross-core; expand into proper structs in later phases) --
volatile uint32_t g_ctrlTicks   = 0;
volatile uint16_t g_ctrlLoopUs  = 0;   // last control-loop exec time (-> LogPacket)
volatile bool     g_estop       = false;

// ============================================================================
// Real-time control task — pinned to core 1, runs at CTRL_HZ with low jitter.
// This is where the gait engine + control law + CAN TX will live (P1, P2, P6).
// ============================================================================
static void controlTask(void *arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / CTRL_HZ);

  for (;;) {
    uint32_t t0 = micros();

    // --- TODO P1: poll CAN/ODrive telemetry snapshot
    // --- TODO P3: fold in coproc force -> torque
    // --- TODO P5: safety (estop / watchdog / ROM enforce / cross-check)
    // --- TODO P6: control.step()  (position-mode AAN  OR  torque-mode impedance)
    // --- TODO P1: push setpoints over CAN
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
  // --- TODO P1: can_odrive_init();     (TWAI 250 kbps; reuse waveshare_twai_port)
  // --- TODO P3: coproc_link_init();    (UART @ PG_COPROC_UART_BAUD)
  // --- TODO P4: host_link_init();      (USB-CDC command parser; WiFi-UDP log)

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr,
                          configMAX_PRIORITIES - 2, nullptr, CTRL_CORE);
  Serial.println("[pgear_v8] control task started");
}

// ============================================================================
// loop() runs on core 0 — comms, telemetry, status. Never blocks the RT loop.
// ============================================================================
void loop() {
  static uint32_t lastStatus = 0;
  uint32_t now = millis();

  // Phase-0 USB-CDC echo so we can confirm the PC link end to end.
  while (Serial.available()) {
    int c = Serial.read();
    Serial.write((uint8_t)c);
  }
  // --- TODO P4: parse CommandPacket frames from Serial here.

  if (now - lastStatus >= 500) {
    lastStatus = now;
    // --- TODO P8: render this to the on-device text status screen too.
    Serial.printf("[status] t=%lu ms  ticks=%lu  loop=%u us  estop=%d\n",
                  (unsigned long)now, (unsigned long)g_ctrlTicks,
                  (unsigned)g_ctrlLoopUs, (int)g_estop);
    // --- TODO P3/P4: build + emit LogPacket over USB-CDC and WiFi-UDP.
  }
}
