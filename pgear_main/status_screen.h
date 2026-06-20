// status_screen.h — minimal on-device TEXT status panel. NO LVGL.  [Phase 0 / 8]
// Reuse only the low-level panel bring-up from v7.1.5 (esp_panel_board_custom_conf.h
// / lvgl_v8_port init), then draw plain text at ~5-10 Hz on the comms core:
//   STATE / E-STOP / PC-link / coproc rate / per-joint torque.
// Optional — the device runs fine headless (GUI is on the PC).
#pragma once
// TODO P0: panel init + draw_text(). Verify ESP32-S3-Touch-LCD-7 GPIO map first.
