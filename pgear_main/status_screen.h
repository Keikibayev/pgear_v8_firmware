// ============================================================================
// status_screen.h — minimal on-device TEXT status panel.  [Phase 8]
// OPTIONAL — the device runs headless (GUI is on the PC). Default OFF.
//
// When USE_STATUS_SCREEN=1 this brings up the 7" RGB panel via the v7.1.5
// ESP_Panel + lvgl_v8_port bring-up and shows ONE LVGL label (a few status
// lines) — NOT the old 316 KB screens.c UI. To build it you must add to the
// sketch: esp_panel_board_custom_conf.h, lvgl_v8_port.{h,cpp} (from v7.1.5) and
// the ESP32_Display_Panel + lvgl libraries. See docs/TODO.md.
// ============================================================================
#pragma once

#ifndef USE_STATUS_SCREEN
#define USE_STATUS_SCREEN 0
#endif

void status_screen_init();
void status_screen_update(const char* text);   // call at ~5 Hz from comms core
