// ============================================================================
// status_screen.cpp — minimal LVGL single-label status panel.  [Phase 8]
// Only compiled when USE_STATUS_SCREEN=1. Reuses the v7.1.5 panel bring-up.
// ============================================================================
#include "status_screen.h"

#if USE_STATUS_SCREEN
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"      // copied from v7.1.5

using namespace esp_panel::drivers;
using namespace esp_panel::board;

static lv_obj_t* s_label = nullptr;

void status_screen_init() {
  Board* board = new Board();
  board->init();
  board->begin();
  lvgl_port_init(board->getLCD(), board->getTouch());

  lvgl_port_lock(-1);
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
  s_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_label, &lv_font_montserrat_24, 0);
  lv_label_set_text(s_label, "P.GEAR v8\nbooting...");
  lv_obj_align(s_label, LV_ALIGN_TOP_LEFT, 16, 16);
  lvgl_port_unlock();
}

void status_screen_update(const char* text) {
  if (!s_label || !text) return;
  lvgl_port_lock(-1);
  lv_label_set_text(s_label, text);
  lvgl_port_unlock();
}

#else  // headless: no-ops
void status_screen_init() {}
void status_screen_update(const char*) {}
#endif
