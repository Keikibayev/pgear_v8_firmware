// ============================================================================
// board_io.cpp — CH422G expander: select CAN vs native-USB on GPIO19/20.
// ============================================================================
#include "board_io.h"
#include <Arduino.h>
#include <ESP_IOExpander_Library.h>

// CH422G shares the touch/expander I2C bus (matches v7.1.5 esp_panel conf).
#define IO_I2C_SCL    9
#define IO_I2C_SDA    8
#define EXIO_USB_SEL  5   // CH422G EXIO5: HIGH = CAN mode, LOW = USB mode

static ESP_IOExpander* s_exp = nullptr;

bool board_io_init() {
  s_exp = new ESP_IOExpander_CH422G(
      (i2c_port_t)I2C_NUM_0, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS,
      IO_I2C_SCL, IO_I2C_SDA);
  if (s_exp == nullptr) {
    Serial.println("[board_io] CH422G alloc failed");
    return false;
  }
  s_exp->init();
  s_exp->begin();
  s_exp->enableAllIO_Output();   // EXIO0..7 as outputs (Waveshare pattern)
  Serial.println("[board_io] CH422G up");
  return true;
}

void board_io_set_can_mode(bool can) {
  if (s_exp == nullptr) return;
  // EXIO5 HIGH routes GPIO19/20 to the CAN transceiver (away from native USB).
  s_exp->digitalWrite(EXIO_USB_SEL, can ? HIGH : LOW);
  Serial.printf("[board_io] USB_SEL(EXIO5)=%s -> %s mode\n",
                can ? "HIGH" : "LOW", can ? "CAN" : "USB");
}
