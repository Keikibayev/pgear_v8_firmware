// ============================================================================
// board_io.h — Waveshare ESP32-S3-Touch-LCD-7 CH422G IO-expander control.
// ----------------------------------------------------------------------------
// On this board GPIO19/20 (the ESP32-S3 NATIVE-USB D-/D+) are MULTIPLEXED with
// the CAN transceiver through an FSUSB42UMX analog switch, selected by the
// CH422G expander line EXIO5 (USB_SEL / CAN_SEL):
//     EXIO5 = HIGH -> CAN mode   (GPIO19/20 -> CAN transceiver)
//     EXIO5 = LOW  -> USB mode   (GPIO19/20 -> native USB-C connector)
// So native USB and CAN cannot run at once. We pick CAN; the PC link uses the
// board's SECOND USB-C port (the "USB TO UART" CH343P bridge), independent of
// GPIO19/20. Call board_io_set_can_mode(true) BEFORE can_odrive_init().
//
// Requires the Waveshare ESP_IOExpander library (same one v7.1.5 used).
// ============================================================================
#pragma once

bool board_io_init();                 // bring up CH422G over I2C (SDA8/SCL9)
void board_io_set_can_mode(bool can); // EXIO5: true=CAN(HIGH), false=USB(LOW)
