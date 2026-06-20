// ============================================================================
// coproc_link.h — UART link to the ADS1256 coproc.  [Phase 3]
//   RX: parse SensorPacketV2 (0xAA55) + CalStatePacket (0xAA77), CRC-checked;
//       convert calibrated N -> joint torque via MOMENT_ARM_M; track link age.
//   TX: DownstreamPacket (0xBB66: estop/fault + tareSeq), CalAdjust (0xBB55).
//
// ⚠ This board's only broken-out UART is GPIO43/44 (UART2 header), the SAME
// pins the CH343P console/flash port uses via the external UART1/UART2 switch.
// At RUN time the switch is on UART2 (coproc) and debug goes over WiFi; the
// text console is a DEV-only mode (switch=CH343P, coproc unplugged). Pins are
// overridable below.
// ============================================================================
#pragma once
#include <stdint.h>
#include "constants.h"

#ifndef COPROC_TX_PIN
#define COPROC_TX_PIN 43   // main TX -> coproc RX
#endif
#ifndef COPROC_RX_PIN
#define COPROC_RX_PIN 44   // main RX <- coproc TX
#endif

struct CoprocData {
  float    forceN[PG_NJOINTS]   = {0};
  float    torqueNm[PG_NJOINTS] = {0};   // forceN * MOMENT_ARM_M
  float    cal[PG_NJOINTS]      = {0};   // live coproc scale (for the GUI)
  uint8_t  sensorFlags          = 0;     // last SensorPacketV2.flags
  bool     online               = false; // valid packet within timeout
  uint32_t ageMs                = 9999;
  uint16_t crcFails             = 0;
  uint16_t resyncs              = 0;
};

void coproc_link_init();
void coproc_link_poll();                 // call often from the comms core
void coproc_get(CoprocData* out);        // thread-safe copy

// downstream
void coproc_request_tare();              // bump tareSeq -> coproc re-tares
void coproc_send_downstream(uint8_t fault_flags, bool estop);
void coproc_send_caladjust(uint8_t joint, float ratio);
