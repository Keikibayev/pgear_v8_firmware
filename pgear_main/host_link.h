// host_link.h — PC supervisor link.  [Phase 3 / 4]
//   - USB-CDC: parse CommandPacket (0xCC33) frames -> dispatch to control/state;
//     emit LogPacket telemetry (~100 Hz).
//   - WiFi-UDP: broadcast the SAME LogPacket bytes to :47000 (telemetry-only)
//     so the existing pgear_tools pgear_udp_logger.py works unchanged.
#pragma once
#include "protocol.h"
// TODO P4: host_link_init(); poll_commands(); build_logpacket(); emit_usb/emit_udp.
