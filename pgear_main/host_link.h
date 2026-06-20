// ============================================================================
// host_link.h — PC supervisor link over WiFi (station mode).  [Phase 4]
//   - joins the external hotspot (wifi_config.h)
//   - UDP: broadcasts LogPacket telemetry to :HOST_UDP_TELEM_PORT (loss-tolerant)
//   - TCP: server on :HOST_TCP_CMD_PORT, framed CommandPacket from the GUI
//   - link-loss watchdog: host_link_link_lost() true if no RX within timeout
// Transport only — pgear_main builds the LogPacket and dispatches commands.
// ============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "protocol.h"

void     host_link_init();
void     host_link_poll();                       // maintain WiFi/TCP, parse frames
bool     host_link_get_command(CommandPacket* out); // pop next parsed command (FIFO)
void     host_link_send_telemetry(const uint8_t* data, size_t len); // UDP broadcast
bool     host_link_wifi_up();
uint32_t host_link_last_rx_ms();
bool     host_link_link_lost();                  // true once RX stale > timeout
