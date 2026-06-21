// ============================================================================
// wifi_config.h — WiFi + host-link settings. EDIT for your hotspot.  [Phase 4]
// Station mode: the ESP32-S3 and the PC both join this external hotspot.
// ============================================================================
#pragma once

#define WIFI_SSID      "PGEAR_TUNING"    // (from v7.1.5 LOG_SSID)
#define WIFI_PASSWORD  "pgearpgear"      // (from v7.1.5 LOG_PASSWORD)

// Telemetry: LogPacket broadcast over UDP (loss-tolerant). pgear_udp_logger.py
// listens on this port.
#define HOST_UDP_TELEM_PORT  47000
#define HOST_TELEM_HZ        50          // LogPacket emit rate

// Commands: reliable TCP server the PC GUI connects to (framed CommandPacket).
#define HOST_TCP_CMD_PORT    47001

// Link-loss watchdog: if no command/keepalive within this window while running,
// the controller auto-ramps to a safe hold. PC should send OP_NOP as keepalive.
#define HOST_LINK_TIMEOUT_MS 1000
