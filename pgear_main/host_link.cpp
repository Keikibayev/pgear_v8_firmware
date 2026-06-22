// ============================================================================
// host_link.cpp — WiFi station + UDP telemetry + TCP command server.  [Phase 4]
// ============================================================================
#include "host_link.h"
#include "wifi_config.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

static WiFiUDP     s_udp;
static WiFiServer  s_tcp(HOST_TCP_CMD_PORT);
static WiFiClient  s_client;
static IPAddress   s_bcast;
static uint32_t    s_lastRxMs = 0;

// CommandPacket RX framing (header[5] + payload[len] + crc[2])
static uint8_t s_rx[5 + PG_CMD_MAXPAYLOAD + 2];
static size_t  s_rxLen = 0;

// small FIFO of parsed commands
static CommandPacket s_q[8];
static volatile int  s_qHead = 0, s_qTail = 0;

static void q_push(const CommandPacket& c) {
  int n = (s_qHead + 1) % 8;
  if (n == s_qTail) return;     // full: drop oldest-safe (drop new)
  s_q[s_qHead] = c; s_qHead = n;
}
bool host_link_get_command(CommandPacket* out) {
  if (s_qTail == s_qHead) return false;
  *out = s_q[s_qTail]; s_qTail = (s_qTail + 1) % 8;
  return true;
}

void host_link_init() {
  WiFi.mode(WIFI_STA);
  // Disable WiFi modem power-save. The default WIFI_PS_MIN_MODEM sleeps the
  // radio between DTIM beacons and wakes periodically; those wakeups stall the
  // chip enough to jitter the real-time control loop (visible as motor
  // stutter/pauses that only happen in WiFi mode, never on USB-direct). Costs
  // a little extra power; buys clean motion + low command latency.
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[host] joining '%s' ...\n", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(100);
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    s_bcast = IPAddress(ip[0], ip[1], ip[2], 255);   // subnet broadcast
    s_udp.begin(HOST_UDP_TELEM_PORT);
    s_tcp.begin();
    s_tcp.setNoDelay(true);
    Serial.printf("[host] up ip=%s  udp:%d tcp:%d\n", ip.toString().c_str(),
                  HOST_UDP_TELEM_PORT, HOST_TCP_CMD_PORT);
  } else {
    Serial.println("[host] WiFi connect FAILED (will keep retrying in poll)");
  }
}

static void parse_cmd_byte(uint8_t b) {
  // resync on start pair
  if (s_rxLen < 2) {
    if (s_rxLen == 0 && b != PG_CMD_START0) return;
    if (s_rxLen == 1 && b != PG_CMD_START1) { s_rxLen = 0; if (b == PG_CMD_START0) s_rx[s_rxLen++] = b; return; }
    s_rx[s_rxLen++] = b;
    return;
  }
  s_rx[s_rxLen++] = b;
  if (s_rxLen == 5) {                       // header complete; validate len
    uint8_t len = s_rx[4];
    if (len > PG_CMD_MAXPAYLOAD) { s_rxLen = 0; return; }
  }
  if (s_rxLen >= 5) {
    uint8_t len = s_rx[4];
    size_t total = 5 + len + 2;
    if (s_rxLen >= total) {
      uint16_t crc = (uint16_t)s_rx[5 + len] | ((uint16_t)s_rx[5 + len + 1] << 8);
      if (pg_crc16_ccitt(s_rx, 5 + len) == crc) {
        CommandPacket c{};
        c.start0 = s_rx[0]; c.start1 = s_rx[1];
        c.opcode = s_rx[2]; c.joint = s_rx[3]; c.len = len;
        for (uint8_t i = 0; i < len; i++) c.payload[i] = s_rx[5 + i];
        q_push(c);
        s_lastRxMs = millis();
      }
      s_rxLen = 0;
    }
  }
}

void host_link_poll() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 3000) { lastTry = millis();
      WiFi.setSleep(false); WiFi.begin(WIFI_SSID, WIFI_PASSWORD); }
    return;
  }
  if (!s_client || !s_client.connected()) {
    WiFiClient nc = s_tcp.available();
    if (nc) { s_client = nc; s_client.setNoDelay(true); s_lastRxMs = millis();
              Serial.println("[host] GUI connected"); }
  }
  while (s_client && s_client.available()) parse_cmd_byte((uint8_t)s_client.read());
}

void host_link_send_telemetry(const uint8_t* data, size_t len) {
  if (WiFi.status() != WL_CONNECTED) return;
  s_udp.beginPacket(s_bcast, HOST_UDP_TELEM_PORT);
  s_udp.write(data, len);
  s_udp.endPacket();
}

bool     host_link_wifi_up()      { return WiFi.status() == WL_CONNECTED; }
uint32_t host_link_last_rx_ms()   { return s_lastRxMs; }
bool     host_link_link_lost()    { return (millis() - s_lastRxMs) > HOST_LINK_TIMEOUT_MS; }
