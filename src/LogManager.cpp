#include "LogManager.h"
#include "ConfigStorage.h"
#include "GnssCmd.h"
#include "SystemStatus.h"
#include "WatchdogManager.h"
#include "NetworkManager.h"
#include "Rtcm.h"
#include <Arduino.h>
#include <WiFi.h>
#include <stdarg.h>
#include "freertos/stream_buffer.h"

namespace Log {

const char* const kStreamNames[8] = { "GGA", "SGA", "GST", "GSV", "RMC", "GSA", "RXM-RAWX", "RXM-SFRBX" };

static StreamBufferHandle_t s_sb = nullptr;
static SemaphoreHandle_t    s_mtx = nullptr;
static volatile uint8_t     s_level = LL_INFO;
static volatile bool        s_stream = false;
static volatile uint32_t    s_baud = 115200;
static volatile bool        s_baudDirty = false;
static volatile uint32_t    s_dropped = 0;

bool streamMode() { return s_stream; }
uint32_t streamDropped() { return s_dropped; }

static bool push(const uint8_t* d, uint16_t n) {
  if (!s_sb || !xSemaphoreTake(s_mtx, 0)) { s_dropped++; return false; }
  bool ok = xStreamBufferSpacesAvailable(s_sb) >= n;
  if (ok) xStreamBufferSend(s_sb, d, n, 0); else s_dropped++;
  xSemaphoreGive(s_mtx);
  return ok;
}
bool streamPush(const uint8_t* d, uint16_t n) { return s_stream ? push(d, n) : false; }

void print(LogLevel lv, const char* tag, const char* fmt, ...) {
  if (s_stream || lv > s_level) return;
  char b[200];
  uint32_t t = millis();
  int n = snprintf(b, sizeof(b), "[%lu.%03lu] %c %s: ", (unsigned long)(t / 1000), (unsigned long)(t % 1000), "EWID"[lv], tag);
  va_list ap; va_start(ap, fmt);
  int m = vsnprintf(b + n, sizeof(b) - n - 2, fmt, ap);
  va_end(ap);
  if (m < 0) return;
  n += m; if (n > (int)sizeof(b) - 3) n = sizeof(b) - 3;
  b[n++] = '\r'; b[n++] = '\n';
  push((const uint8_t*)b, (uint16_t)n);
}

// CLI output: always emitted (the user typed a command).
static void out(const char* fmt, ...) {
  char b[200]; va_list ap; va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b) - 2, fmt, ap); va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(b) - 3) n = sizeof(b) - 3;
  b[n++] = '\r'; b[n++] = '\n';
  push((const uint8_t*)b, (uint16_t)n);
}

void applyConfig(const AppConfig& c) {
  s_level = c.logLevel;
  uint32_t want = c.usbMode ? c.usbBaud : 115200;
  if (want != s_baud) { s_baud = want; s_baudDirty = true; }
  s_stream = c.usbMode == 1;
}

static void cmdStatus() {
  const Status::Data& g = Status::g;
  out("%s %s  reset: %s  boots %lu wdt %lu panic %lu", FW_NAME, FW_VERSION, Watchdog::resetReason(),
      (unsigned long)Watchdog::rtc().boots, (unsigned long)Watchdog::rtc().wdtResets, (unsigned long)Watchdog::rtc().panicResets);
  out("uptime %lus heap %lu min %lu cpu %luMHz", (unsigned long)(millis() / 1000), (unsigned long)ESP.getFreeHeap(),
      (unsigned long)ESP.getMinFreeHeap(), (unsigned long)getCpuFrequencyMhz());
  out("wifi %s ip %s rssi %d", g.net.connected ? "up" : "DOWN", g.net.ip, (int)g.net.rssi);
  out("gnss uart %s bytes %lu rtcmOk %lu crcErr %lu nmeaOk %lu resync %lu recov %lu", g.uartOk ? "ok" : "SILENT",
      (unsigned long)g.uartBytes, (unsigned long)g.parser.rtcmOk, (unsigned long)g.parser.rtcmCrcErr,
      (unsigned long)g.parser.nmeaOk, (unsigned long)g.parser.resyncs, (unsigned long)g.uartRecoveries);
  for (uint8_t i = 0; i < Rtcm::kCatalogCount; i++)
    if (g.rtcm[i].rx || g.rtcm[i].gen) out("  rtcm %u rx %lu gen %lu fwd %lu", Rtcm::kCatalog[i].num,
        (unsigned long)g.rtcm[i].rx, (unsigned long)g.rtcm[i].gen, (unsigned long)g.rtcm[i].fwd);
  static const char* const st[] = { "disabled", "wait-net", "backoff", "connecting", "connected" };
  for (int i = 0; i < 2; i++)
    out("ntrip%d %s tx %lu reconn %lu err '%s'", i + 1, st[g.ntrip[i].state], (unsigned long)g.ntrip[i].tx,
        (unsigned long)g.ntrip[i].reconnects, g.ntrip[i].err);
  out("caster clients %u tx %lu | ntp req %lu | time %s", g.caster.clients, (unsigned long)g.caster.tx,
      (unsigned long)g.ntp.requests, g.time.valid ? "valid" : "invalid");
  for (uint8_t i = 0; i < WD_COUNT; i++) {
    Watchdog::TaskInfo t; Watchdog::taskInfo(i, t);
    if (t.registered) out("  task %-8s core %u stack-free %lu B", t.name, t.core, (unsigned long)t.stackFreeBytes);
  }
}

static void runCommand(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  AppConfig c; Config::snapshot(c); char err[96] = "";
  if (!strcmp(cmd, "help")) {
    out("commands: status | gnss <PQTM cmd> | dns <host> | level <e|w|i|d> | wifi <ssid> [pass] | stream <on|off> | save | factory | reboot");
  } else if (!strcmp(cmd, "status")) {
    cmdStatus();
  } else if (!strcmp(cmd, "level")) {
    char* a = strtok(nullptr, " ");
    if (!a) { out("level=%u", (unsigned)c.logLevel); return; }
    const char* L = "ewid"; const char* p = strchr(L, a[0]);
    if (!p) { out("use e|w|i|d"); return; }
    c.logLevel = (uint8_t)(p - L);
    if (Config::commit(c, err, sizeof(err))) out("level set (use 'save' to persist)"); else out("%s", err);
  } else if (!strcmp(cmd, "wifi")) {
    char* s = strtok(nullptr, " "); char* p = strtok(nullptr, " ");
    if (!s) { out("usage: wifi <ssid> [password]"); return; }
    strlcpy(c.wifiSsid, s, sizeof(c.wifiSsid)); strlcpy(c.wifiPass, p ? p : "", sizeof(c.wifiPass));
    if (Config::commit(c, err, sizeof(err))) out("Wi-Fi applied (use 'save' to persist)"); else out("%s", err);
  } else if (!strcmp(cmd, "stream")) {
    char* a = strtok(nullptr, " ");
    if (!a) { out("usage: stream on|off"); return; }
    c.usbMode = !strcmp(a, "on");
    if (Config::commit(c, err, sizeof(err))) out("stream mode %s (use 'save' to persist)", c.usbMode ? "ON" : "off"); else out("%s", err);
  } else if (!strcmp(cmd, "gnss")) {
    char* rest = strtok(nullptr, "");
    if (!rest) { out("usage: gnss PQTMCFGMSGRATE,W,RMC,1   (checksum added automatically)"); return; }
    char sent[112];
    if (GnssCmd::send(rest, sent, sizeof(sent), err, sizeof(err))) out("sent %s (reply appears as 'receiver: ...')", sent); else out("%s", err);
  } else if (!strcmp(cmd, "dns")) {
    char* h = strtok(nullptr, " ");
    out("resolvers: %s, %s", WiFi.dnsIP(0).toString().c_str(), WiFi.dnsIP(1).toString().c_str());
    if (!h) { out("usage: dns <hostname>"); return; }
    IPAddress ip; uint32_t t0 = millis();
    bool ok = WiFi.hostByName(h, ip);
    out("%s -> %s (%lu ms)", h, ok ? ip.toString().c_str() : "FAILED", (unsigned long)(millis() - t0));
  } else if (!strcmp(cmd, "save")) {
    out(ConfigStorage::save(c) ? "saved" : "SAVE FAILED");
  } else if (!strcmp(cmd, "factory")) {
    out(ConfigStorage::factoryReset() ? "factory reset done, rebooting" : "reset failed");
    vTaskDelay(pdMS_TO_TICKS(300)); Watchdog::restartDevice(RR_USER);
  } else if (!strcmp(cmd, "reboot")) {
    Watchdog::restartDevice(RR_USER);
  } else out("unknown command, try 'help'");
}

static void task(void*) {
  Watchdog::registerTask(WD_CONSOLE, "console", 5000);
  uint8_t tmp[256]; char line[96]; size_t ll = 0;
  for (;;) {
    Watchdog::kick(WD_CONSOLE);
    if (s_baudDirty) { s_baudDirty = false; Serial.flush(); Serial.updateBaudRate(s_baud); }
    size_t n = xStreamBufferReceive(s_sb, tmp, sizeof(tmp), pdMS_TO_TICKS(20));
    if (n) Serial.write(tmp, n);
    while (Serial.available()) {
      int ch = Serial.read();
      if (ch == '\r' || ch == '\n') { if (ll) { line[ll] = 0; ll = 0; runCommand(line); } }
      else if (ll < sizeof(line) - 1 && ch >= 0x20 && ch < 0x7F) line[ll++] = (char)ch;
    }
  }
}

void begin() {
  Serial.begin(115200);
  s_sb = xStreamBufferCreate(6144, 1);
  s_mtx = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(task, "console", 4608, nullptr, 2, nullptr, 1);
}

}  // namespace Log
