#include "Config.h"
#include "Rtcm.h"
#include "Rtcm1005.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static AppConfig         s_cfg;
static SemaphoreHandle_t s_mtx = nullptr;
static volatile uint32_t s_gen = 1;

namespace Config {

void setDefaults(AppConfig& c) {
  memset(&c, 0, sizeof(c));
  strlcpy(c.wifiSsid, DEF_WIFI_SSID, sizeof(c.wifiSsid));
  strlcpy(c.wifiPass, DEF_WIFI_PASS, sizeof(c.wifiPass));
  strlcpy(c.hostname, DEF_HOSTNAME, sizeof(c.hostname));
  strlcpy(c.webUser, DEF_WEB_USER, sizeof(c.webUser));
  strlcpy(c.webPass, DEF_WEB_PASS, sizeof(c.webPass));
  c.logLevel = DEF_LOG_LEVEL;
  c.usbMode = 0;
  c.gnssBaud = DEF_GNSS_BAUD; c.gnssRx = DEF_GNSS_RX_PIN; c.gnssTx = DEF_GNSS_TX_PIN;
  c.x = DEF_BASE_X; c.y = DEF_BASE_Y; c.z = DEF_BASE_Z;
  c.stationId = DEF_STATION_ID; c.itrfYear = DEF_ITRF_YEAR;
  c.gpsInd = c.gloInd = c.galInd = 1;
  c.gen1005 = DEF_1005_ENABLE; c.gen1005IntervalS = DEF_1005_INTERVAL_S;
  // Everything in the catalogue is forwarded *if the receiver sends it*; harmless otherwise.
  c.rtcmMask = (Rtcm::kCatalogCount >= 32) ? 0xFFFFFFFFu : ((1u << Rtcm::kCatalogCount) - 1u);

  NtripClientCfg& a = c.ntrip[0]; NtripClientCfg& b = c.ntrip[1];
  a.enabled = DEF_N1_ENABLED; a.version = 2; a.port = DEF_N1_PORT; a.reconnectS = DEF_N_RETRY_S;
  strlcpy(a.host, DEF_N1_HOST, sizeof(a.host)); strlcpy(a.mount, DEF_N1_MOUNT, sizeof(a.mount));
  strlcpy(a.user, DEF_N1_USER, sizeof(a.user)); strlcpy(a.pass, DEF_N1_PASS, sizeof(a.pass));
  b.enabled = DEF_N2_ENABLED; b.version = 2; b.port = DEF_N2_PORT; b.reconnectS = DEF_N_RETRY_S;
  strlcpy(b.host, DEF_N2_HOST, sizeof(b.host)); strlcpy(b.mount, DEF_N2_MOUNT, sizeof(b.mount));
  strlcpy(b.user, DEF_N2_USER, sizeof(b.user)); strlcpy(b.pass, DEF_N2_PASS, sizeof(b.pass));

  c.casterEnabled = DEF_CASTER_ENABLED; c.casterPort = DEF_CASTER_PORT;
  c.casterMaxClients = DEF_CASTER_MAXCLI; c.casterTimeoutS = 15;
  strlcpy(c.casterMount, DEF_CASTER_MOUNT, sizeof(c.casterMount));
  strlcpy(c.casterUser, DEF_CASTER_USER, sizeof(c.casterUser));
  strlcpy(c.casterPass, DEF_CASTER_PASS, sizeof(c.casterPass));
  c.ntpEnabled = DEF_NTP_ENABLED; c.ntpPort = DEF_NTP_PORT; c.ntpLatencyMs = DEF_NTP_LATENCY_MS;
  c.usbBaud = DEF_USB_BAUD; c.logMask = DEF_LOG_MASK;
  updateDerived(c);
}

void updateDerived(AppConfig& c) {
  c.ecef[0] = Rtcm1005::metersToUnits(c.x);
  c.ecef[1] = Rtcm1005::metersToUnits(c.y);
  c.ecef[2] = Rtcm1005::metersToUnits(c.z);
}

static bool fail(char* e, size_t n, const char* m) { if (e && n) strlcpy(e, m, n); return false; }

// printable ASCII, length in [minLen, cap-1]
static bool txt(const char* s, size_t cap, size_t minLen) {
  size_t n = strnlen(s, cap);
  if (n >= cap || n < minLen) return false;
  for (size_t i = 0; i < n; i++) if ((uint8_t)s[i] < 0x20 || (uint8_t)s[i] > 0x7E) return false;
  return true;
}
static bool tokenOk(const char* s, bool allowDot) {      // mountpoints / hostnames
  if (!*s) return false;
  for (; *s; s++) {
    char c = *s;
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '-' || c == '_' || (allowDot && c == '.'))) return false;
  }
  return true;
}
static bool baudOk(uint32_t b) { return b >= 4800 && b <= 3000000; }
static bool pinOk(int p, bool out) {
  if (p < 0 || p > 39) return false;
  if (p >= 6 && p <= 11) return false;      // SPI flash
  if (p == 1 || p == 3) return false;       // UART0 console
  if (out && p >= 34) return false;         // input-only pins
  return true;
}

bool validate(AppConfig& c, char* e, size_t en) {
  if (!txt(c.wifiSsid, sizeof(c.wifiSsid), 1)) return fail(e, en, "Wi-Fi SSID must be 1-32 printable characters");
  size_t pl = strnlen(c.wifiPass, sizeof(c.wifiPass));
  if (!txt(c.wifiPass, sizeof(c.wifiPass), 0) || (pl != 0 && pl < 8)) return fail(e, en, "Wi-Fi password must be empty or 8-64 characters");
  if (!txt(c.hostname, sizeof(c.hostname), 1) || !tokenOk(c.hostname, false)) return fail(e, en, "Hostname: letters, digits, '-' only");
  if (!txt(c.webUser, sizeof(c.webUser), 1) || !txt(c.webPass, sizeof(c.webPass), 4)) return fail(e, en, "Web user 1-23 chars, web password 4-31 chars");
  if (c.logLevel > 3) return fail(e, en, "Log level 0-3");
  if (c.usbMode > 1) return fail(e, en, "USB mode 0 or 1");
  if (!baudOk(c.gnssBaud) || !baudOk(c.usbBaud)) return fail(e, en, "Baud rate must be 4800-3000000");
  if (!pinOk(c.gnssRx, false)) return fail(e, en, "GNSS RX pin invalid (0-39, not flash/UART0)");
  if (c.gnssTx != -1 && !pinOk(c.gnssTx, true)) return fail(e, en, "GNSS TX pin invalid (-1 or output-capable GPIO)");
  if (c.gnssTx == c.gnssRx) return fail(e, en, "RX and TX pins must differ");
  {   // sanity: a point on/near the Earth's surface. NaN fails every comparison, so it is rejected too.
    double r = sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
    if (!(r >= 6.2e6 && r <= 6.5e6)) return fail(e, en, "ECEF X/Y/Z must be metres from the Earth's centre (radius ~6 357 000 - 6 379 000 m)");
  }
  if (c.stationId > 4095) return fail(e, en, "Station ID 0-4095");
  if (c.itrfYear > 63) return fail(e, en, "ITRF year code 0-63");
  c.gpsInd = !!c.gpsInd; c.gloInd = !!c.gloInd; c.galInd = !!c.galInd; c.gen1005 = !!c.gen1005;
  if (c.gen1005IntervalS < 1 || c.gen1005IntervalS > 600) return fail(e, en, "1005 interval 1-600 s");
  for (int i = 0; i < 2; i++) {
    NtripClientCfg& n = c.ntrip[i];
    n.enabled = !!n.enabled; n.tls = !!n.tls;
    if (n.version != 1 && n.version != 2) return fail(e, en, "NTRIP version must be 1 or 2");
    if (!txt(n.host, sizeof(n.host), 0) || !txt(n.mount, sizeof(n.mount), 0) ||
        !txt(n.user, sizeof(n.user), 0) || !txt(n.pass, sizeof(n.pass), 0)) return fail(e, en, "NTRIP text field too long or invalid");
    if (n.reconnectS < 2 || n.reconnectS > 3600) return fail(e, en, "NTRIP reconnect interval 2-3600 s");
    if (n.enabled) {
      if (!tokenOk(n.host, true)) return fail(e, en, "NTRIP host invalid");
      if (!tokenOk(n.mount, true)) return fail(e, en, "NTRIP mountpoint: letters, digits, - _ . only");
      if (n.port == 0) return fail(e, en, "NTRIP port invalid");
      if (strchr(n.user, ':')) return fail(e, en, "NTRIP user must not contain ':'");
    }
  }
  c.casterEnabled = !!c.casterEnabled;
  if (c.casterMaxClients < 1 || c.casterMaxClients > CASTER_MAX_CLIENTS) return fail(e, en, "Caster max clients out of range (see CASTER_MAX_CLIENTS)");
  if (c.casterTimeoutS < 5 || c.casterTimeoutS > 300) return fail(e, en, "Caster client timeout 5-300 s");
  if (c.casterPort == 0 || c.casterPort == 80) return fail(e, en, "Caster port invalid (0 and 80 reserved)");
  if (!txt(c.casterMount, sizeof(c.casterMount), 1) || !tokenOk(c.casterMount, true)) return fail(e, en, "Caster mountpoint: letters, digits, - _ . only");
  if (!txt(c.casterUser, sizeof(c.casterUser), 0) || !txt(c.casterPass, sizeof(c.casterPass), 0) || strchr(c.casterUser, ':'))
    return fail(e, en, "Caster credentials invalid (no ':' in user)");
  c.ntpEnabled = !!c.ntpEnabled;
  if (c.ntpPort == 0 || c.ntpPort == 80 || c.ntpPort == c.casterPort) return fail(e, en, "NTP port invalid or conflicts");
  if (c.ntpLatencyMs < -500 || c.ntpLatencyMs > 1500) return fail(e, en, "NTP latency compensation -500..1500 ms");
  c.logMask &= 0x00FF;
  return true;
}

void begin() {
  s_mtx = xSemaphoreCreateMutex();
  setDefaults(s_cfg);
}

void snapshot(AppConfig& out) {
  xSemaphoreTake(s_mtx, portMAX_DELAY);
  memcpy(&out, &s_cfg, sizeof(AppConfig));
  xSemaphoreGive(s_mtx);
}

bool commit(const AppConfig& cand, char* err, size_t en) {
  AppConfig t = cand;
  if (!validate(t, err, en)) return false;
  updateDerived(t);
  xSemaphoreTake(s_mtx, portMAX_DELAY);
  memcpy(&s_cfg, &t, sizeof(AppConfig));
  s_gen++;
  xSemaphoreGive(s_mtx);
  return true;
}

uint32_t generation() { return s_gen; }

bool restartRequired(const AppConfig& a, const AppConfig& b) {
  return a.gnssRx != b.gnssRx || a.gnssTx != b.gnssTx;
}

void getWebCreds(char* u, size_t un, char* p, size_t pn) {
  xSemaphoreTake(s_mtx, portMAX_DELAY);
  strlcpy(u, s_cfg.webUser, un); strlcpy(p, s_cfg.webPass, pn);
  xSemaphoreGive(s_mtx);
}

uint8_t warnings() {
  // Compare with the LITERAL factory placeholders, not the DEF_* macros: users edit those macros
  // in UserConfig.h, and a check against them would never clear.
  AppConfig c; snapshot(c); uint8_t w = 0;
  if (!strcmp(c.webPass, "admin")) w |= 1;
  if (fabs(c.x - 4092296.4623) < 1.0 && fabs(c.y - 358029.5482) < 1.0 && fabs(c.z - 4862865.6422) < 1.0) w |= 2;
  if (!strcmp(c.wifiSsid, "CHANGE_ME_SSID")) w |= 4;
  return w;
}

}  // namespace Config
