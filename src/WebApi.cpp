#include "WebApi.h"
#include "SystemStatus.h"
#include "WatchdogManager.h"
#include "GnssCmd.h"
#include "LogManager.h"
#include "NetworkManager.h"
#include "Rtcm.h"
#include <Arduino.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>

static RtcmBus* s_bus = nullptr;
void WebApi::setBus(RtcmBus* b) { s_bus = b; }

// ---------------------------------------------------------------- streaming JSON writer
class JsonOut {
public:
  explicit JsonOut(httpd_req_t* r) : r_(r) {}
  void obj()  { pre(); put('{'); push(); }
  void end()  { pop(); put('}'); }
  void arr()  { pre(); put('['); push(); }
  void endA() { pop(); put(']'); }
  void key(const char* k) { if (cm_[d_]) put(','); cm_[d_] = true; q(k); put(':'); ak_ = true; }
  void u(uint32_t v)  { char t[12]; snprintf(t, sizeof t, "%lu", (unsigned long)v); pre(); raw(t); }
  void i(int32_t v)   { char t[12]; snprintf(t, sizeof t, "%ld", (long)v); pre(); raw(t); }
  void f(double v, int p) { char t[32]; snprintf(t, sizeof t, "%.*f", p, v); pre(); raw(t); }
  void b(bool v)      { pre(); raw(v ? "true" : "false"); }
  void s(const char* v) { pre(); q(v); }
  void kU(const char* k, uint32_t v) { key(k); u(v); }
  void kI(const char* k, int32_t v)  { key(k); i(v); }
  void kF(const char* k, double v, int p) { key(k); f(v, p); }
  void kB(const char* k, bool v)     { key(k); b(v); }
  void kS(const char* k, const char* v) { key(k); s(v); }
  void finish() { flush(); if (!err_) httpd_resp_send_chunk(r_, nullptr, 0); }
  bool ok() const { return !err_; }
private:
  void pre() { if (ak_) ak_ = false; else { if (cm_[d_]) put(','); cm_[d_] = true; } }
  void push() { if (d_ < 7) d_++; cm_[d_] = false; }
  void pop()  { if (d_) d_--; }
  void put(char c) { if (n_ >= sizeof(buf_)) flush(); buf_[n_++] = c; }
  void raw(const char* s) { while (*s) put(*s++); }
  void q(const char* s) {
    put('"');
    for (; *s; s++) {
      unsigned char c = (unsigned char)*s;
      if (c == '"' || c == '\\') { put('\\'); put((char)c); }
      else if (c < 0x20) { put(' '); }
      else put((char)c);
    }
    put('"');
  }
  void flush() { if (n_ && !err_ && httpd_resp_send_chunk(r_, buf_, n_) != ESP_OK) err_ = true; n_ = 0; }
  httpd_req_t* r_; char buf_[1536]; size_t n_ = 0; uint8_t d_ = 0; bool cm_[8] = {false}; bool ak_ = false; bool err_ = false;
};

static int32_t ageOf(uint32_t last) { return last ? (int32_t)(millis() - last) : -1; }

// ---------------------------------------------------------------- status
esp_err_t WebApi::status(httpd_req_t* r) {
  httpd_resp_set_type(r, "application/json");
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
  httpd_resp_set_hdr(r, "Connection", "close");
  JsonOut j(r);
  const Status::Data& g = Status::g;
  AppConfig c; Config::snapshot(c);
  uint32_t now = millis();

  j.obj();
  j.key("sys"); j.obj();
    j.kS("fw", FW_VERSION); j.kU("uptime", (uint32_t)(esp_timer_get_time() / 1000000LL));
    j.kU("heap", ESP.getFreeHeap()); j.kU("heapMin", ESP.getMinFreeHeap());
    j.kU("heapBlock", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    j.kU("cpuMHz", getCpuFrequencyMhz()); j.kS("reset", Watchdog::resetReason());
    const Watchdog::RtcInfo& rc = Watchdog::rtc();
    j.kU("boots", rc.boots); j.kU("wdtResets", rc.wdtResets); j.kU("panicResets", rc.panicResets);
    j.kU("brownouts", rc.brownouts); j.kU("heapResets", rc.heapResets); j.kU("wifiResets", rc.wifiResets);
    j.kU("softRecoveries", Watchdog::softRecoveries());
    j.key("tasks"); j.arr();
    for (uint8_t i = 0; i < WD_COUNT; i++) {
      Watchdog::TaskInfo t; Watchdog::taskInfo(i, t); if (!t.registered) continue;
      j.obj(); j.kS("name", t.name); j.kU("core", t.core); j.kU("stackFree", t.stackFreeBytes); j.kU("heartbeatMs", t.kickAgeMs); j.end();
    }
    j.endA();
  j.end();
  j.key("net"); j.obj();
    j.kB("connected", Net::connected()); j.kS("ssid", g.net.ssid); j.kS("ip", g.net.ip); j.kI("rssi", g.net.rssi);
    j.kU("disconnects", g.net.disconnects); j.kU("attempts", g.net.attempts); j.kU("hardRecoveries", g.net.hardRecoveries);
  j.end();

  j.key("gnss"); j.obj();
    j.kS("uart", g.uartOk ? "receiving" : "SILENT"); j.kU("baud", c.gnssBaud); j.kU("rxBuffer", GNSS_UART_RX_BUF);
    j.kU("bytes", g.uartBytes); j.kU("bytesPerSec", g.uartBps); j.kU("backlogMax", g.backlogMax);
    j.kU("silentMs", g.silentMs); j.kU("uartRecoveries", g.uartRecoveries);
    const ParserStats& p = g.parser;
    j.key("parser"); j.obj();
      j.kU("rtcmOk", p.rtcmOk); j.kU("rtcmCrcErr", p.rtcmCrcErr); j.kU("rtcmBadHdr", p.rtcmBadHeader);
      j.kU("nmeaOk", p.nmeaOk); j.kU("nmeaBadSum", p.nmeaBadChecksum); j.kU("nmeaAbort", p.nmeaAbort);
      j.kU("ubxOk", p.ubxOk); j.kU("ubxBad", p.ubxBadChecksum + p.ubxOversize);
      j.kU("garbage", p.garbageBytes); j.kU("resyncs", p.resyncs); j.kU("partialTimeouts", p.partialTimeouts);
    j.end();
    static const char* const nk[NK_COUNT] = { "GGA", "RMC", "GLL", "VTG", "GSA", "GST", "GSV", "SGA", "proprietary", "other" };
    j.key("nmea"); j.arr();
    for (int i = 0; i < NK_COUNT; i++) { j.obj(); j.kS("type", nk[i]); j.kU("count", g.nmea[i].n); j.kI("ageMs", ageOf(g.nmea[i].lastMs)); j.end(); }
    j.endA();
    Status::Gga a; Status::getGga(a);
    j.key("gga"); j.obj();
      j.kB("valid", a.valid); j.kS("utc", a.utc); j.kF("lat", a.lat, 8); j.kF("lon", a.lon, 8);
      j.kF("alt", a.alt, 2); j.kF("sep", a.sep, 2); j.kF("hdop", a.hdop, 1); j.kU("sats", a.sats); j.kU("quality", a.quality);
      j.kI("ageMs", ageOf(a.ms));
    j.end();
    j.kI("txPin", c.gnssTx); j.kU("cmdSent", g.cmdSent); j.kU("cmdReplies", g.cmdReplies);
    { static char rb[6][100]; uint8_t nr = Status::getReplies(rb, 6);
      j.key("replies"); j.arr(); for (uint8_t i = 0; i < nr; i++) j.s(rb[i]); j.endA(); }
  j.end();

  j.key("rtcm"); j.obj();
    j.kU("other", g.rtcmOther); j.kU("suppressed", g.rtcmSuppressed); j.kU("filtered", g.rtcmFiltered);
    if (s_bus) { j.kU("busFrames", s_bus->published); j.kU("busOverwritten", s_bus->overwritten); }
    j.key("messages"); j.arr();
    for (uint8_t i = 0; i < Rtcm::kCatalogCount; i++) {
      const Status::RtcmStat& s = g.rtcm[i];
      bool gen = (i == Rtcm::IDX_1005);
      j.obj(); j.kU("msg", Rtcm::kCatalog[i].num); j.kS("name", Rtcm::kCatalog[i].name);
      j.kS("source", gen ? "ESP32" : "LG290P");
      j.kB("enabled", gen ? c.gen1005 : ((c.rtcmMask >> i) & 1));
      j.kU("rx", s.rx); j.kU("gen", s.gen); j.kU("fwd", s.fwd); j.kU("rate", s.rate); j.kU("bps", s.bps);
      j.kI("ageMs", ageOf(s.lastMs)); j.end();
    }
    j.endA();
  j.end();

  static const char* const nst[] = { "disabled", "waiting for Wi-Fi", "backoff", "connecting", "connected" };
  j.key("ntrip"); j.arr();
  for (int i = 0; i < 2; i++) {
    const Status::Ntrip& n = g.ntrip[i];
    j.obj(); j.kU("id", i + 1); j.kS("state", nst[n.state]); j.kS("target", n.target); j.kU("rx", n.rx); j.kU("tx", n.tx);
    j.kU("reconnects", n.reconnects); j.kU("failures", n.failures);
    j.kU("connectedS", n.state == NS_CONNECTED ? (uint32_t)(now / 1000 - n.connectedSinceS) : 0);
    j.kU("lastOkUnix", n.lastOkUnix); j.kS("lastError", n.err); j.end();
  }
  j.endA();

  j.key("caster"); j.obj();
    j.kB("enabled", c.casterEnabled); j.kB("listening", g.caster.listening); j.kU("port", c.casterPort);
    j.kS("mount", c.casterMount); j.kU("clients", g.caster.clients); j.kU("maxClients", c.casterMaxClients);
    j.kU("tx", g.caster.tx); j.kU("rx", g.caster.rx); j.kU("accepted", g.caster.accepted);
    j.kU("rejected", g.caster.rejected); j.kU("authFail", g.caster.authFail); j.kU("dropped", g.caster.dropped);
  j.end();

  j.key("ntp"); j.obj();
    j.kB("enabled", c.ntpEnabled); j.kB("running", g.ntp.running); j.kU("port", c.ntpPort);
    j.kU("requests", g.ntp.requests); j.kU("replies", g.ntp.replies); j.kU("dropped", g.ntp.dropped);
    j.kB("gnssTimeValid", g.time.valid);
    static const char* const ms[] = { "none", "GNSS RMC (NMEA latency compensated)", "GNSS RMC + 1PPS" };
    j.kS("source", ms[g.time.mode > 2 ? 0 : g.time.mode]); j.kU("ageMs", g.time.ageMs); j.kU("ppsPulses", g.time.ppsCount);
    j.kS("sync", !g.time.valid ? "unsynchronised" : (g.time.ageMs < 5000 ? "locked" : "holdover"));
    j.kU("rmcReceived", g.time.rmcRx); j.kU("rmcNoFix", g.time.rmcNoFix); j.kU("rmcRejected", g.time.rmcBad);
    { char st[2] = { g.time.rmcStatus ? g.time.rmcStatus : '-', 0 }; j.kS("lastRmcStatus", st); }
    j.kS("why", g.time.valid ? "time valid"
         : !g.uartOk ? "no data from the LG290P UART"
         : g.time.rmcRx == 0 ? "no RMC received - enable the RMC sentence on the LG290P"
         : g.time.rmcNoFix == g.time.rmcRx ? "RMC arrives but status is V (no valid position fix yet)"
         : g.time.rmcBad ? "RMC received but date/time fields rejected"
         : "waiting for the first valid RMC");
  j.end();

  j.key("log"); j.obj();
    j.kS("mode", c.usbMode ? "GNSS stream on USB/serial" : "diagnostic console"); j.kU("baud", c.usbMode ? c.usbBaud : 115200);
    j.kU("dropped", Log::streamDropped());
    j.key("streams"); j.arr();
    for (int i = 0; i < 8; i++) {
      j.obj(); j.kS("name", Log::kStreamNames[i]); j.kB("selected", (c.logMask >> i) & 1);
      j.kB("receiving", g.logRx[i].lastMs && now - g.logRx[i].lastMs < 5000);
      j.kU("rx", g.logRx[i].n); j.kU("fwd", g.logFwd[i]); j.end();
    }
    j.endA();
  j.end();

  j.key("warn"); j.arr();
  uint8_t w = Config::warnings();
  if (w & 1) j.s("Default web password is still set - change it.");
  if (w & 2) j.s("Base position is still the factory example - enter your surveyed coordinates.");
  if ((w & 4) && !Net::connected()) j.s("Wi-Fi SSID is still the factory placeholder.");
  if (!g.uartOk) j.s("No data from the LG290P UART.");
  j.endA();
  j.end();
  j.finish();
  return j.ok() ? ESP_OK : ESP_FAIL;
}

// ---------------------------------------------------------------- config field table (serialise AND parse)
enum FT : uint8_t { T_STR, T_PASS, T_U8, T_U16, T_U32, T_I8, T_I16, T_DBL };
struct FieldDef { const char* key; FT t; uint16_t off; uint16_t sz; };
#define FLD(k, t, m) { k, t, (uint16_t)offsetof(AppConfig, m), (uint16_t)sizeof(((AppConfig*)0)->m) }
#define NTF(P, I) \
  FLD(P "en", T_U8, ntrip[I].enabled), FLD(P "tls", T_U8, ntrip[I].tls), FLD(P "ver", T_U8, ntrip[I].version), \
  FLD(P "host", T_STR, ntrip[I].host), FLD(P "port", T_U16, ntrip[I].port), FLD(P "mount", T_STR, ntrip[I].mount), \
  FLD(P "user", T_STR, ntrip[I].user), FLD(P "pass", T_PASS, ntrip[I].pass), FLD(P "retry", T_U16, ntrip[I].reconnectS)

static const FieldDef kFields[] = {
  FLD("wifi_ssid", T_STR, wifiSsid), FLD("wifi_pass", T_PASS, wifiPass), FLD("hostname", T_STR, hostname),
  FLD("web_user", T_STR, webUser), FLD("web_pass", T_PASS, webPass),
  FLD("log_level", T_U8, logLevel), FLD("usb_mode", T_U8, usbMode), FLD("usb_baud", T_U32, usbBaud),
  FLD("gnss_baud", T_U32, gnssBaud), FLD("gnss_rx", T_I8, gnssRx), FLD("gnss_tx", T_I8, gnssTx),
  FLD("base_x", T_DBL, x), FLD("base_y", T_DBL, y), FLD("base_z", T_DBL, z),
  FLD("station_id", T_U16, stationId), FLD("itrf_year", T_U8, itrfYear),
  FLD("ind_gps", T_U8, gpsInd), FLD("ind_glo", T_U8, gloInd), FLD("ind_gal", T_U8, galInd),
  FLD("gen1005", T_U8, gen1005), FLD("gen1005_s", T_U16, gen1005IntervalS),
  NTF("n1_", 0), NTF("n2_", 1),
  FLD("caster_en", T_U8, casterEnabled), FLD("caster_port", T_U16, casterPort), FLD("caster_mount", T_STR, casterMount),
  FLD("caster_user", T_STR, casterUser), FLD("caster_pass", T_PASS, casterPass),
  FLD("caster_max", T_U8, casterMaxClients), FLD("caster_timeout", T_U16, casterTimeoutS),
  FLD("ntp_en", T_U8, ntpEnabled), FLD("ntp_port", T_U16, ntpPort), FLD("ntp_lat_ms", T_I16, ntpLatencyMs),
};
static const size_t kNFields = sizeof(kFields) / sizeof(kFields[0]);

esp_err_t WebApi::configGet(httpd_req_t* r) {
  httpd_resp_set_type(r, "application/json");
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
  httpd_resp_set_hdr(r, "Connection", "close");
  AppConfig c; Config::snapshot(c);
  const uint8_t* base = (const uint8_t*)&c;
  JsonOut j(r);
  j.obj();
  for (size_t i = 0; i < kNFields; i++) {
    const FieldDef& d = kFields[i]; const uint8_t* p = base + d.off; j.key(d.key);
    switch (d.t) {
      case T_STR:  j.s((const char*)p); break;
      case T_PASS: j.s((const char*)p); break;                  // passwords are shown in the settings panel (by request)
      case T_U8:   j.u(*p); break;
      case T_U16:  j.u(*(const uint16_t*)p); break;
      case T_U32:  j.u(*(const uint32_t*)p); break;
      case T_I8:   j.i(*(const int8_t*)p); break;
      case T_I16:  j.i(*(const int16_t*)p); break;
      case T_DBL:  j.f(*(const double*)p, 4); break;
    }
  }
  for (uint8_t i = 1; i < Rtcm::kCatalogCount; i++) { char k[12]; snprintf(k, sizeof k, "rt_%u", Rtcm::kCatalog[i].num); j.kU(k, (c.rtcmMask >> i) & 1); }
  for (int i = 0; i < 8; i++) { char k[24]; snprintf(k, sizeof k, "lg_%s", Log::kStreamNames[i]); j.kU(k, (c.logMask >> i) & 1); }
  j.end();
  j.finish();
  return j.ok() ? ESP_OK : ESP_FAIL;
}

// ---------------------------------------------------------------- form parsing
static void urldecode(char* s) {
  char* o = s;
  for (; *s; s++) {
    if (*s == '+') *o++ = ' ';
    else if (*s == '%' && s[1] && s[2]) { char h[3] = { s[1], s[2], 0 }; *o++ = (char)strtol(h, nullptr, 16); s += 2; }
    else *o++ = *s;
  }
  *o = 0;
}

static bool setField(AppConfig& c, const FieldDef& d, const char* v, char* err, size_t en) {
  uint8_t* p = (uint8_t*)&c + d.off;
  if (d.t == T_STR || d.t == T_PASS) {
    if (strlen(v) >= d.sz) { snprintf(err, en, "'%s' is too long (max %u)", d.key, (unsigned)(d.sz - 1)); return false; }
    strlcpy((char*)p, v, d.sz); return true;
  }
  char* e; errno = 0;
  if (d.t == T_DBL) { double x = strtod(v, &e); if (e == v || *e || errno) goto bad; *(double*)p = x; return true; }
  { long long x = strtoll(v, &e, 10); if (e == v || *e || errno) goto bad;
    switch (d.t) {
      case T_U8:  if (x < 0 || x > 255) goto bad; *p = (uint8_t)x; break;
      case T_U16: if (x < 0 || x > 65535) goto bad; *(uint16_t*)p = (uint16_t)x; break;
      case T_U32: if (x < 0 || x > 4294967295LL) goto bad; *(uint32_t*)p = (uint32_t)x; break;
      case T_I8:  if (x < -128 || x > 127) goto bad; *(int8_t*)p = (int8_t)x; break;
      case T_I16: if (x < -32768 || x > 32767) goto bad; *(int16_t*)p = (int16_t)x; break;
      default: goto bad;
    }
    return true; }
bad:
  snprintf(err, en, "invalid value for '%s'", d.key);
  return false;
}

bool WebApi::applyForm(char* body, AppConfig& c, char* err, size_t en) {
  char* save = nullptr;
  for (char* pair = strtok_r(body, "&", &save); pair; pair = strtok_r(nullptr, "&", &save)) {
    char* eq = strchr(pair, '='); if (!eq) continue;
    *eq = 0; char* k = pair; char* v = eq + 1;
    urldecode(k); urldecode(v);
    if (!strncmp(k, "rt_", 3)) {
      int idx = Rtcm::catalogIndex((uint16_t)atoi(k + 3));
      if (idx > 0) { if (atoi(v)) c.rtcmMask |= (1u << idx); else c.rtcmMask &= ~(1u << idx); }
      continue;
    }
    if (!strncmp(k, "lg_", 3)) {
      for (int i = 0; i < 8; i++) if (!strcmp(k + 3, Log::kStreamNames[i])) { if (atoi(v)) c.logMask |= (1u << i); else c.logMask &= ~(1u << i); }
      continue;
    }
    for (size_t i = 0; i < kNFields; i++)
      if (!strcmp(k, kFields[i].key)) { if (!setField(c, kFields[i], v, err, en)) return false; break; }
  }
  return true;
}
