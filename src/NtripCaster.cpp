#include "NtripCaster.h"
#include "NetworkManager.h"
#include "SystemStatus.h"
#include "WatchdogManager.h"
#include "LogManager.h"
#include "NetUtil.h"
#include "Rtcm1005.h"
#include <mbedtls/base64.h>

using Status::g;

void NtripCaster::begin(RtcmBus& bus) {
  bus_ = &bus;
  for (auto& s : slot_) { s.used = false; }
  xTaskCreatePinnedToCore(thunk, "caster", 6144, this, 3, nullptr, 0);
}

static bool ctEq(const char* a, const char* b) {          // constant-time compare
  size_t la = strlen(a), lb = strlen(b); uint8_t d = (uint8_t)(la ^ lb);
  for (size_t i = 0; i < la && i < lb; i++) d |= (uint8_t)(a[i] ^ b[i]);
  return d == 0;
}

// case-insensitive header lookup in a request buffer; returns pointer to value (after ':' and spaces)
static const char* findHeader(const char* req, const char* name) {
  size_t nl = strlen(name);
  const char* p = strstr(req, "\r\n");
  while (p) {
    p += 2;
    if (!strncasecmp(p, name, nl) && p[nl] == ':') { p += nl + 1; while (*p == ' ') p++; return p; }
    p = strstr(p, "\r\n");
  }
  return nullptr;
}
static void sendStr(int fd, const char* s, size_t n) { sockSend(fd, (const uint8_t*)s, n); }  // replies are < socket buffer
#define SEND_LIT(fd, lit) sendStr(fd, lit, sizeof(lit) - 1)

void NtripCaster::drop(Slot& s, const char* why) {
  if (s.streaming) { g.caster.dropped++; LOGI("caster", "client dropped: %s", why); }
  s.cl.stop(); s.used = false; s.streaming = false;
}

void NtripCaster::sendSourcetable(Slot& s, const AppConfig& c, bool rev2) {
  char fmt[128] = ""; size_t o = 0;
  for (uint8_t i = 0; i < Rtcm::kCatalogCount && o < sizeof(fmt) - 8; i++) {
    bool on = (i == Rtcm::IDX_1005) ? c.gen1005 : ((c.rtcmMask >> i) & 1);
    if (on) o += snprintf(fmt + o, sizeof(fmt) - o, "%s%u", o ? "," : "", Rtcm::kCatalog[i].num);
  }
  Rtcm1005::Geodetic gd = Rtcm1005::ecefToGeodetic(c.x, c.y, c.z);   // sourcetable wants lat/lon
  char body[480];
  int bl = snprintf(body, sizeof(body),
    "STR;%s;%s;RTCM 3.2;%s;2;GPS+GLO+GAL+BDS;ESP32;" CASTER_COUNTRY ";%.2f;%.2f;0;0;%s;none;%c;N;0;\r\nENDSOURCETABLE\r\n",
    c.casterMount, c.casterMount, fmt, gd.latDeg, gd.lonDeg, FW_NAME, (c.casterUser[0] || c.casterPass[0]) ? 'B' : 'N');
  char hdr[200];
  int hl = snprintf(hdr, sizeof(hdr), "%s\r\nServer: %s/%s\r\n%sContent-Type: gnss/sourcetable\r\nContent-Length: %d\r\nConnection: close\r\n\r\n",
    rev2 ? "HTTP/1.1 200 OK" : "SOURCETABLE 200 OK", FW_NAME, FW_VERSION, rev2 ? "Ntrip-Version: Ntrip/2.0\r\n" : "", bl);
  sendStr(s.cl.fd(), hdr, hl); sendStr(s.cl.fd(), body, bl);
}

void NtripCaster::handleRequest(Slot& s, const AppConfig& c) {
  int fd = s.cl.fd();
  char method[8] = "", path[48] = "", proto[12] = "";
  if (sscanf(s.hdr, "%7s %47s %11s", method, path, proto) < 2 || strcmp(method, "GET")) {
    SEND_LIT(fd, "HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n"); drop(s, "bad request"); return;
  }
  char* q = strchr(path, '?'); if (q) *q = 0;
  const char* nv = findHeader(s.hdr, "ntrip-version");
  bool rev2 = (nv && strstr(nv, "Ntrip/2")) || !strcmp(proto, "HTTP/1.1");

  if (!strcmp(path, "/") || !path[0]) { sendSourcetable(s, c, rev2); drop(s, "sourcetable served"); return; }
  if (strcmp(path + 1, c.casterMount)) { SEND_LIT(fd, "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n"); drop(s, "unknown mount"); return; }

  if (c.casterUser[0] || c.casterPass[0]) {
    bool ok = false;
    const char* a = findHeader(s.hdr, "authorization");
    if (a && !strncasecmp(a, "Basic ", 6)) {
      char b[96]; const char* e = strstr(a, "\r\n"); size_t l = e ? (size_t)(e - (a + 6)) : 0;
      unsigned char dec[72]; size_t ol = 0;
      if (l && l < sizeof(b)) {
        memcpy(b, a + 6, l); b[l] = 0;
        if (!mbedtls_base64_decode(dec, sizeof(dec) - 1, &ol, (unsigned char*)b, l)) {
          dec[ol] = 0; char* col = strchr((char*)dec, ':');
          if (col) { *col = 0; ok = ctEq((char*)dec, c.casterUser) & ctEq(col + 1, c.casterPass); }
        }
      }
    }
    if (!ok) {
      g.caster.authFail++;
      static const char r[] = "HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Basic realm=\"NTRIP\"\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
      sendStr(fd, r, sizeof(r) - 1); drop(s, "auth failed"); return;
    }
  }
  char ok[200];
  int n = rev2 ? snprintf(ok, sizeof(ok), "HTTP/1.1 200 OK\r\nNtrip-Version: Ntrip/2.0\r\nServer: %s/%s\r\nContent-Type: gnss/data\r\nConnection: close\r\n\r\n", FW_NAME, FW_VERSION)
               : snprintf(ok, sizeof(ok), "ICY 200 OK\r\n");
  sendStr(fd, ok, n);
  s.streaming = true; s.txLen = s.txPos = 0; s.progressMs = millis();
  uint16_t l = 0;
  if (bus_->lastReference(s.tx, l)) s.txLen = l;     // rover gets the base position immediately
  bus_->attach(s.cur);
  g.caster.accepted++;
  LOGI("caster", "rover connected from %s", s.cl.remoteIP().toString().c_str());
}

void NtripCaster::serviceSlot(Slot& s, const AppConfig& c) {
  uint32_t now = millis(); int fd = s.cl.fd();
  uint8_t tmp[64];
  if (!s.streaming) {                                   // collecting the request header
    if (now - s.acceptMs > 5000) { drop(s, "handshake timeout"); return; }
    int r = sockRecv(fd, (uint8_t*)s.hdr + s.hdrLen, sizeof(s.hdr) - 1 - s.hdrLen);
    if (r < 0) { drop(s, "closed"); return; }
    if (r > 0) {
      s.hdrLen += r; s.hdr[s.hdrLen] = 0;
      if (strstr(s.hdr, "\r\n\r\n")) handleRequest(s, c);
      else if (s.hdrLen >= sizeof(s.hdr) - 1) { SEND_LIT(fd, "HTTP/1.1 431 Header Too Large\r\nConnection: close\r\n\r\n"); drop(s, "header too large"); }
    }
    return;
  }
  // rover -> caster: usually NMEA GGA; discard
  int r = sockRecv(fd, tmp, sizeof(tmp));
  if (r < 0) { drop(s, "closed by rover"); return; }
  if (r > 0) g.caster.rx += (uint32_t)r;

  if (s.txPos >= s.txLen) { s.txPos = 0; s.txLen = bus_->read(s.cur, s.tx, sizeof(s.tx)); }
  if (s.txLen > s.txPos) {
    int w = sockSend(fd, s.tx + s.txPos, s.txLen - s.txPos);
    if (w < 0) { drop(s, "send error"); return; }
    if (w > 0) { s.txPos += w; g.caster.tx += (uint32_t)w; s.progressMs = now; }
    else if (now - s.progressMs > (uint32_t)c.casterTimeoutS * 1000u) { drop(s, "stalled (no TCP progress)"); return; }
  } else s.progressMs = now;                            // nothing to send is not a stall
}

void NtripCaster::acceptNew(const AppConfig& c, uint8_t active) {
  for (int guard = 0; guard < 2; guard++) {
    WiFiClient nc = server_.available();
    if (nc.fd() < 0) return;
    Slot* fs = nullptr;
    if (active < c.casterMaxClients) for (auto& s : slot_) if (!s.used) { fs = &s; break; }
    if (!fs) {
      static const char r[] = "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
      sendStr(nc.fd(), r, sizeof(r) - 1); nc.stop(); g.caster.rejected++; continue;
    }
    fs->cl = nc; fs->used = true; fs->streaming = false; fs->hdrLen = 0; fs->txLen = fs->txPos = 0;
    fs->acceptMs = fs->progressMs = millis();
    fs->cl.setNoDelay(true); sockKeepAlive(fs->cl.fd());
    active++;
  }
}

void NtripCaster::run() {
  Watchdog::registerTask(WD_CASTER, "caster", 5000);
  bus_->addWaiter(xTaskGetCurrentTaskHandle());
  AppConfig cfg; uint32_t gen = 0; bool listening = false; uint16_t lport = 0;
  for (;;) {
    Watchdog::kick(WD_CASTER);
    if (gen != Config::generation()) { Config::snapshot(cfg); gen = Config::generation(); g.caster.enabled = cfg.casterEnabled; g.caster.port = cfg.casterPort; }
    bool want = cfg.casterEnabled && Net::connected();
    if (listening && (!want || lport != cfg.casterPort || Watchdog::consumeRecover(WD_CASTER))) {
      for (auto& s : slot_) if (s.used) drop(s, "caster restarting");
      server_.end(); listening = false; g.caster.listening = 0;
    }
    if (!listening && want) {
      server_.begin(cfg.casterPort); server_.setNoDelay(true);
      listening = true; lport = cfg.casterPort; g.caster.listening = 1;
      LOGI("caster", "listening on %u, mount /%s", (unsigned)lport, cfg.casterMount);
    }
    if (!listening) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }

    uint8_t active = 0;
    for (auto& s : slot_) if (s.used) { serviceSlot(s, cfg); if (s.used) active++; }
    acceptNew(cfg, active);
    uint8_t clients = 0; for (auto& s : slot_) if (s.used && s.streaming) clients++;
    g.caster.clients = clients;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
  }
}
