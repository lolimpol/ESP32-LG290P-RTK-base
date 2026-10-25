#include "NtripClient.h"
#include "NetworkManager.h"
#include "SystemStatus.h"
#include "WatchdogManager.h"
#include "LogManager.h"
#include "NetUtil.h"
#include <mbedtls/base64.h>
#include <lwip/dns.h>

static const uint32_t CONNECT_TIMEOUT_MS = 8000, REPLY_TIMEOUT_MS = 6000, HEALTHY_MS = 30000, MAX_BACKOFF_MS = 300000;

void NtripClient::begin(uint8_t idx, RtcmBus& bus) {
  idx_ = idx; bus_ = &bus;
  xTaskCreatePinnedToCore(thunk, idx ? "ntrip2" : "ntrip1", 7168, this, 3, nullptr, 0);
}

void NtripClient::setErr(const char* e) {
  char* d = Status::g.ntrip[idx_].err;
  size_t i = 0;
  for (; e[i] && i < sizeof(Status::g.ntrip[0].err) - 1; i++) d[i] = ((uint8_t)e[i] >= 0x20 && (uint8_t)e[i] < 0x7F) ? e[i] : '?';
  d[i] = 0;
}

void NtripClient::closeSock() {
  if (c_) { c_->stop(); }
  c_ = nullptr;
}

// Resolve with three escalating steps so one flaky/filtering resolver cannot keep the upload down:
//  1) the DHCP-supplied resolver, twice (first lookup after Wi-Fi up often times out),
//  2) public resolvers 1.1.1.1 / 8.8.8.8 for this one lookup (DHCP servers are restored afterwards),
//  3) the address we resolved last time for this same host name.
bool NtripClient::resolve(const char* host, IPAddress& ip, char* err, size_t en) {
  const WdId wd = idx_ ? WD_NTRIP2 : WD_NTRIP1;
  auto good = [&]() { return ip != IPAddress((uint32_t)0); };
  auto remember = [&]() { lastIp_ = ip; strlcpy(lastHost_, host, sizeof(lastHost_)); };
  for (int a = 0; a < 2; a++) {
    ip = IPAddress((uint32_t)0);
    if (WiFi.hostByName(host, ip) && good()) { remember(); return true; }
    Watchdog::kick(wd); vTaskDelay(pdMS_TO_TICKS(250));
  }
  ip_addr_t old0 = *dns_getserver(0), old1 = *dns_getserver(1), p0, p1;
  IP_ADDR4(&p0, 1, 1, 1, 1); IP_ADDR4(&p1, 8, 8, 8, 8);
  dns_setserver(0, &p0); dns_setserver(1, &p1);
  ip = IPAddress((uint32_t)0);
  bool ok = WiFi.hostByName(host, ip) && good();
  dns_setserver(0, &old0); dns_setserver(1, &old1);
  Watchdog::kick(wd);
  if (ok) {
    remember();
    LOGW("ntrip", "#%u '%s' resolved only via public DNS - your router/ISP resolver (%s) fails for this name",
         idx_ + 1, host, IPAddress(ip_2_ip4(&old0)->addr).toString().c_str());
    return true;
  }
  if (!strcmp(lastHost_, host) && lastIp_ != IPAddress((uint32_t)0)) {
    ip = lastIp_;
    LOGW("ntrip", "#%u DNS failed for '%s', using last known address %s", idx_ + 1, host, ip.toString().c_str());
    return true;
  }
  snprintf(err, en, "DNS lookup of '%s' failed (resolver %s)", host, IPAddress(ip_2_ip4(&old0)->addr).toString().c_str());
  return false;
}

bool NtripClient::open(const NtripClientCfg& c, char* err, size_t en) {
  const WdId wd = idx_ ? WD_NTRIP2 : WD_NTRIP1;
  IPAddress ip;
  if (!resolve(c.host, ip, err, en)) return false;
  Watchdog::kick(wd);
  if (c.tls) { secure_.setInsecure(); secure_.setHandshakeTimeout(8); c_ = &secure_; }
  else c_ = &plain_;
  // TLS needs the host NAME (SNI); plain TCP uses the address we already resolved.
  bool okc = c.tls ? c_->connect(c.host, c.port, (int32_t)CONNECT_TIMEOUT_MS) : c_->connect(ip, c.port, (int32_t)CONNECT_TIMEOUT_MS);
  if (!okc) { strlcpy(err, c.tls ? "TLS/TCP connect failed" : "TCP connect failed", en); return false; }
  Watchdog::kick(wd);
  c_->setNoDelay(true);
  if (!c.tls) sockKeepAlive(c_->fd());

  char req[480]; int n;
  if (c.version == 1) {
    // NTRIP Rev1 source login. Only the password is used.
    n = snprintf(req, sizeof(req), "SOURCE %s /%s\r\nSource-Agent: NTRIP %s/%s\r\n\r\n", c.pass, c.mount, FW_NAME, FW_VERSION);
  } else {
    // NTRIP Rev2 source login: HTTP/1.1 POST + Basic auth, then the raw RTCM stream follows the 200 reply.
    char cred[72], b64[100]; size_t ol = 0;
    snprintf(cred, sizeof(cred), "%s:%s", c.user, c.pass);
    mbedtls_base64_encode((unsigned char*)b64, sizeof(b64) - 1, &ol, (const unsigned char*)cred, strlen(cred));
    b64[ol] = 0;
    n = snprintf(req, sizeof(req),
      "POST /%s HTTP/1.1\r\nHost: %s:%u\r\nNtrip-Version: Ntrip/2.0\r\nAuthorization: Basic %s\r\n"
      "User-Agent: NTRIP %s/%s\r\nConnection: close\r\n\r\n", c.mount, c.host, (unsigned)c.port, b64, FW_NAME, FW_VERSION);
  }
  if (n <= 0 || n >= (int)sizeof(req) || c_->write((const uint8_t*)req, n) != (size_t)n) { strlcpy(err, "request send failed", en); return false; }

  // Read the status line.
  char line[96]; int ll = 0; bool got = false;
  uint32_t t0 = millis();
  while (millis() - t0 < REPLY_TIMEOUT_MS && !got) {
    Watchdog::kick(wd);
    while (c_->available()) {
      int ch = c_->read(); if (ch < 0) break;
      Status::g.ntrip[idx_].rx++;
      if (ch == '\n') { got = true; break; }
      if (ch != '\r' && ll < (int)sizeof(line) - 1) line[ll++] = (char)ch;
    }
    if (!got) { if (!c_->connected() && !c_->available()) break; vTaskDelay(pdMS_TO_TICKS(10)); }
  }
  line[ll] = 0;
  if (!got) { strlcpy(err, ll ? line : "no reply from caster", en); return false; }
  bool ok = !strncmp(line, "ICY 200", 7) || !strcmp(line, "OK") || (!strncmp(line, "HTTP/", 5) && strstr(line, " 200"));
  if (!ok) { strlcpy(err, line, en); return false; }
  // swallow the remainder of the reply headers (Rev2)
  uint32_t t1 = millis();
  while (millis() - t1 < 300) { while (c_->available()) { c_->read(); Status::g.ntrip[idx_].rx++; } vTaskDelay(pdMS_TO_TICKS(20)); }
  return true;
}

void NtripClient::run() {
  const WdId wd = idx_ ? WD_NTRIP2 : WD_NTRIP1;
  Watchdog::registerTask(wd, idx_ ? "ntrip2" : "ntrip1", 20000);
  bus_->addWaiter(xTaskGetCurrentTaskHandle());
  Status::Ntrip& st = Status::g.ntrip[idx_];
  AppConfig cfg; NtripClientCfg nc; memset(&nc, 0, sizeof(nc));
  uint32_t gen = 0, nextTry = 0, upSince = 0, attempts = 0, streak = 0;
  bool connected = false;
  uint8_t rxd[64];

  for (;;) {
    Watchdog::kick(wd);
    uint32_t now = millis();
    if (gen != Config::generation()) {
      Config::snapshot(cfg); gen = Config::generation();
      bool changed = memcmp(&nc, &cfg.ntrip[idx_], sizeof(nc)) != 0;
      nc = cfg.ntrip[idx_];
      if (changed) {                                    // apply live: drop and reconnect with new settings
        if (connected) { closeSock(); connected = false; }
        nextTry = 0; streak = 0;
        snprintf(st.target, sizeof(st.target), "%s:%u/%s%s", nc.host, (unsigned)nc.port, nc.mount, nc.tls ? " (TLS)" : "");
      }
    }
    if (Watchdog::consumeRecover(wd)) { closeSock(); connected = false; nextTry = now + 2000; }

    if (!nc.enabled) { st.state = NS_DISABLED; if (connected) { closeSock(); connected = false; } vTaskDelay(pdMS_TO_TICKS(500)); continue; }
    if (!Net::connected()) { st.state = NS_WAIT_NET; if (connected) { closeSock(); connected = false; } vTaskDelay(pdMS_TO_TICKS(500)); continue; }

    if (!connected) {
      if ((int32_t)(now - nextTry) < 0) { st.state = NS_BACKOFF; vTaskDelay(pdMS_TO_TICKS(200)); continue; }
      st.state = NS_CONNECTING;
      if (attempts++) st.reconnects++;
      char err[64] = "";
      if (open(nc, err, sizeof(err))) {
        connected = true; upSince = millis(); st.state = NS_CONNECTED; st.err[0] = 0;
        st.connectedSinceS = upSince / 1000; st.lastOkUnix = (uint32_t)time(nullptr);
        bus_->attach(cur_);                              // only fresh data; never replay stale ephemeris
        LOGI("ntrip", "#%u connected to %s", idx_ + 1, st.target);
      } else {
        closeSock(); st.failures++; setErr(err); streak++;
        uint32_t base = (uint32_t)nc.reconnectS * 1000u;
        uint32_t d = base << (streak > 5 ? 5 : streak - 1);
        if (d > MAX_BACKOFF_MS) d = MAX_BACKOFF_MS;
        d += (esp_random() % (d / 5 + 1)) - d / 10;       // +-10 % jitter
        nextTry = millis() + d;
        LOGW("ntrip", "#%u connect failed: %s (retry in %lu s)", idx_ + 1, err, (unsigned long)(d / 1000));
      }
      continue;
    }

    // ---- streaming ----
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));        // woken by RtcmBus::publish
    const char* drop = nullptr;
    uint16_t n = bus_->read(cur_, tx_, sizeof(tx_));
    if (n) {
      size_t w = c_->write(tx_, n);
      if (w != n) drop = "write failed";
      else st.tx += n;
    }
    while (!drop && c_->available()) { int r = c_->read(rxd, sizeof(rxd)); if (r > 0) st.rx += (uint32_t)r; else break; }
    if (!drop && !c_->connected()) drop = "closed by caster";
    if (drop) {
      setErr(drop); st.failures++; closeSock(); connected = false;
      streak = (millis() - upSince > HEALTHY_MS) ? 0 : streak + 1;
      uint32_t d = (uint32_t)nc.reconnectS * 1000u << (streak > 5 ? 5 : (streak ? streak - 1 : 0));
      nextTry = millis() + (d > MAX_BACKOFF_MS ? MAX_BACKOFF_MS : d);
      LOGW("ntrip", "#%u dropped: %s", idx_ + 1, drop);
    }
  }
}
