#include "NtpServer.h"
#include "Config.h"
#include "NetworkManager.h"
#include "SystemStatus.h"
#include "WatchdogManager.h"
#include "LogManager.h"
#include "NetUtil.h"
#include <esp_timer.h>
#include <Arduino.h>

static const uint32_t NTP_UNIX_DELTA = 2208988800UL;

static void putTs(uint8_t* o, int64_t unixUs) {
  uint32_t sec = (uint32_t)(unixUs / 1000000LL) + NTP_UNIX_DELTA;          // era 0 (until 2036)
  uint32_t frac = (uint32_t)(((uint64_t)(unixUs % 1000000LL) << 32) / 1000000ULL);
  o[0] = sec >> 24; o[1] = sec >> 16; o[2] = sec >> 8; o[3] = sec;
  o[4] = frac >> 24; o[5] = frac >> 16; o[6] = frac >> 8; o[7] = frac;
}

void NtpServer::begin(GnssTime& t) { time_ = &t; xTaskCreatePinnedToCore(thunk, "ntp", 4096, this, 4, nullptr, 0); }

void NtpServer::run() {
  Watchdog::registerTask(WD_NTP, "ntp", 5000);
  AppConfig cfg; uint32_t gen = 0; int fd = -1; uint16_t bound = 0;
  uint32_t tokens = 50, lastRefill = millis();
  Status::Ntp& st = Status::g.ntp;

  for (;;) {
    Watchdog::kick(WD_NTP);
    if (gen != Config::generation()) {
      Config::snapshot(cfg); gen = Config::generation();
      time_->setLatencyUs((int32_t)cfg.ntpLatencyMs * 1000);
      st.enabled = cfg.ntpEnabled; st.port = cfg.ntpPort;
    }
    bool want = cfg.ntpEnabled && Net::connected();
    if (fd >= 0 && (!want || bound != cfg.ntpPort || Watchdog::consumeRecover(WD_NTP))) { close(fd); fd = -1; st.running = 0; }
    if (!want) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
    if (fd < 0) {
      fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      sockaddr_in a; memset(&a, 0, sizeof(a)); a.sin_family = AF_INET; a.sin_port = htons(cfg.ntpPort); a.sin_addr.s_addr = htonl(INADDR_ANY);
      if (fd < 0 || bind(fd, (sockaddr*)&a, sizeof(a)) < 0) {
        LOGE("ntp", "socket/bind failed on port %u", (unsigned)cfg.ntpPort);
        if (fd >= 0) close(fd);
        fd = -1; vTaskDelay(pdMS_TO_TICKS(2000)); continue;
      }
      bound = cfg.ntpPort; st.running = 1;
      LOGI("ntp", "listening on UDP %u", (unsigned)bound);
    }

    fd_set rs; FD_ZERO(&rs); FD_SET(fd, &rs);
    timeval tv = { 0, 500000 };
    if (select(fd + 1, &rs, nullptr, nullptr, &tv) <= 0) continue;   // blocks in the kernel: exact arrival timing
    uint8_t pkt[96]; sockaddr_in from; socklen_t fl = sizeof(from);
    int n = recvfrom(fd, pkt, sizeof(pkt), 0, (sockaddr*)&from, &fl);
    int64_t rxMono = esp_timer_get_time();
    if (n < 48) continue;
    st.requests++;
    uint32_t now = millis();
    if (now - lastRefill >= 1000) { tokens = 50; lastRefill = now; }          // global limit 50 req/s
    if (!tokens) { st.dropped++; continue; }
    tokens--;
    if ((pkt[0] & 7) != 3) { st.dropped++; continue; }                          // answer client requests only

    GnssTime::Fix fx; bool ok = time_->snapshot(fx);
    uint8_t rep[48]; memset(rep, 0, sizeof(rep));
    uint8_t vn = (pkt[0] >> 3) & 7; if (vn < 3 || vn > 4) vn = 4;
    uint32_t dispFix;                                                           // root dispersion, 16.16 s
    if (ok) {
      dispFix = (uint32_t)(((fx.mode == GnssTime::M_PPS ? 0.005 : 0.050) + (fx.ageMs / 1000.0) * 50e-6) * 65536.0);   // base + 50 ppm drift
      rep[0] = (0 << 6) | (vn << 3) | 4; rep[1] = 1;                            // stratum 1, GNSS
      rep[12] = 'G'; rep[13] = 'P'; rep[14] = 'S';
    } else {
      dispFix = 65536; rep[0] = (3 << 6) | (vn << 3) | 4; rep[1] = 16;         // LI=alarm, unsynchronised
    }
    rep[2] = pkt[2]; rep[3] = (uint8_t)-16;                                     // poll echo, precision 2^-16 s
    rep[8] = dispFix >> 24; rep[9] = dispFix >> 16; rep[10] = dispFix >> 8; rep[11] = dispFix;
    if (ok) {
      putTs(rep + 16, fx.lastUnixUs);                                           // reference timestamp
      putTs(rep + 32, rxMono + fx.offsetUs);                                    // receive timestamp
    }
    memcpy(rep + 24, pkt + 40, 8);                                              // originate = client's transmit
    if (ok) putTs(rep + 40, esp_timer_get_time() + fx.offsetUs);                // transmit timestamp
    if (sendto(fd, rep, sizeof(rep), 0, (sockaddr*)&from, fl) == (int)sizeof(rep)) st.replies++;
  }
}
