#include "NetworkManager.h"
#include "Config.h"
#include "SystemStatus.h"
#include "WatchdogManager.h"
#include "LogManager.h"
#include <WiFi.h>

namespace Net {

static const uint32_t HARD_RECOVER_MS = 10UL * 60 * 1000;
static const uint32_t REBOOT_AFTER_MS = 60UL * 60 * 1000;   // set to 0 to never reboot on Wi-Fi loss
static volatile bool s_up = false;

bool connected() { return s_up; }

static void task(void*) {
  Watchdog::registerTask(WD_NET, "net", 20000);
  AppConfig cfg; Config::snapshot(cfg);
  uint32_t gen = Config::generation();
  WiFi.persistent(false);
  WiFi.setHostname(cfg.hostname);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false);                 // modem sleep adds latency jitter; mains-powered appliance

  uint32_t lastAttempt = 0, backoff = 1000, downSince = millis(), lastHard = millis(), lastStat = 0;
  bool started = false;
  for (;;) {
    Watchdog::kick(WD_NET);
    uint32_t now = millis();

    if (gen != Config::generation()) {
      AppConfig nc; Config::snapshot(nc); gen = Config::generation();
      bool changed = strcmp(nc.wifiSsid, cfg.wifiSsid) || strcmp(nc.wifiPass, cfg.wifiPass) || strcmp(nc.hostname, cfg.hostname);
      cfg = nc;
      if (changed && started) {
        LOGI("net", "Wi-Fi settings changed, reconnecting");
        WiFi.disconnect(); WiFi.setHostname(cfg.hostname);
        backoff = 500; lastAttempt = 0; started = false;
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (!s_up) {
        s_up = true; backoff = 1000; downSince = 0;
        strlcpy(Status::g.net.ip, WiFi.localIP().toString().c_str(), sizeof(Status::g.net.ip));
        LOGI("net", "connected, IP %s RSSI %d", Status::g.net.ip, WiFi.RSSI());
      }
      if (now - lastStat > 2000) {
        lastStat = now;
        Status::g.net.rssi = (int8_t)WiFi.RSSI();
        strlcpy(Status::g.net.ssid, cfg.wifiSsid, sizeof(Status::g.net.ssid));
      }
    } else {
      if (s_up) { s_up = false; Status::g.net.disconnects++; downSince = now; LOGW("net", "Wi-Fi lost"); }
      if (!downSince) downSince = now;
      if (!started || now - lastAttempt >= backoff) {
        WiFi.disconnect(false, false);
        WiFi.begin(cfg.wifiSsid, cfg.wifiPass[0] ? cfg.wifiPass : nullptr);
        lastAttempt = now; started = true; Status::g.net.attempts++;
        backoff = backoff < 30000 ? backoff * 2 : 60000;          // 1,2,4,...,30,60 s
        LOGD("net", "connect attempt %lu, next in %lu ms", (unsigned long)Status::g.net.attempts, (unsigned long)backoff);
      }
      if (now - lastHard > HARD_RECOVER_MS && now - downSince > HARD_RECOVER_MS) {
        LOGW("net", "down for >10 min: restarting Wi-Fi radio");
        WiFi.disconnect(true, false); vTaskDelay(pdMS_TO_TICKS(500));
        WiFi.mode(WIFI_STA); WiFi.setHostname(cfg.hostname); WiFi.setSleep(false);
        lastHard = now; Status::g.net.hardRecoveries++; started = false; backoff = 1000;
      }
      if (REBOOT_AFTER_MS && now - downSince > REBOOT_AFTER_MS) Watchdog::restartDevice(RR_WIFI);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void begin() { xTaskCreatePinnedToCore(task, "net", 4096, nullptr, 2, nullptr, 0); }

}  // namespace Net
