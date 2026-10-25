// ESP32 RTK base-station controller for the Quectel LG290P. See README.md for the architecture.
#include <Arduino.h>
#include "UserConfig.h"
#include "Config.h"
#include "ConfigStorage.h"
#include "SystemStatus.h"
#include "LogManager.h"
#include "WatchdogManager.h"
#include "NetworkManager.h"
#include "RtcmBus.h"
#include "GnssTime.h"
#include "GnssUart.h"
#include "NtripClient.h"
#include "NtripCaster.h"
#include "NtpServer.h"
#include "WebApi.h"
#include "WebUi.h"
#include "GnssCmd.h"

static RtcmBus      g_bus;          // ~16 KB static, allocated once
static GnssTime     g_time;
static GnssUart     g_gnss(g_bus, g_time);
static NtripClient  g_ntrip[2];
static NtripCaster  g_caster;
static NtpServer    g_ntp;

void setup() {
  setCpuFrequencyMhz(CPU_FREQ_MHZ);          // before any baud-rate-dependent peripheral starts
  Log::begin();
  Watchdog::begin();
  Config::begin();

  AppConfig c; char err[96] = "";
  ConfigStorage::LoadResult lr = ConfigStorage::load(c);
  if (!Config::commit(c, err, sizeof(err))) { Config::setDefaults(c); Config::commit(c, err, sizeof(err)); }
  Log::applyConfig(c);
  LOGI("main", "%s %s | reset: %s | config: %s | CPU %lu MHz", FW_NAME, FW_VERSION, Watchdog::resetReason(),
       ConfigStorage::resultStr(lr), (unsigned long)getCpuFrequencyMhz());

  if (!g_bus.begin()) { LOGE("main", "RTCM bus init failed"); delay(1000); ESP.restart(); }
  GnssCmd::begin();
  g_time.begin(GNSS_PPS_PIN);
  WebApi::setBus(&g_bus);

  Net::begin();
  g_gnss.begin();                            // core 1, highest app priority
  g_ntrip[0].begin(0, g_bus);
  g_ntrip[1].begin(1, g_bus);
  g_caster.begin(g_bus);
  g_ntp.begin(g_time);
  WebUi::begin();
  Watchdog::registerTask(WD_MAIN, "main", 5000);
}

// Housekeeping only (core 1, lowest priority): never touches the data path.
void loop() {
  static uint32_t lastSec = 0, gen = 0;
  uint32_t now = millis();

  if (gen != Config::generation()) {         // live-apply settings owned by this task
    gen = Config::generation();
    AppConfig c; Config::snapshot(c); Log::applyConfig(c);
  }

  if (now - lastSec >= 1000) {
    lastSec = now;
    Status::Data& g = Status::g;
    for (uint8_t i = 0; i < Rtcm::kCatalogCount; i++) {          // per-second rates
      Status::RtcmStat& s = g.rtcm[i];
      uint32_t total = s.rx + s.gen;
      s.rate = total - s.pn; s.pn = total;
      s.bps = s.bytes - s.pb; s.pb = s.bytes;
    }
    g.uartBps = g.uartBytes - g.uartPb; g.uartPb = g.uartBytes;
    GnssTime::Fix f; g_time.snapshot(f);
    g.time.valid = f.valid; g.time.mode = f.valid ? f.mode : 0; g.time.ageMs = f.ageMs; g.time.ppsCount = g_time.ppsCount();
    g.net.connected = Net::connected();
    g.logDropped = Log::streamDropped();
    Watchdog::supervise();
  }
  delay(100);
}
