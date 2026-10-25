#include "GnssUart.h"
#include "SystemStatus.h"
#include "WatchdogManager.h"
#include "LogManager.h"
#include <Arduino.h>

static const uint32_t SILENT_MS = 5000;

void GnssUart::openPort(const AppConfig& c) {
  GNSS_SERIAL.end();
  GNSS_SERIAL.setRxBufferSize(GNSS_UART_RX_BUF);     // must precede begin()
  GNSS_SERIAL.begin(c.gnssBaud, SERIAL_8N1, c.gnssRx, c.gnssTx);
  LOGI("gnss", "UART open: %lu baud, RX=%d TX=%d, rxbuf=%d", (unsigned long)c.gnssBaud, c.gnssRx, c.gnssTx, GNSS_UART_RX_BUF);
}

void GnssUart::begin() { xTaskCreatePinnedToCore(thunk, "gnss", 5120, this, 6, nullptr, 1); }

void GnssUart::run() {
  Watchdog::registerTask(WD_GNSS, "gnss", 5000);
  AppConfig cfg; Config::snapshot(cfg);
  uint32_t gen = Config::generation();
  openPort(cfg);
  router_.applyConfig(cfg);

  uint8_t buf[512];
  uint32_t lastRx = millis(), lastRecovery = lastRx, lastStat = 0;
  // An unfinished frame may legitimately take (max frame bytes * 10 bit / baud) to complete.
  uint32_t partialMs = 50 + 2 * (uint32_t)((uint64_t)GnssParser::FRAME_MAX * 10000ull / cfg.gnssBaud);

  for (;;) {
    Watchdog::kick(WD_GNSS);
    uint32_t now = millis();

    if (gen != Config::generation()) {
      AppConfig nc; Config::snapshot(nc); gen = Config::generation();
      if (nc.gnssBaud != cfg.gnssBaud) {
        GNSS_SERIAL.updateBaudRate(nc.gnssBaud);
        partialMs = 50 + 2 * (uint32_t)((uint64_t)GnssParser::FRAME_MAX * 10000ull / nc.gnssBaud);
      }
      cfg = nc;
      router_.applyConfig(cfg);          // pin changes need a reboot (flagged by the web layer)
    }
    if (Watchdog::consumeRecover(WD_GNSS)) { openPort(cfg); Status::g.uartRecoveries++; lastRx = lastRecovery = now; }

    int avail = GNSS_SERIAL.available();
    if (avail > 0) {
      if ((uint32_t)avail > Status::g.backlogMax) Status::g.backlogMax = (uint32_t)avail;
      int n = (int)GNSS_SERIAL.readBytes((char*)buf, avail < (int)sizeof(buf) ? avail : (int)sizeof(buf));
      if (n > 0) {
        lastRx = now; Status::g.uartBytes += (uint32_t)n; Status::g.uartOk = true;
        parser_.feed(buf, (size_t)n);
      }
      router_.tick(now);
      if (avail > n) continue;           // more queued: keep draining without sleeping
    } else {
      if (parser_.midFrame() && now - lastRx > partialMs) parser_.abortPartial();   // stalled mid-frame
      Status::g.silentMs = now - lastRx;
      if (now - lastRx > SILENT_MS) {
        Status::g.uartOk = false;
        if (now - lastRecovery > SILENT_MS) {                  // reopen the port every 5 s while silent
          openPort(cfg); Status::g.uartRecoveries++; lastRecovery = now;
        }
      }
      router_.tick(now);                 // 1005 keeps flowing even if the receiver is silent
      vTaskDelay(1);
    }
    if (now - lastStat > 250) { lastStat = now; Status::g.parser = parser_.stats(); }
  }
}
