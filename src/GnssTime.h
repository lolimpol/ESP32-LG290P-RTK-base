#pragma once
// GNSS-derived time base. The ESP32 system clock is NOT used by the NTP server; instead we keep
//   unix_us(now) = esp_timer_us(now) + offset
// where `offset` is re-estimated from every valid RMC (and from the optional 1PPS edge, which
// removes the NMEA transmission latency). The libc clock is set once (and only re-stepped on >2 s error)
// purely for log/UI timestamps.
#include <stdint.h>
#include <Arduino.h>

class GnssTime {
public:
  enum Mode : uint8_t { M_NONE = 0, M_NMEA = 1, M_PPS = 2 };
  struct Fix { bool valid; Mode mode; int64_t offsetUs; uint32_t ageMs; int64_t lastUnixUs; };

  void begin(int ppsPin);
  void setLatencyUs(int32_t us) { latencyUs_ = us; }
  void onRmc(uint32_t unixSec, uint16_t fracMs, int64_t rxMonoUs);   // GNSS task
  bool snapshot(Fix& f);                                              // any task
  uint32_t ppsCount() const { return ppsCount_; }
  static uint32_t civilToUnix(int y, int mo, int d, int h, int mi, int s);

  static const uint32_t HOLDOVER_MS = 120000;     // beyond this the time is declared invalid

private:
  static void IRAM_ATTR ppsIsr(void* self);
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile int64_t ppsUs_ = 0;
  volatile uint32_t ppsCount_ = 0;
  int64_t offsetUs_ = 0, lastUnixUs_ = 0;
  uint32_t lastMs_ = 0;
  Mode mode_ = M_NONE;
  bool have_ = false, sysSet_ = false;
  volatile int32_t latencyUs_ = 0;
  int ppsPin_ = -1;
};
