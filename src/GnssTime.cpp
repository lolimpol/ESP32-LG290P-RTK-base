#include "GnssTime.h"
#include <sys/time.h>
#include <esp_timer.h>

void IRAM_ATTR GnssTime::ppsIsr(void* p) {
  GnssTime* t = (GnssTime*)p;
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&t->mux_);
  t->ppsUs_ = now; t->ppsCount_++;
  portEXIT_CRITICAL_ISR(&t->mux_);
}

void GnssTime::begin(int ppsPin) {
  ppsPin_ = ppsPin;
  if (ppsPin >= 0) { pinMode(ppsPin, INPUT); attachInterruptArg(ppsPin, ppsIsr, this, RISING); }
}

// Days-from-civil (Howard Hinnant), valid for the proleptic Gregorian calendar.
uint32_t GnssTime::civilToUnix(int y, int m, int d, int hh, int mm, int ss) {
  y -= m <= 2;
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int64_t days = (int64_t)era * 146097 + (int)doe - 719468;
  return (uint32_t)(days * 86400 + hh * 3600 + mm * 60 + ss);
}

// Assumptions (documented in README): the RMC for epoch T is transmitted AFTER the 1PPS edge of T,
// within <900 ms. Without PPS, the epoch is taken as (arrival time - ntpLatencyMs).
void GnssTime::onRmc(uint32_t unixSec, uint16_t fracMs, int64_t rx) {
  int64_t label = (int64_t)unixSec * 1000000LL + (int64_t)fracMs * 1000LL;
  int64_t pps; portENTER_CRITICAL(&mux_); pps = ppsUs_; portEXIT_CRITICAL(&mux_);
  Mode m; int64_t monoAtLabel;
  if (ppsPin_ >= 0 && pps > 0 && rx - pps >= 0 && rx - pps < 900000) { monoAtLabel = pps; m = M_PPS; }
  else { monoAtLabel = rx - latencyUs_; m = M_NMEA; }
  int64_t newOff = label - monoAtLabel;

  portENTER_CRITICAL(&mux_);
  int64_t err = newOff - offsetUs_;
  if (!have_ || err > 200000 || err < -200000) offsetUs_ = newOff;     // step
  else offsetUs_ += err / (m == M_PPS ? 4 : 8);                         // slew (EMA)
  have_ = true; mode_ = m; lastMs_ = millis(); lastUnixUs_ = label;
  int64_t off = offsetUs_;
  portEXIT_CRITICAL(&mux_);

  struct timeval tv; gettimeofday(&tv, nullptr);
  int64_t sys = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
  int64_t gnss = esp_timer_get_time() + off;
  if (!sysSet_ || llabs(sys - gnss) > 2000000LL) {
    tv.tv_sec = (time_t)(gnss / 1000000LL); tv.tv_usec = (suseconds_t)(gnss % 1000000LL);
    settimeofday(&tv, nullptr); sysSet_ = true;
  }
}

bool GnssTime::snapshot(Fix& f) {
  portENTER_CRITICAL(&mux_);
  f.offsetUs = offsetUs_; f.mode = mode_; f.lastUnixUs = lastUnixUs_;
  uint32_t age = millis() - lastMs_;
  bool h = have_;
  portEXIT_CRITICAL(&mux_);
  f.ageMs = age;
  f.valid = h && age <= HOLDOVER_MS;
  return f.valid;
}
