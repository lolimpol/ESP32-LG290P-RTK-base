#pragma once
#include "GnssTime.h"
// SNTP-style server (RFC 5905 packet format, mode 4 replies to mode 3 requests only).
// Timestamps come from GnssTime (esp_timer + GNSS offset), not from the libc clock.
// Realistic accuracy: ~10-50 ms without 1PPS (NMEA latency + Wi-Fi jitter), ~1-5 ms with 1PPS.
class NtpServer {
public:
  void begin(GnssTime& t);
private:
  static void thunk(void* p) { ((NtpServer*)p)->run(); }
  void run();
  GnssTime* time_ = nullptr;
};
