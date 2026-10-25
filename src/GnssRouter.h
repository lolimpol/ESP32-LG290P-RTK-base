#pragma once
// Receives verified frames from GnssParser and routes them. Runs in the GNSS task: it never blocks
// (bus publish = short mutex + memcpy; log push = non-blocking StreamBuffer).
#include "GnssParser.h"
#include "RtcmBus.h"
#include "GnssTime.h"
#include "Config.h"

class GnssRouter : public GnssSink {
public:
  GnssRouter(RtcmBus& bus, GnssTime& t) : bus_(bus), time_(t) {}
  void applyConfig(const AppConfig& c);       // GNSS task context only
  void tick(uint32_t nowMs);                  // periodic ESP32-generated RTCM 1005
  void onRtcm(const uint8_t* f, uint16_t len, uint16_t msg) override;
  void onNmea(const char* line, uint16_t len) override;
  void onUbx(const uint8_t* f, uint16_t len, uint8_t cls, uint8_t id) override;
private:
  void handleGga(char* f[], int n, const char* raw, uint16_t len);
  void handleRmc(char* f[], int n);
  RtcmBus& bus_; GnssTime& time_;
  uint32_t rtcmMask_ = 0; uint16_t logMask_ = 0;
  bool gen_ = false, genValid_ = false, genNow_ = true;
  uint32_t genMs_ = 5000, lastGen_ = 0;
  uint8_t f1005_[25];
};
