#pragma once
// Outbound NTRIP service. NOTE ON TERMINOLOGY: a base station that pushes corrections to a remote
// caster takes the NTRIP *Server* (source) role in the protocol. This module is the "NTRIP client
// service" of the firmware (it dials out), but speaks SOURCE (Rev1) / POST (Rev2). One task per
// instance: a hung DNS/TLS/TCP operation in one can never delay the other or the GNSS path.
#include "RtcmBus.h"
#include "Config.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>

class NtripClient {
public:
  void begin(uint8_t idx, RtcmBus& bus);
private:
  static void thunk(void* p) { ((NtripClient*)p)->run(); }
  void run();
  bool open(const NtripClientCfg& c, char* err, size_t en);
  bool resolve(const char* host, IPAddress& ip, char* err, size_t en);
  void closeSock();
  void setErr(const char* e);

  uint8_t idx_ = 0;
  RtcmBus* bus_ = nullptr;
  WiFiClient plain_;
  WiFiClientSecure secure_;
  WiFiClient* c_ = nullptr;
  RtcmBus::Cursor cur_;
  uint8_t tx_[1536];
  IPAddress lastIp_;          // last successfully resolved address (used if DNS later fails)
  char lastHost_[64] = "";
};
