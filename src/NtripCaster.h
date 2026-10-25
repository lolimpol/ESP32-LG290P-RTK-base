#pragma once
// Local NTRIP caster (single mountpoint). One task multiplexes all rovers with NON-BLOCKING sockets;
// each rover only owns a read cursor into the shared RtcmBus plus one small staging buffer,
// so RAM per rover is ~2 KB regardless of the RTCM rate. A stalled rover is dropped after the
// configured timeout and can never delay the others.
#include "RtcmBus.h"
#include "Config.h"
#include <WiFi.h>

class NtripCaster {
public:
  NtripCaster() : server_(0, 1) {}
  void begin(RtcmBus& bus);
private:
  struct Slot {
    WiFiClient cl; bool used, streaming;
    uint32_t acceptMs, progressMs; uint16_t hdrLen, txLen, txPos;
    char hdr[512]; uint8_t tx[1536];
    RtcmBus::Cursor cur;
  };
  static void thunk(void* p) { ((NtripCaster*)p)->run(); }
  void run();
  void acceptNew(const AppConfig& c, uint8_t active);
  void serviceSlot(Slot& s, const AppConfig& c);
  void handleRequest(Slot& s, const AppConfig& c);
  void drop(Slot& s, const char* why);
  void sendSourcetable(Slot& s, const AppConfig& c, bool rev2);
  RtcmBus* bus_ = nullptr;
  WiFiServer server_;
  Slot slot_[CASTER_MAX_CLIENTS];
};
