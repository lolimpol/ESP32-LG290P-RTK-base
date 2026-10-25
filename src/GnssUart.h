#pragma once
#include "GnssRouter.h"
#include "GnssParser.h"

// Owns the LG290P UART and the GNSS task (core 1, highest app priority). The UART driver's ISR-fed
// ring buffer (GNSS_UART_RX_BUF) is the producer side; this task is the consumer + parser + router.
class GnssUart {
public:
  GnssUart(RtcmBus& bus, GnssTime& t) : router_(bus, t), parser_(router_) {}
  void begin();
private:
  static void thunk(void* p) { ((GnssUart*)p)->run(); }
  void run();
  void openPort(const AppConfig& c);
  GnssRouter router_;
  GnssParser parser_;
};
