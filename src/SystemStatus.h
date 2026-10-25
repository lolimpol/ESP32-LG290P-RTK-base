#pragma once
// Shared, lock-free status block. Every field has exactly ONE writer task; the web UI and
// console only read it, so the UI can never stall the GNSS path. 32-bit counters wrap at 4.29e9.
#include <stdint.h>
#include "Rtcm.h"
#include "GnssParser.h"

enum NmeaKind : uint8_t { NK_GGA, NK_RMC, NK_GLL, NK_VTG, NK_GSA, NK_GST, NK_GSV, NK_SGA, NK_PROP, NK_OTHER, NK_COUNT };
enum NtripState : uint8_t { NS_DISABLED, NS_WAIT_NET, NS_BACKOFF, NS_CONNECTING, NS_CONNECTED };

namespace Status {

const uint8_t RTCM_SLOTS = 24;   // >= Rtcm::kCatalogCount

struct Stat     { uint32_t n; uint32_t lastMs; };
struct RtcmStat { uint32_t rx, gen, fwd, bytes, lastMs, rate, bps, pn, pb; };

struct Gga {
  bool valid; uint8_t quality, sats;
  char utc[12]; double lat, lon; float alt, sep, hdop;
  uint32_t ms; char raw[100];
};
struct Ntrip {
  uint8_t state; char target[128];
  uint32_t rx, tx, reconnects, failures, connectedSinceS, lastOkUnix; char err[64];
};
struct Caster { uint8_t enabled, listening, clients; uint16_t port; uint32_t tx, rx, accepted, rejected, authFail, dropped; };
struct Ntp    { uint8_t enabled, running; uint16_t port; uint32_t requests, replies, dropped; };
struct Net    { bool connected; int8_t rssi; char ip[16]; char ssid[33]; uint32_t disconnects, attempts, hardRecoveries; };
struct Time   { bool valid; uint8_t mode; uint32_t ageMs, ppsCount;
                uint32_t rmcRx, rmcNoFix, rmcBad; char rmcStatus; };   // rmc*: written by the GNSS task   // mode: 0 none,1 NMEA,2 NMEA+PPS

struct Data {
  bool     uartOk;
  uint32_t uartBytes, uartBps, uartPb, backlogMax, uartRecoveries, silentMs;
  ParserStats parser;
  Stat     nmea[NK_COUNT];
  Stat     logRx[8];
  uint32_t logFwd[8], logDropped;
  RtcmStat rtcm[RTCM_SLOTS];
  uint32_t rtcmOther, rtcmSuppressed, rtcmFiltered;
  uint32_t cmdSent, cmdReplies;       // LG290P command channel
  Gga      gga;
  Ntrip    ntrip[2];
  Caster   caster;
  Ntp      ntp;
  Net      net;
  Time     time;
};
extern Data g;

void pushReply(const char* line, uint16_t len);          // GNSS task: store a receiver reply line
uint8_t getReplies(char out[][100], uint8_t maxN);       // newest first
void setGga(const Gga& in);     // GNSS task
void getGga(Gga& out);          // any task (spinlock-protected copy)
}
