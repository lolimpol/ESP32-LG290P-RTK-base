#pragma once
// Runtime configuration: one POD struct, validated before commit, published to modules via a
// generation counter (modules re-read their section when the counter changes - no reboot needed).
// RULE FOR FUTURE VERSIONS: only append fields at the END of AppConfig (see ConfigStorage migration).
#include <stdint.h>
#include <stddef.h>
#include "UserConfig.h"

#define CFG_VERSION 1

struct NtripClientCfg {
  uint8_t  enabled;
  uint8_t  tls;          // 1 = TLS (encrypted, server certificate NOT verified - see README)
  uint8_t  version;      // 1 = "SOURCE" (Rev1), 2 = "POST" (Rev2)
  uint8_t  reserved;
  uint16_t port;
  uint16_t reconnectS;   // base reconnect interval; exponential backoff up to 300 s
  char host[64];
  char mount[40];
  char user[32];         // Rev2 only
  char pass[32];         // Rev1: source password; Rev2: Basic-auth password
};

struct AppConfig {
  char     wifiSsid[33];
  char     wifiPass[65];
  char     hostname[32];
  char     webUser[24];
  char     webPass[32];
  uint8_t  logLevel;
  uint8_t  usbMode;          // 0 = diagnostic console, 1 = raw GNSS stream (PPP/PPK)
  uint32_t gnssBaud;
  int8_t   gnssRx, gnssTx;
  double   x, y, z;          // ARP ECEF in metres, as entered by the user
  int64_t  ecef[3];          // same, in 0.1 mm units (RTCM 1005 resolution), derived
  uint16_t stationId;
  uint8_t  itrfYear;
  uint8_t  gpsInd, gloInd, galInd;
  uint8_t  gen1005;
  uint16_t gen1005IntervalS;
  uint32_t rtcmMask;         // bit i = Rtcm::kCatalog[i] forwarded (bit 0 unused: see gen1005)
  NtripClientCfg ntrip[2];
  uint8_t  casterEnabled, casterMaxClients;
  uint16_t casterPort, casterTimeoutS;
  char     casterMount[40], casterUser[32], casterPass[32];
  uint8_t  ntpEnabled;
  uint16_t ntpPort;
  int16_t  ntpLatencyMs;
  uint32_t usbBaud;
  uint16_t logMask;
};

namespace Config {
void     begin();                                     // defaults into the live config
void     setDefaults(AppConfig& c);
bool     validate(AppConfig& c, char* err, size_t errLen);   // may normalise; false = reject
void     updateDerived(AppConfig& c);                 // lat/lon/h -> ECEF
void     snapshot(AppConfig& out);                    // consistent copy (mutex-protected)
bool     commit(const AppConfig& candidate, char* err, size_t errLen);  // validate + publish + generation++
uint32_t generation();
bool     restartRequired(const AppConfig& a, const AppConfig& b);       // UART pins changed
void     getWebCreds(char* user, size_t un, char* pass, size_t pn);
uint8_t  warnings();                                  // bit0 default web pw, bit1 default position, bit2 default Wi-Fi
}
