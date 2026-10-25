#include "GnssRouter.h"
#include "Rtcm1005.h"
#include "SystemStatus.h"
#include "LogManager.h"
#include "GnssCmd.h"
#include <Arduino.h>
#include <esp_timer.h>
#include <string.h>
#include <stdlib.h>

using Status::g;

void GnssRouter::applyConfig(const AppConfig& c) {
  rtcmMask_ = c.rtcmMask; logMask_ = c.logMask; gen_ = c.gen1005; genMs_ = (uint32_t)c.gen1005IntervalS * 1000u;
  Rtcm1005::Params p = {};
  p.stationId = c.stationId; p.itrfYear = c.itrfYear; p.gps = c.gpsInd; p.glo = c.gloInd; p.gal = c.galInd;
  p.nonPhysical = false; p.singleOscillator = true; p.quarterCycle = 0;
  p.x = c.ecef[0]; p.y = c.ecef[1]; p.z = c.ecef[2];
  genValid_ = Rtcm1005::encode(p, f1005_);
  if (!genValid_) LOGE("rtcm", "cannot encode 1005 from configured position");
  genNow_ = true;                                  // publish immediately after any change
}

void GnssRouter::tick(uint32_t now) {
  if (!gen_ || !genValid_) return;
  if (genNow_ || now - lastGen_ >= genMs_) {
    genNow_ = false; lastGen_ = now;
    if (bus_.publish(f1005_, sizeof(f1005_))) {
      Status::RtcmStat& s = g.rtcm[Rtcm::IDX_1005];
      s.gen++; s.fwd++; s.bytes += sizeof(f1005_); s.lastMs = now;
    }
  }
}

// ---- RTCM -----------------------------------------------------------------------------------
void GnssRouter::onRtcm(const uint8_t* f, uint16_t len, uint16_t msg) {
  // The LG290P is not expected to send 1005; if it ever does (or sends 1006) while the ESP32
  // generates its own reference position, drop it so rovers never see two conflicting positions.
  if ((msg == 1005 || msg == 1006) && gen_) { g.rtcmSuppressed++; return; }
  int idx = Rtcm::catalogIndex(msg);
  if (idx < 0) { g.rtcmOther++; return; }
  Status::RtcmStat& s = g.rtcm[idx];
  s.rx++; s.bytes += len; s.lastMs = millis();
  if (!(rtcmMask_ & (1u << idx))) { g.rtcmFiltered++; return; }
  if (bus_.publish(f, len)) s.fwd++;
}

// ---- NMEA -----------------------------------------------------------------------------------
static int split(char* s, char** f, int maxf) {
  int n = 0; f[n++] = s;
  for (; *s && n < maxf; s++) if (*s == ',') { *s = 0; f[n++] = s + 1; }
  return n;
}
static double coord(const char* s, char h) {
  if (!*s) return 0;
  double v = atof(s); int d = (int)(v / 100.0);
  double r = d + (v - d * 100.0) / 60.0;
  return (h == 'S' || h == 'W') ? -r : r;
}
static int dig2(const char* p) { return (p[0] >= '0' && p[0] <= '9' && p[1] >= '0' && p[1] <= '9') ? (p[0] - '0') * 10 + (p[1] - '0') : -1; }

void GnssRouter::onNmea(const char* l, uint16_t len) {
  NmeaKind k = NK_OTHER; int stream = -1;
  if (len > 7) {
    if (l[1] == 'P') k = NK_PROP;
    else {
      const char* t = l + 3;
      if      (!memcmp(t, "GGA", 3)) { k = NK_GGA; stream = 0; }
      else if (!memcmp(t, "RMC", 3)) { k = NK_RMC; stream = 4; }
      else if (!memcmp(t, "GLL", 3)) k = NK_GLL;
      else if (!memcmp(t, "VTG", 3)) k = NK_VTG;
      else if (!memcmp(t, "GSA", 3)) { k = NK_GSA; stream = 5; }
      else if (!memcmp(t, "GST", 3)) { k = NK_GST; stream = 2; }
      else if (!memcmp(t, "GSV", 3)) { k = NK_GSV; stream = 3; }
      else if (!memcmp(t, "SGA", 3)) { k = NK_SGA; stream = 1; }
    }
  }
  g.nmea[k].n++; g.nmea[k].lastMs = millis();
  if (stream >= 0) {
    g.logRx[stream].n++; g.logRx[stream].lastMs = millis();
    if ((logMask_ & (1u << stream)) && Log::streamPush((const uint8_t*)l, len)) g.logFwd[stream]++;
  }
  if (k == NK_PROP) {                            // Quectel $PQTM...: keep replies to our commands, ignore periodic status
    char b[100]; uint16_t m = len < sizeof(b) ? len : (uint16_t)(sizeof(b) - 1);
    memcpy(b, l, m); while (m && (b[m - 1] == '\r' || b[m - 1] == '\n')) m--; b[m] = 0;
    uint32_t ls = GnssCmd::lastSendMs();
    if ((ls && millis() - ls < 3000) || strstr(b, ",OK") || strstr(b, ",ERROR")) {
      Status::pushReply(b, m); g.cmdReplies++; LOGI("gnss", "receiver: %s", b);
    }
    return;
  }
  if (k != NK_GGA && k != NK_RMC) return;       // only GGA/RMC are parsed; everything else is just counted

  char tmp[128]; uint16_t n = len < sizeof(tmp) ? len : (uint16_t)(sizeof(tmp) - 1);
  memcpy(tmp, l, n); tmp[n] = 0;
  char* star = strchr(tmp, '*'); if (star) *star = 0;
  char* f[16]; int nf = split(tmp, f, 16);
  if (k == NK_GGA) handleGga(f, nf, l, len); else handleRmc(f, nf);
}

void GnssRouter::handleGga(char* f[], int n, const char* raw, uint16_t len) {
  if (n < 12) return;
  Status::Gga a; memset(&a, 0, sizeof(a));
  strlcpy(a.utc, f[1], sizeof(a.utc));
  a.lat = coord(f[2], f[3][0]); a.lon = coord(f[4], f[5][0]);
  a.quality = (uint8_t)atoi(f[6]); a.sats = (uint8_t)atoi(f[7]); a.hdop = (float)atof(f[8]);
  a.alt = (float)atof(f[9]); a.sep = (float)atof(f[11]);
  a.valid = a.quality > 0; a.ms = millis();
  uint16_t m = len < sizeof(a.raw) ? len : (uint16_t)(sizeof(a.raw) - 1);
  memcpy(a.raw, raw, m); while (m && (a.raw[m - 1] == '\r' || a.raw[m - 1] == '\n')) m--; a.raw[m] = 0;
  Status::setGga(a);
}

void GnssRouter::handleRmc(char* f[], int n) {
  g.time.rmcRx++;
  if (n < 10) { g.time.rmcBad++; return; }
  g.time.rmcStatus = f[2][0] ? f[2][0] : '?';
  if (f[2][0] != 'A') { g.time.rmcNoFix++; return; }       // need an 'A'ctive (valid) fix
  const char* t = f[1]; const char* d = f[9];
  if (strlen(t) < 6 || strlen(d) != 6) { g.time.rmcBad++; return; }
  int hh = dig2(t), mi = dig2(t + 2), ss = dig2(t + 4), dd = dig2(d), mo = dig2(d + 2), yy = dig2(d + 4);
  if (hh < 0 || mi < 0 || ss < 0 || dd < 1 || mo < 1 || mo > 12 || yy < 24 || hh > 23 || mi > 59 || ss > 60) { g.time.rmcBad++; return; }
  uint16_t frac = 0;
  const char* dot = strchr(t, '.');
  if (dot) { char b[4] = {'0', '0', '0', 0}; for (int i = 0; i < 3 && dot[1 + i]; i++) b[i] = dot[1 + i]; frac = (uint16_t)atoi(b); }
  time_.onRmc(GnssTime::civilToUnix(2000 + yy, mo, dd, hh, mi, ss), frac, esp_timer_get_time());
}

// ---- UBX (only forwarded to the log stream; LG290P does not natively emit UBX) -------------------
void GnssRouter::onUbx(const uint8_t* f, uint16_t len, uint8_t cls, uint8_t id) {
  int stream = -1;
  if (cls == 0x02 && id == 0x15) stream = 6;       // UBX-RXM-RAWX
  else if (cls == 0x02 && id == 0x13) stream = 7;  // UBX-RXM-SFRBX
  if (stream < 0) return;
  g.logRx[stream].n++; g.logRx[stream].lastMs = millis();
  if ((logMask_ & (1u << stream)) && Log::streamPush(f, len)) g.logFwd[stream]++;
}
