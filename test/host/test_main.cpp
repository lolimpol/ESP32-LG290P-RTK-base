// Host-side tests for the hardware-independent core: CRC24Q, RTCM 1005, stream parser, RTCM bus.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include "Rtcm.h"
#include "Rtcm1005.h"
#include "GnssParser.h"
#include "RtcmBus.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static std::vector<uint8_t> mkRtcm(uint16_t msg, size_t payloadLen, uint8_t fill) {
  std::vector<uint8_t> f(payloadLen + 6, fill);
  f[0] = 0xD3; f[1] = (payloadLen >> 8) & 3; f[2] = payloadLen & 0xFF;
  f[3] = msg >> 4; f[4] = (uint8_t)((msg & 0xF) << 4) | (fill & 0x0F);
  uint32_t c = Rtcm::crc24q(f.data(), payloadLen + 3);
  f[payloadLen + 3] = c >> 16; f[payloadLen + 4] = c >> 8; f[payloadLen + 5] = c;
  return f;
}
static std::vector<uint8_t> mkNmea(const char* body) {
  uint8_t x = 0; for (const char* p = body; *p; p++) x ^= (uint8_t)*p;
  char b[200]; int n = snprintf(b, sizeof b, "$%s*%02X\r\n", body, x);
  return std::vector<uint8_t>(b, b + n);
}
static std::vector<uint8_t> mkUbx(uint8_t cls, uint8_t id, size_t plen) {
  std::vector<uint8_t> f(plen + 8);
  f[0] = 0xB5; f[1] = 0x62; f[2] = cls; f[3] = id; f[4] = plen & 0xFF; f[5] = plen >> 8;
  for (size_t i = 0; i < plen; i++) f[6 + i] = (uint8_t)(i * 7 + 1);
  uint8_t a = 0, b = 0; for (size_t i = 2; i < plen + 6; i++) { a += f[i]; b += a; }
  f[plen + 6] = a; f[plen + 7] = b; return f;
}

struct Sink : GnssSink {
  std::vector<uint16_t> rtcm; int nmea = 0, ubx = 0; std::vector<uint8_t> ubxId;
  void onRtcm(const uint8_t*, uint16_t, uint16_t m) override { rtcm.push_back(m); }
  void onNmea(const char*, uint16_t) override { nmea++; }
  void onUbx(const uint8_t*, uint16_t, uint8_t, uint8_t id) override { ubx++; ubxId.push_back(id); }
};
static void feedAll(GnssParser& p, const std::vector<uint8_t>& v, size_t chunk = 97) {
  for (size_t i = 0; i < v.size(); i += chunk) p.feed(v.data() + i, std::min(chunk, v.size() - i));
}
static void app(std::vector<uint8_t>& a, const std::vector<uint8_t>& b) { a.insert(a.end(), b.begin(), b.end()); }

// ---- independent RTCM 1005 decoder (reads bits MSB first) ----
static uint64_t getBits(const uint8_t* b, size_t& pos, int n) {
  uint64_t v = 0; for (int i = 0; i < n; i++, pos++) v = (v << 1) | ((b[pos >> 3] >> (7 - (pos & 7))) & 1); return v;
}
static int64_t sx(uint64_t v, int n) { return (v & ((uint64_t)1 << (n - 1))) ? (int64_t)(v | (~0ULL << n)) : (int64_t)v; }

static void testCrc() {
  CHECK(Rtcm::crc24q((const uint8_t*)"123456789", 9) == 0xCDE703);
  auto f = mkRtcm(1019, 40, 0x5A);
  CHECK(Rtcm::crc24q(f.data(), f.size()) == 0);   // CRC over data+CRC is zero
}

static void test1005() {
  // Equator/prime meridian, h=0 -> X = a
  auto e = Rtcm1005::geodeticToEcef(0, 0, 0);
  CHECK(fabs(e.x - 6378137.0) < 1e-6 && fabs(e.y) < 1e-6 && fabs(e.z) < 1e-6);
  auto n = Rtcm1005::geodeticToEcef(90, 0, 0);     // pole: Z = b = 6356752.314245
  CHECK(fabs(n.z - 6356752.314245) < 1e-3);

  Rtcm1005::Params p = {};
  auto ec = Rtcm1005::geodeticToEcef(50.8503, 5.6910, 112.345);   // negative-Y/Z cases below
  p.stationId = 1234; p.itrfYear = 14; p.gps = p.glo = p.gal = true; p.singleOscillator = true;
  p.x = Rtcm1005::metersToUnits(ec.x); p.y = Rtcm1005::metersToUnits(ec.y); p.z = Rtcm1005::metersToUnits(ec.z);
  uint8_t f[25];
  CHECK(Rtcm1005::encode(p, f));
  CHECK(f[0] == 0xD3 && f[1] == 0x00 && f[2] == 19);
  CHECK(Rtcm::crc24q(f, 25) == 0);
  CHECK(Rtcm::msgNumber(f) == 1005);
  size_t b = 24;
  CHECK(getBits(f, b, 12) == 1005);
  CHECK(getBits(f, b, 12) == 1234);
  CHECK(getBits(f, b, 6) == 14);
  CHECK(getBits(f, b, 1) == 1 && getBits(f, b, 1) == 1 && getBits(f, b, 1) == 1);
  CHECK(getBits(f, b, 1) == 0);
  CHECK(sx(getBits(f, b, 38), 38) == p.x);
  CHECK(getBits(f, b, 1) == 1);
  CHECK(getBits(f, b, 1) == 0);
  CHECK(sx(getBits(f, b, 38), 38) == p.y);
  CHECK(getBits(f, b, 2) == 0);
  CHECK(sx(getBits(f, b, 38), 38) == p.z);
  CHECK(b == 24 + 152);

  // negative coordinates round-trip (southern/western hemisphere)
  auto ng = Rtcm1005::geodeticToEcef(-33.8688, -70.6693, 520.0);
  p.x = Rtcm1005::metersToUnits(ng.x); p.y = Rtcm1005::metersToUnits(ng.y); p.z = Rtcm1005::metersToUnits(ng.z);
  CHECK(p.y < 0 && p.z < 0);
  CHECK(Rtcm1005::encode(p, f));
  b = 24 + 12 + 12 + 6 + 4; CHECK(sx(getBits(f, b, 38), 38) == p.x);
  b += 2; CHECK(sx(getBits(f, b, 38), 38) == p.y);
  b += 2; CHECK(sx(getBits(f, b, 38), 38) == p.z);
  // range / field rejection
  Rtcm1005::Params bad = p; bad.stationId = 4096; CHECK(!Rtcm1005::encode(bad, f));
  bad = p; bad.x = (int64_t)1 << 37; CHECK(!Rtcm1005::encode(bad, f));
}

static void testEcefRoundTrip() {
  const double pts[][3] = {{50.8503, 5.6910, 112.3}, {-33.8688, -70.6693, 520}, {0, 0, 0}, {89.9, 10, 50}, {-89.5, 170, 1000}, {64.1, -21.9, -20}, {1.0, 179.99, 8000}};
  for (auto& q : pts) {
    auto e = Rtcm1005::geodeticToEcef(q[0], q[1], q[2]);
    auto g = Rtcm1005::ecefToGeodetic(e.x, e.y, e.z);
    CHECK(fabs(g.latDeg - q[0]) < 1e-9 && fabs(g.lonDeg - q[1]) < 1e-9 && fabs(g.h - q[2]) < 1e-4);
  }
  // XYZ entered directly (the firmware's primary input) -> 0.1 mm units -> 1005 -> decoded back
  Rtcm1005::Params p = {}; p.x = Rtcm1005::metersToUnits(4092296.4623); p.y = Rtcm1005::metersToUnits(358029.5482); p.z = Rtcm1005::metersToUnits(4862865.6422);
  CHECK(p.x == 40922964623LL && p.y == 3580295482LL && p.z == 48628656422LL);
  uint8_t f[25]; CHECK(Rtcm1005::encode(p, f));
  size_t b = 24 + 12 + 12 + 6 + 4; CHECK(sx(getBits(f, b, 38), 38) == p.x);
}

static void testCatalog() {
  const uint16_t want[] = {1005, 1033, 1077, 1087, 1097, 1117, 1127, 1137, 1230};
  for (uint16_t m : want) CHECK(Rtcm::catalogIndex(m) >= 0);
  CHECK(Rtcm::kCatalogCount <= 24 && Rtcm::kCatalogCount <= 32);       // status slots / 32-bit mask
  CHECK(Rtcm::catalogIndex(1005) == Rtcm::IDX_1005);
  CHECK(Rtcm::catalogIndex(1019) == 1 && Rtcm::catalogIndex(1127) == 15);   // existing bit positions unchanged
  auto f = mkRtcm(1137, 40, 0x05); Sink s; GnssParser P(s); feedAll(P, f);
  CHECK(s.rtcm.size() == 1 && s.rtcm[0] == 1137);
}

static void testParser() {
  Sink s; GnssParser P(s);
  std::vector<uint8_t> st;
  auto r1 = mkRtcm(1019, 61, 0x11), r2 = mkRtcm(1005, 19, 0x22), r3 = mkRtcm(1077, 1023, 0x33);
  app(st, mkNmea("GNGGA,123519.00,5051.0180,N,00541.4600,E,1,12,0.8,100.1,M,47.0,M,,")); app(st, r1);
  app(st, mkNmea("GNRMC,123519.00,A,5051.0180,N,00541.4600,E,0.0,0.0,121026,,,A,V")); app(st, r2); app(st, r3);
  app(st, mkUbx(0x02, 0x15, 100)); app(st, mkUbx(0x02, 0x13, 20));
  feedAll(P, st, 13);
  CHECK(s.rtcm.size() == 3 && s.rtcm[0] == 1019 && s.rtcm[1] == 1005 && s.rtcm[2] == 1077);
  CHECK(s.nmea == 2 && s.ubx == 2 && s.ubxId[0] == 0x15 && s.ubxId[1] == 0x13);
  CHECK(P.stats().rtcmCrcErr == 0 && P.stats().resyncs == 0);

  // 2. corrupted frame (flip one payload bit) followed by good frames; corrupted one dropped
  Sink s2; GnssParser P2(s2);
  auto bad = mkRtcm(1020, 50, 0x44); bad[20] ^= 0x01;
  std::vector<uint8_t> st2; app(st2, bad); app(st2, mkRtcm(1019, 30, 0x55)); app(st2, mkRtcm(1046, 30, 0x66));
  feedAll(P2, st2);
  CHECK(s2.rtcm.size() == 2 && s2.rtcm[0] == 1019 && s2.rtcm[1] == 1046);
  CHECK(P2.stats().rtcmCrcErr == 1);

  // 3. valid frame hidden inside a corrupted frame's span must be recovered
  Sink s3; GnssParser P3(s3);
  auto inner = mkRtcm(1230, 12, 0x77);
  std::vector<uint8_t> st3 = {0xD3, 0x00, 0xC8};      // header announcing 200-byte payload (lie)
  app(st3, inner);                                    // real frame follows
  for (int i = 0; i < 300; i++) st3.push_back((uint8_t)(0x30 + (i & 7)));  // filler so lie completes
  app(st3, mkRtcm(1019, 20, 0x12));
  feedAll(P3, st3, 5);
  bool has1230 = false, has1019 = false;
  for (auto m : s3.rtcm) { has1230 |= (m == 1230); has1019 |= (m == 1019); }
  CHECK(has1230 && has1019);

  // 4. truncated frame, then idle timeout, then a good frame
  Sink s4; GnssParser P4(s4);
  auto part = mkRtcm(1020, 80, 0x21); part.resize(40);
  P4.feed(part.data(), part.size());
  CHECK(P4.midFrame());
  P4.abortPartial();
  CHECK(!P4.midFrame());
  auto ok4 = mkRtcm(1019, 30, 0x22); P4.feed(ok4.data(), ok4.size());
  CHECK(s4.rtcm.size() == 1 && s4.rtcm[0] == 1019);

  // 5. pure garbage + NMEA with bad checksum + binary inside "NMEA" + random noise: no false frames, no crash
  Sink s5; GnssParser P5(s5);
  std::vector<uint8_t> st5; srand(1);
  for (int i = 0; i < 200000; i++) st5.push_back((uint8_t)(rand() & 0xFF));
  feedAll(P5, st5, 211);
  CHECK(s5.rtcm.empty());                                // 24-bit CRC: false accept ~ 6e-8 per candidate
  auto g = mkNmea("GNGGA,1,2,3"); g[g.size() - 4] ^= 1;  // wrong checksum char
  Sink s6; GnssParser P6(s6); feedAll(P6, g); CHECK(s6.nmea == 0 && P6.stats().nmeaBadChecksum == 1);
  // after noise the parser is still alive
  auto okf = mkRtcm(1019, 30, 0x31); P5.feed(okf.data(), okf.size());
  CHECK(!s5.rtcm.empty() && s5.rtcm.back() == 1019);

  // 6. '$' inside RTCM payload must not start NMEA
  Sink s7; GnssParser P7(s7);
  auto dollar = mkRtcm(1019, 60, 0x24);                   // payload full of '$'
  feedAll(P7, dollar); CHECK(s7.rtcm.size() == 1 && s7.nmea == 0);

  // 7. oversize UBX length is rejected and parser recovers
  Sink s8; GnssParser P8(s8);
  std::vector<uint8_t> u = {0xB5, 0x62, 0x02, 0x15, 0xFF, 0xFF};
  app(u, mkRtcm(1019, 30, 0x41)); feedAll(P8, u);
  CHECK(s8.rtcm.size() == 1 && P8.stats().ubxOversize == 1);
}

static void testBus() {
  static RtcmBus bus; bus.begin();
  RtcmBus::Cursor fast, slow; bus.attach(fast); bus.attach(slow);
  uint8_t out[2048]; uint32_t seq = 0, got = 0, lastSeen = 0; bool ordered = true;
  for (int i = 0; i < 4000; i++) {
    size_t pl = 20 + (i * 37) % 900;
    auto f = mkRtcm(1074, pl, 0x10);
    f[5] = seq >> 8; f[6] = seq;                 // sequence stamp in payload (CRC not re-verified here)
    CHECK(bus.publish(f.data(), (uint16_t)f.size())); seq++;
    uint16_t n = bus.read(fast, out, sizeof out);
    size_t o = 0;
    while (o < n) {                              // walk frames in the returned buffer
      size_t l = (((out[o + 1] & 3) << 8) | out[o + 2]) + 6;
      uint32_t s = (out[o + 5] << 8) | out[o + 6];
      if (got && s != ((lastSeen + 1) & 0xFFFF)) ordered = false;
      lastSeen = s; got++; o += l;
    }
    CHECK(o == n);
  }
  CHECK(got == 4000 && ordered);
  // slow consumer was overtaken: must jump to an intact frame boundary
  uint16_t n = bus.read(slow, out, sizeof out);
  CHECK(slow.lapped >= 1 && n > 0 && out[0] == 0xD3);
  size_t o = 0; while (o < n) o += (((out[o + 1] & 3) << 8) | out[o + 2]) + 6;
  CHECK(o == n);
  CHECK(bus.overwritten > 0);
  // cached 1005
  Rtcm1005::Params p = {}; p.x = 1; p.y = 2; p.z = 3; uint8_t f1005[25]; Rtcm1005::encode(p, f1005);
  bus.publish(f1005, 25); uint8_t c[32]; uint16_t cl = 0;
  CHECK(bus.lastReference(c, cl) && cl == 25 && memcmp(c, f1005, 25) == 0);
}

int main() {
  testCrc(); testCatalog(); test1005(); testEcefRoundTrip(); testParser(); testBus();
  printf(fails ? "\n%d FAILURE(S)\n" : "\nALL TESTS PASSED\n", fails);
  return fails != 0;
}
