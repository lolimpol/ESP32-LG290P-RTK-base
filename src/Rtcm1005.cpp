#include "Rtcm1005.h"
#include "Rtcm.h"
#include <math.h>
#include <string.h>

namespace Rtcm1005 {

Ecef geodeticToEcef(double latDeg, double lonDeg, double h) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double e2 = f * (2.0 - f);
  const double d2r = 3.14159265358979323846 / 180.0;
  double lat = latDeg * d2r, lon = lonDeg * d2r;
  double s = sin(lat), c = cos(lat);
  double N = a / sqrt(1.0 - e2 * s * s);
  Ecef r;
  r.x = (N + h) * c * cos(lon);
  r.y = (N + h) * c * sin(lon);
  r.z = (N * (1.0 - e2) + h) * s;
  return r;
}

Geodetic ecefToGeodetic(double x, double y, double z) {
  const double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2.0 - f), r2d = 180.0 / 3.14159265358979323846;
  double p = sqrt(x * x + y * y);
  double lat = atan2(z, p * (1.0 - e2)), h = 0;
  for (int i = 0; i < 8; i++) {
    double s = sin(lat), N = a / sqrt(1.0 - e2 * s * s);
    h = (fabs(lat) < 0.7853981633974483) ? p / cos(lat) - N : z / s - N * (1.0 - e2);
    lat = atan2(z, p * (1.0 - e2 * N / (N + h)));
  }
  Geodetic g; g.latDeg = lat * r2d; g.lonDeg = atan2(y, x) * r2d; g.h = h; return g;
}

int64_t metersToUnits(double m) { return (int64_t)floor(m * 10000.0 + 0.5); }

// MSB-first bit writer into a zeroed buffer.
static void putBits(uint8_t* buf, size_t& bitPos, unsigned nbits, uint64_t v) {
  for (unsigned i = 0; i < nbits; i++) {
    unsigned bv = (unsigned)((v >> (nbits - 1 - i)) & 1u);
    if (bv) buf[bitPos >> 3] |= (uint8_t)(0x80u >> (bitPos & 7));
    bitPos++;
  }
}

bool encode(const Params& p, uint8_t out[FRAME_LEN]) {
  const int64_t LIM = ((int64_t)1 << 37) - 1;   // 38-bit signed range
  if (p.stationId > 4095 || p.itrfYear > 63 || p.quarterCycle > 3) return false;
  if (p.x > LIM || p.x < -LIM - 1 || p.y > LIM || p.y < -LIM - 1 || p.z > LIM || p.z < -LIM - 1) return false;

  memset(out, 0, FRAME_LEN);
  out[0] = 0xD3; out[1] = 0x00; out[2] = (uint8_t)PAYLOAD_LEN;
  uint8_t* pl = out + 3;
  const uint64_t M38 = ((uint64_t)1 << 38) - 1;
  size_t bp = 0;
  putBits(pl, bp, 12, 1005);
  putBits(pl, bp, 12, p.stationId);
  putBits(pl, bp, 6,  p.itrfYear);
  putBits(pl, bp, 1,  p.gps);
  putBits(pl, bp, 1,  p.glo);
  putBits(pl, bp, 1,  p.gal);
  putBits(pl, bp, 1,  p.nonPhysical);
  putBits(pl, bp, 38, (uint64_t)p.x & M38);
  putBits(pl, bp, 1,  p.singleOscillator);
  putBits(pl, bp, 1,  0);
  putBits(pl, bp, 38, (uint64_t)p.y & M38);
  putBits(pl, bp, 2,  p.quarterCycle);
  putBits(pl, bp, 38, (uint64_t)p.z & M38);
  if (bp != PAYLOAD_LEN * 8) return false;       // layout self-check

  uint32_t crc = Rtcm::crc24q(out, 3 + PAYLOAD_LEN);
  out[22] = (uint8_t)(crc >> 16); out[23] = (uint8_t)(crc >> 8); out[24] = (uint8_t)crc;
  return true;
}

}  // namespace Rtcm1005
