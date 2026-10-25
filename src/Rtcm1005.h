#pragma once
// RTCM 3.x message 1005 (Stationary RTK Reference Station ARP) encoder.
// No Arduino dependencies (host-testable).
//
// Payload layout (152 bits = 19 bytes), MSB first:
//   DF002 message number           12  uint   = 1005
//   DF003 reference station ID     12  uint
//   DF021 ITRF realization year     6  uint   (0 = unspecified)
//   DF022 GPS indicator             1
//   DF023 GLONASS indicator         1
//   DF024 Galileo indicator         1
//   DF141 reference-station ind.    1         0 = physical, 1 = non-physical/computed
//   DF025 ARP ECEF-X               38  int    two's complement, unit 0.0001 m
//   DF142 single receiver osc.      1
//   (reserved)                      1         0
//   DF026 ARP ECEF-Y               38  int    0.0001 m
//   DF364 quarter-cycle indicator   2         0 = unspecified
//   DF027 ARP ECEF-Z               38  int    0.0001 m
// Frame = D3 | 00 13 | payload | CRC24Q(3 bytes over the first 22 bytes) = 25 bytes.
#include <stdint.h>
#include <stddef.h>

namespace Rtcm1005 {

const size_t PAYLOAD_LEN = 19;
const size_t FRAME_LEN   = 25;

struct Params {
  uint16_t stationId;
  uint8_t  itrfYear;
  bool     gps, glo, gal;
  bool     nonPhysical;      // DF141
  bool     singleOscillator; // DF142
  uint8_t  quarterCycle;     // DF364 (0..3)
  int64_t  x, y, z;          // ECEF in 0.1 mm units
};

struct Ecef { double x, y, z; };

Ecef    geodeticToEcef(double latDeg, double lonDeg, double heightM);  // WGS-84
struct Geodetic { double latDeg, lonDeg, h; };
Geodetic ecefToGeodetic(double x, double y, double z);                 // WGS-84, iterative
int64_t metersToUnits(double m);                                        // round to 0.1 mm
bool    encode(const Params& p, uint8_t out[FRAME_LEN]);                // false if a field is out of range

}  // namespace Rtcm1005
