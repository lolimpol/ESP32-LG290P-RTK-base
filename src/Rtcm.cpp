#include "Rtcm.h"

namespace Rtcm {

// 4-bit lookup for the 24-bit MSB-first CRC (two steps per byte, 64 B of flash).
static const uint32_t kNib[16] = {
  0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
  0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E};

uint32_t crc24q(const uint8_t* d, size_t n) {
  uint32_t c = 0;
  while (n--) {
    c ^= (uint32_t)(*d++) << 16;
    c = ((c << 4) & 0xFFFFFF) ^ kNib[(c >> 20) & 0xF];
    c = ((c << 4) & 0xFFFFFF) ^ kNib[(c >> 20) & 0xF];
  }
  return c;
}

const CatalogEntry kCatalog[] = {
  {1005, "Stationary RTK ARP (ESP32-generated)"},
  {1019, "GPS ephemeris"},
  {1020, "GLONASS ephemeris"},
  {1042, "BeiDou ephemeris"},
  {1045, "Galileo F/NAV ephemeris"},
  {1046, "Galileo I/NAV ephemeris"},
  {1033, "Receiver/antenna descriptors"},
  {1230, "GLONASS code-phase biases"},
  {1074, "GPS MSM4"},     {1077, "GPS MSM7"},
  {1084, "GLONASS MSM4"}, {1087, "GLONASS MSM7"},
  {1094, "Galileo MSM4"}, {1097, "Galileo MSM7"},
  {1124, "BeiDou MSM4"},  {1127, "BeiDou MSM7"},
  {1117, "QZSS MSM7"},    {1137, "NavIC/IRNSS MSM7"},
};
const uint8_t kCatalogCount = sizeof(kCatalog) / sizeof(kCatalog[0]);

int catalogIndex(uint16_t num) {
  for (uint8_t i = 0; i < kCatalogCount; i++)
    if (kCatalog[i].num == num) return i;
  return -1;
}

}  // namespace Rtcm
