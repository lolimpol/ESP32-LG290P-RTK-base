#pragma once
// Diagnostic logging + serial console + GNSS "USB stream" output, all through ONE non-blocking
// StreamBuffer drained by ONE task, so no producer (GNSS task!) can ever block on the serial port.
// Mode 0: human-readable diagnostics + CLI. Mode 1: raw GNSS stream for PPP/PPK (diagnostics muted,
// because mixing text into a binary stream would corrupt it).
#include <stdint.h>
#include "Config.h"

enum LogLevel : uint8_t { LL_ERROR, LL_WARN, LL_INFO, LL_DEBUG };

// Selectable stream bits (AppConfig::logMask). "SGA" is kept as requested: it is NOT a standard
// NMEA 0183 sentence; it is matched by its 3-letter ID and only forwarded if actually received.
enum LogBit : uint16_t { LB_GGA = 0x01, LB_SGA = 0x02, LB_GST = 0x04, LB_GSV = 0x08,
                         LB_RMC = 0x10, LB_GSA = 0x20, LB_RAWX = 0x40, LB_SFRBX = 0x80 };

namespace Log {
extern const char* const kStreamNames[8];
void     begin();                              // early in setup(): Serial @115200, task, queue
void     applyConfig(const AppConfig& c);      // level, mode, baud
void     print(LogLevel lv, const char* tag, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
bool     streamPush(const uint8_t* d, uint16_t n);   // non-blocking, whole-or-nothing
uint32_t streamDropped();
bool     streamMode();
}

#define LOGE(tag, ...) ::Log::print(LL_ERROR, tag, __VA_ARGS__)
#define LOGW(tag, ...) ::Log::print(LL_WARN,  tag, __VA_ARGS__)
#define LOGI(tag, ...) ::Log::print(LL_INFO,  tag, __VA_ARGS__)
#define LOGD(tag, ...) ::Log::print(LL_DEBUG, tag, __VA_ARGS__)
