#pragma once
// Two-tier supervision:
//  1) SOFT: every worker task kicks a heartbeat; the supervisor (main loop, 1 Hz) flags a task whose
//     heartbeat is overdue, and the task tears down/reopens its own resources (socket, UART...).
//  2) HARD: all workers are also subscribed to the ESP-IDF task watchdog (TWDT_TIMEOUT_S). A task that
//     is truly wedged inside a call is reset by the TWDT - the last resort, counted across reboots.
#include <stdint.h>

enum WdId : uint8_t { WD_GNSS, WD_NTRIP1, WD_NTRIP2, WD_CASTER, WD_NTP, WD_NET, WD_CONSOLE, WD_MAIN, WD_COUNT };
enum RestartReason : uint8_t { RR_USER = 1, RR_HEAP, RR_WIFI, RR_OTA, RR_CONFIG };

namespace Watchdog {
struct RtcInfo { uint32_t magic, boots, wdtResets, panicResets, brownouts, heapResets, wifiResets; uint8_t pending; };
struct TaskInfo { const char* name; uint8_t core; uint32_t stackFreeBytes; uint32_t kickAgeMs; bool registered; };

void begin();                                                   // call first in setup()
void registerTask(WdId id, const char* name, uint32_t softTimeoutMs);   // from inside the task
void kick(WdId id);
bool consumeRecover(WdId id);                                   // true once after a soft timeout
void supervise();                                               // main loop, ~1 Hz
void restartDevice(RestartReason why);                          // clean, counted restart (never returns)
void taskInfo(uint8_t i, TaskInfo& t);
const RtcInfo& rtc();
uint32_t softRecoveries();
const char* resetReason();
}
