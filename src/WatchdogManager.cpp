#include "WatchdogManager.h"
#include "LogManager.h"
#include "UserConfig.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_heap_caps.h>

namespace Watchdog {

static const uint32_t RTC_MAGIC = 0x57444731;
static const uint32_t HEAP_WARN = 30000, HEAP_CRIT = 14000, HEAP_CRIT_SECS = 60;
static RTC_NOINIT_ATTR RtcInfo s_rtc;      // survives software/WDT resets, cleared on power-up

struct Slot {
  const char* name; TaskHandle_t h; uint32_t softMs; volatile uint32_t last; volatile bool recover; bool flagged; bool reg; uint8_t core;
};
static Slot s_slot[WD_COUNT];
static uint32_t s_soft = 0;
static uint32_t s_heapLowSecs = 0, s_lastHeapWarn = 0;

void begin() {
  if (s_rtc.magic != RTC_MAGIC) { memset(&s_rtc, 0, sizeof(s_rtc)); s_rtc.magic = RTC_MAGIC; }
  s_rtc.boots++;
  switch (esp_reset_reason()) {
    case ESP_RST_TASK_WDT: case ESP_RST_INT_WDT: case ESP_RST_WDT: s_rtc.wdtResets++; break;
    case ESP_RST_PANIC:    s_rtc.panicResets++; break;
    case ESP_RST_BROWNOUT: s_rtc.brownouts++; break;
    default: break;
  }
  if (s_rtc.pending == RR_HEAP) s_rtc.heapResets++;
  if (s_rtc.pending == RR_WIFI) s_rtc.wifiResets++;
  s_rtc.pending = 0;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t c = { (uint32_t)TWDT_TIMEOUT_S * 1000u, 0x3, true };
  esp_task_wdt_reconfigure(&c);
#else
  esp_task_wdt_init(TWDT_TIMEOUT_S, true);   // re-init updates timeout/panic if already running
#endif
}

void registerTask(WdId id, const char* name, uint32_t softMs) {
  Slot& s = s_slot[id];
  s.name = name; s.h = xTaskGetCurrentTaskHandle(); s.softMs = softMs; s.last = millis();
  s.recover = false; s.flagged = false; s.core = (uint8_t)xPortGetCoreID(); s.reg = true;
  esp_task_wdt_add(NULL);
}

void kick(WdId id) {
  s_slot[id].last = millis();
  s_slot[id].flagged = false;
  esp_task_wdt_reset();
}

bool consumeRecover(WdId id) {
  if (!s_slot[id].recover) return false;
  s_slot[id].recover = false;
  return true;
}

void restartDevice(RestartReason why) {
  s_rtc.pending = (uint8_t)why;
  LOGW("wd", "controlled restart, reason %u", (unsigned)why);
  delay(200);
  ESP.restart();
}

void supervise() {
  uint32_t now = millis();
  kick(WD_MAIN);
  for (uint8_t i = 0; i < WD_COUNT; i++) {
    Slot& s = s_slot[i];
    if (!s.reg || i == WD_MAIN) continue;
    if (now - s.last > s.softMs && !s.flagged) {
      s.flagged = true; s.recover = true; s_soft++;
      LOGW("wd", "task '%s' missed heartbeat (%lu ms) - soft recovery requested", s.name, (unsigned long)(now - s.last));
    }
    if (s.h && uxTaskGetStackHighWaterMark(s.h) * sizeof(StackType_t) < 400)
      LOGW("wd", "task '%s' stack nearly exhausted", s.name);
  }
  uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < HEAP_WARN && now - s_lastHeapWarn > 60000) {
    s_lastHeapWarn = now;
    LOGW("mem", "low heap: free %lu min %lu largest %lu", (unsigned long)freeHeap, (unsigned long)ESP.getMinFreeHeap(),
         (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  }
  if (freeHeap < HEAP_CRIT) {
    if (++s_heapLowSecs >= HEAP_CRIT_SECS) restartDevice(RR_HEAP);   // leak/fragmentation: restart cleanly
  } else s_heapLowSecs = 0;
}

void taskInfo(uint8_t i, TaskInfo& t) {
  Slot& s = s_slot[i];
  t.name = s.name ? s.name : "-"; t.registered = s.reg; t.core = s.core;
  t.stackFreeBytes = (s.reg && s.h) ? uxTaskGetStackHighWaterMark(s.h) * sizeof(StackType_t) : 0;
  t.kickAgeMs = s.reg ? millis() - s.last : 0;
}

const RtcInfo& rtc() { return s_rtc; }
uint32_t softRecoveries() { return s_soft; }

const char* resetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";   case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";        case ESP_RST_INT_WDT: return "interrupt WDT";
    case ESP_RST_TASK_WDT: return "task WDT";  case ESP_RST_WDT: return "other WDT";
    case ESP_RST_BROWNOUT: return "brownout";  case ESP_RST_EXT: return "external pin";
    case ESP_RST_DEEPSLEEP: return "deep sleep"; default: return "unknown";
  }
}

}  // namespace Watchdog
