#include "GnssCmd.h"
#include "Config.h"
#include "SystemStatus.h"
#include "LogManager.h"
#include <Arduino.h>

namespace GnssCmd {

static SemaphoreHandle_t s_mtx = nullptr;
static volatile uint32_t s_last = 0;

void begin() { s_mtx = xSemaphoreCreateMutex(); }
uint32_t lastSendMs() { return s_last; }

static bool fail(char* e, size_t n, const char* m) { strlcpy(e, m, n); return false; }

bool send(const char* text, char* sent, size_t sentLen, char* err, size_t errLen) {
  // Normalise what the user typed/pasted: optional leading '$', optional "*CS" tail, surrounding blanks.
  char body[100]; size_t n = 0;
  while (*text == ' ' || *text == '$') text++;
  for (; *text && *text != '*' && *text != '\r' && *text != '\n'; text++) {
    if (n >= sizeof(body) - 1) return fail(err, errLen, "command too long (max 99 characters)");
    body[n++] = *text;
  }
  while (n && body[n - 1] == ' ') n--;
  body[n] = 0;
  if (n < 5 || strncmp(body, "PQTM", 4)) return fail(err, errLen, "only Quectel PQTM... commands are accepted");
  for (size_t i = 0; i < n; i++) {
    unsigned char c = (unsigned char)body[i];
    if (c < 0x20 || c > 0x7E || c == '$' || c == '"' || c == '\\') return fail(err, errLen, "command contains an illegal character");
  }
  AppConfig cfg; Config::snapshot(cfg);
  if (cfg.gnssTx < 0) return fail(err, errLen, "GNSS TX pin is disabled (-1): wire ESP32 TX to LG290P RX and set the pin");

  uint8_t x = 0; for (size_t i = 0; i < n; i++) x ^= (uint8_t)body[i];
  char line[112];
  int ll = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, x);
  if (ll <= 0 || ll >= (int)sizeof(line)) return fail(err, errLen, "internal formatting error");

  if (!s_mtx || xSemaphoreTake(s_mtx, pdMS_TO_TICKS(500)) != pdTRUE) return fail(err, errLen, "command channel busy, try again");
  uint32_t since = millis() - s_last;
  if (s_last && since < 100) vTaskDelay(pdMS_TO_TICKS(100 - since));      // keep a gap between commands
  size_t w = GNSS_SERIAL.write((const uint8_t*)line, (size_t)ll);
  s_last = millis(); if (!s_last) s_last = 1;
  xSemaphoreGive(s_mtx);
  if (w != (size_t)ll) return fail(err, errLen, "UART write failed");

  Status::g.cmdSent++;
  line[ll - 2] = 0;                                   // strip CR/LF for display
  strlcpy(sent, line, sentLen);
  LOGI("gnss", "sent %s", line);
  return true;
}

}  // namespace GnssCmd
