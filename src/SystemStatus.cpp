#include "SystemStatus.h"
#include <Arduino.h>
#include <string.h>

namespace Status {
Data g;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_rep[6][100]; static uint8_t s_repHead = 0, s_repN = 0;
void pushReply(const char* line, uint16_t len) {
  portENTER_CRITICAL(&s_mux);
  uint16_t m = len < 99 ? len : 99; memcpy(s_rep[s_repHead], line, m); s_rep[s_repHead][m] = 0;
  s_repHead = (uint8_t)((s_repHead + 1) % 6); if (s_repN < 6) s_repN++;
  portEXIT_CRITICAL(&s_mux);
}
uint8_t getReplies(char out[][100], uint8_t maxN) {
  portENTER_CRITICAL(&s_mux);
  uint8_t n = s_repN < maxN ? s_repN : maxN;
  for (uint8_t i = 0; i < n; i++) memcpy(out[i], s_rep[(s_repHead + 6 - 1 - i) % 6], 100);
  portEXIT_CRITICAL(&s_mux);
  return n;
}
void setGga(const Gga& in) { portENTER_CRITICAL(&s_mux); memcpy(&g.gga, &in, sizeof(Gga)); portEXIT_CRITICAL(&s_mux); }
void getGga(Gga& out)      { portENTER_CRITICAL(&s_mux); memcpy(&out, &g.gga, sizeof(Gga)); portEXIT_CRITICAL(&s_mux); }
}
