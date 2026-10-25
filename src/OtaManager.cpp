#include "OtaManager.h"
#include "LogManager.h"
#include <Update.h>
#include <Arduino.h>

namespace Ota {

static uint32_t s_delay; static RestartReason s_why;
static void restartTask(void*) { vTaskDelay(pdMS_TO_TICKS(s_delay)); Watchdog::restartDevice(s_why); vTaskDelete(nullptr); }
void restartSoon(uint32_t ms, RestartReason why) { s_delay = ms; s_why = why; xTaskCreate(restartTask, "restart", 3072, nullptr, 1, nullptr); }

static esp_err_t fail(httpd_req_t* r, const char* msg) {
  Update.abort();
  LOGE("ota", "upload failed: %s", msg);
  httpd_resp_set_status(r, "400 Bad Request");
  httpd_resp_set_type(r, "text/plain");
  httpd_resp_send(r, msg, HTTPD_RESP_USE_STRLEN);
  return ESP_FAIL;                      // closes the connection; any unread body is discarded
}

esp_err_t handleUpload(httpd_req_t* r) {
  static uint8_t buf[2048];             // httpd serves one request at a time
  size_t total = r->content_len;
  if (total < 100000) return fail(r, "file too small to be a firmware image");
  if (!Update.begin(total, U_FLASH)) return fail(r, Update.errorString());
  LOGI("ota", "upload started, %u bytes", (unsigned)total);
  size_t got = 0; int timeouts = 0; bool first = true;
  while (got < total) {
    size_t want = total - got < sizeof(buf) ? total - got : sizeof(buf);
    int n = httpd_req_recv(r, (char*)buf, want);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) { if (++timeouts > 10) return fail(r, "upload stalled"); continue; }
    if (n <= 0) return fail(r, "connection lost during upload");
    timeouts = 0;
    if (first) { if (buf[0] != 0xE9) return fail(r, "not an ESP32 firmware image (bad magic byte)"); first = false; }
    if (Update.write(buf, n) != (size_t)n) return fail(r, Update.errorString());
    got += n;
  }
  if (!Update.end(true)) return fail(r, Update.errorString());   // validates image before switching slot
  LOGI("ota", "upload OK, rebooting");
  httpd_resp_set_type(r, "application/json");
  httpd_resp_send(r, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
  restartSoon(1500, RR_OTA);
  return ESP_OK;
}

}  // namespace Ota
