#pragma once
#include <esp_http_server.h>
#include "WatchdogManager.h"
// Firmware upload through the web UI. The Arduino `Update` class writes into the *inactive* OTA slot;
// the boot partition only switches after esp_ota_end() has validated the complete image, so an aborted
// or corrupt upload can never damage the running firmware.
namespace Ota {
esp_err_t handleUpload(httpd_req_t* r);                 // POST /api/ota  (raw .bin body)
void restartSoon(uint32_t delayMs, RestartReason why);  // lets the HTTP response leave first
}
