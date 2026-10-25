#pragma once
#include <esp_http_server.h>
#include "Config.h"
#include "RtcmBus.h"
// JSON status/config serialisation + form parsing. Read-only on the live system: it only reads
// the Status block, so it cannot affect the GNSS data path. No heap use (chunked streaming).
namespace WebApi {
void      setBus(RtcmBus* b);
esp_err_t status(httpd_req_t* r);
esp_err_t configGet(httpd_req_t* r);
bool      applyForm(char* body, AppConfig& cfg, char* err, size_t errLen);   // body is modified in place
}
