#include "WebUi.h"
#include "WebApi.h"
#include "WebAssets.h"          // generated from /data by tools/embed_web.py (gzip'd)
#include "OtaManager.h"
#include "GnssCmd.h"
#include "Config.h"
#include "ConfigStorage.h"
#include "LogManager.h"
#include "WatchdogManager.h"
#include <esp_http_server.h>
#include <mbedtls/base64.h>
#include <Arduino.h>

static const size_t BODY_MAX = 3072;

static bool ctEq(const char* a, const char* b) {
  size_t la = strlen(a), lb = strlen(b); uint8_t d = (uint8_t)(la ^ lb);
  for (size_t i = 0; i < la && i < lb; i++) d |= (uint8_t)(a[i] ^ b[i]);
  return d == 0;
}

// HTTP Basic auth. NOTE: plain HTTP - credentials are base64, not encrypted (see README, Security).
static bool authOk(httpd_req_t* r) {
  char h[128];
  if (httpd_req_get_hdr_value_str(r, "Authorization", h, sizeof(h)) != ESP_OK || strncmp(h, "Basic ", 6)) return false;
  unsigned char dec[96]; size_t ol = 0;
  if (mbedtls_base64_decode(dec, sizeof(dec) - 1, &ol, (const unsigned char*)h + 6, strlen(h + 6))) return false;
  dec[ol] = 0;
  char* col = strchr((char*)dec, ':'); if (!col) return false; *col = 0;
  char u[24], p[32]; Config::getWebCreds(u, sizeof(u), p, sizeof(p));
  return ctEq((char*)dec, u) & ctEq(col + 1, p);
}
static esp_err_t deny(httpd_req_t* r) {
  httpd_resp_set_status(r, "401 Unauthorized");
  httpd_resp_set_hdr(r, "WWW-Authenticate", "Basic realm=\"RTK Base\"");
  httpd_resp_send(r, "Unauthorized", HTTPD_RESP_USE_STRLEN);
  vTaskDelay(pdMS_TO_TICKS(250));        // throttle password guessing
  return ESP_OK;
}
#define AUTH(r) do { if (!authOk(r)) return deny(r); } while (0)

static esp_err_t asset(httpd_req_t* r, const uint8_t* d, size_t n, const char* type) {
  AUTH(r);
  httpd_resp_set_type(r, type);
  httpd_resp_set_hdr(r, "Content-Encoding", "gzip");
  httpd_resp_set_hdr(r, "Cache-Control", "no-cache");
  return httpd_resp_send(r, (const char*)d, n);
}
static esp_err_t hIndex(httpd_req_t* r) { return asset(r, kIndexGz, kIndexGzLen, "text/html"); }
static esp_err_t hCss(httpd_req_t* r)   { return asset(r, kStyleGz, kStyleGzLen, "text/css"); }
static esp_err_t hJs(httpd_req_t* r)    { return asset(r, kAppGz, kAppGzLen, "application/javascript"); }
static esp_err_t hStatus(httpd_req_t* r){ AUTH(r); return WebApi::status(r); }
static esp_err_t hCfgGet(httpd_req_t* r){ AUTH(r); return WebApi::configGet(r); }

static esp_err_t json(httpd_req_t* r, const char* status, const char* body) {
  httpd_resp_set_status(r, status); httpd_resp_set_type(r, "application/json");
  return httpd_resp_send(r, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t hCfgPost(httpd_req_t* r) {
  AUTH(r);
  size_t len = r->content_len;
  if (len == 0 || len > BODY_MAX) { httpd_resp_set_status(r, "413 Payload Too Large"); httpd_resp_send(r, "too large", HTTPD_RESP_USE_STRLEN); return ESP_FAIL; }
  static char body[BODY_MAX + 1];       // single httpd task => safe to share
  size_t got = 0;
  while (got < len) {
    int n = httpd_req_recv(r, body + got, len - got);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (n <= 0) return ESP_FAIL;
    got += n;
  }
  body[len] = 0;
  char q[24] = "", sv[4] = "";
  httpd_req_get_url_query_str(r, q, sizeof(q)); httpd_query_key_value(q, "save", sv, sizeof(sv));

  AppConfig oldc, c; Config::snapshot(oldc); c = oldc;
  char err[96] = "", out[200];
  if (!WebApi::applyForm(body, c, err, sizeof(err)) || !Config::commit(c, err, sizeof(err))) {
    snprintf(out, sizeof(out), "{\"ok\":false,\"error\":\"%s\"}", err);
    return json(r, "400 Bad Request", out);
  }
  bool saved = false;
  if (sv[0] == '1') { AppConfig live; Config::snapshot(live); saved = ConfigStorage::save(live); }
  snprintf(out, sizeof(out), "{\"ok\":true,\"saved\":%s,\"saveRequested\":%s,\"restart\":%s}", saved ? "true" : "false",
           sv[0] == '1' ? "true" : "false", Config::restartRequired(oldc, c) ? "true" : "false");
  LOGI("web", "configuration applied%s", saved ? " and saved" : "");
  return json(r, "200 OK", out);
}

static esp_err_t hSave(httpd_req_t* r) {
  AUTH(r); AppConfig c; Config::snapshot(c);
  return ConfigStorage::save(c) ? json(r, "200 OK", "{\"ok\":true}") : json(r, "500 Internal Server Error", "{\"ok\":false,\"error\":\"flash write failed\"}");
}
static esp_err_t hRestart(httpd_req_t* r) { AUTH(r); json(r, "200 OK", "{\"ok\":true}"); Ota::restartSoon(800, RR_USER); return ESP_OK; }
static esp_err_t hFactory(httpd_req_t* r) {
  AUTH(r);
  bool ok = ConfigStorage::factoryReset();
  json(r, ok ? "200 OK" : "500 Internal Server Error", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"erase failed\"}");
  if (ok) Ota::restartSoon(800, RR_USER);
  return ESP_OK;
}
static esp_err_t hGnssCmd(httpd_req_t* r) {
  AUTH(r);
  size_t len = r->content_len;
  if (len == 0 || len > 200) { httpd_resp_set_status(r, "413 Payload Too Large"); httpd_resp_send(r, "bad size", HTTPD_RESP_USE_STRLEN); return ESP_FAIL; }
  char body[208]; size_t got = 0;
  while (got < len) {
    int n = httpd_req_recv(r, body + got, len - got);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (n <= 0) return ESP_FAIL;
    got += n;
  }
  body[len] = 0;
  char sent[112] = "", err[100] = "", out[360];
  if (!GnssCmd::send(body, sent, sizeof(sent), err, sizeof(err))) {
    snprintf(out, sizeof(out), "{\"ok\":false,\"error\":\"%s\"}", err);
    return json(r, "400 Bad Request", out);
  }
  snprintf(out, sizeof(out), "{\"ok\":true,\"sent\":\"%s\"}", sent);
  return json(r, "200 OK", out);
}
static esp_err_t hOta(httpd_req_t* r) { AUTH(r); return Ota::handleUpload(r); }

bool WebUi::begin() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.core_id = 0; cfg.task_priority = 3; cfg.stack_size = 10240;
  cfg.max_open_sockets = HTTPD_OPEN_SOCKETS; cfg.max_uri_handlers = 16;
  cfg.lru_purge_enable = true; cfg.recv_wait_timeout = 10; cfg.send_wait_timeout = 10;
  httpd_handle_t h = nullptr;
  if (httpd_start(&h, &cfg) != ESP_OK) { LOGE("web", "httpd_start failed"); return false; }
  static const httpd_uri_t routes[] = {
    { "/",           HTTP_GET,  hIndex,   nullptr }, { "/style.css",  HTTP_GET,  hCss,     nullptr },
    { "/app.js",     HTTP_GET,  hJs,      nullptr }, { "/api/status", HTTP_GET,  hStatus,  nullptr },
    { "/api/config", HTTP_GET,  hCfgGet,  nullptr }, { "/api/config", HTTP_POST, hCfgPost, nullptr },
    { "/api/save",   HTTP_POST, hSave,    nullptr }, { "/api/restart",HTTP_POST, hRestart, nullptr },
    { "/api/factory",HTTP_POST, hFactory, nullptr }, { "/api/ota",    HTTP_POST, hOta,     nullptr },
    { "/api/gnsscmd",HTTP_POST, hGnssCmd, nullptr },
  };
  for (const auto& u : routes) httpd_register_uri_handler(h, &u);
  LOGI("web", "HTTP server on port 80");
  return true;
}
