#pragma once
// HTTP server on the ESP-IDF `esp_http_server` (own task on core 0, bounded sockets, LRU purge,
// header/URI size limits). Chosen over the Arduino WebServer because request bodies are read in
// bounded chunks (no unbounded heap use), OTA streams straight to flash, and it never runs in loop().
namespace WebUi { bool begin(); }
