#pragma once
#include <stdint.h>
// Wi-Fi station manager: own reconnect/backoff logic (Arduino auto-reconnect is disabled),
// escalating recovery: reconnect -> radio restart (10 min down) -> clean reboot (1 h down).
namespace Net {
void begin();            // creates the manager task (core 0)
bool connected();        // lock-free flag, safe from any task
}
