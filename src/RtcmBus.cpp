#include "RtcmBus.h"
#include <string.h>

#ifdef HOST_TEST
  #define TAKE() true
  #define GIVE() do {} while (0)
  #define NOTIFY(h) do {} while (0)
#else
  #include <Arduino.h>
  #include "freertos/FreeRTOS.h"
  #include "freertos/semphr.h"
  #include "freertos/task.h"
  #define TAKE() (xSemaphoreTake((SemaphoreHandle_t)mtx_, pdMS_TO_TICKS(5)) == pdTRUE)
  #define GIVE() xSemaphoreGive((SemaphoreHandle_t)mtx_)
  #define NOTIFY(h) xTaskNotifyGive((TaskHandle_t)(h))
#endif

static const uint32_t MASK = RtcmBus::SIZE - 1;

bool RtcmBus::begin() {
#ifndef HOST_TEST
  mtx_ = xSemaphoreCreateMutex();
  if (!mtx_) return false;
#endif
  head_ = tail_ = 0; last1005Len_ = 0; nWaiters_ = 0;
  return true;
}

uint16_t RtcmBus::peek16(uint32_t a) const { return (uint16_t)(ring_[a & MASK] | (ring_[(a + 1) & MASK] << 8)); }

void RtcmBus::copyIn(uint32_t a, const uint8_t* s, uint16_t n) {
  uint32_t i = a & MASK, first = SIZE - i;
  if (first >= n) { memcpy(ring_ + i, s, n); }
  else { memcpy(ring_ + i, s, first); memcpy(ring_, s + first, n - first); }
}
void RtcmBus::copyOut(uint32_t a, uint8_t* d, uint16_t n) const {
  uint32_t i = a & MASK, first = SIZE - i;
  if (first >= n) { memcpy(d, ring_ + i, n); }
  else { memcpy(d, ring_ + i, first); memcpy(d + first, ring_, n - first); }
}

bool RtcmBus::publish(const uint8_t* f, uint16_t len) {
  uint32_t need = (uint32_t)len + 2;
  if (len < 6 || need > SIZE / 2) return false;
  if (!TAKE()) { lockFail++; return false; }
  while ((uint32_t)(head_ - tail_) + need > SIZE) {          // evict oldest whole frames
    tail_ += (uint32_t)peek16(tail_) + 2;
    overwritten++;
  }
  uint8_t hdr[2] = { (uint8_t)len, (uint8_t)(len >> 8) };
  copyIn(head_, hdr, 2);
  copyIn(head_ + 2, f, len);
  head_ += need;
  if (f[0] == 0xD3 && len <= sizeof(last1005_) && len >= 5 &&
      (uint16_t)(((uint16_t)f[3] << 4) | (f[4] >> 4)) == 1005) {
    memcpy(last1005_, f, len); last1005Len_ = len;
  }
  published++; publishedBytes += len;
  uint8_t n = nWaiters_;
  GIVE();
  for (uint8_t i = 0; i < n; i++) NOTIFY(waiters_[i]);
  return true;
}

void RtcmBus::attach(Cursor& c) {
  if (TAKE()) { c.pos = head_; GIVE(); } else c.pos = head_;
  c.lapped = 0;
}

uint16_t RtcmBus::read(Cursor& c, uint8_t* out, uint16_t maxOut) {
  if (!TAKE()) return 0;
  if ((int32_t)(c.pos - tail_) < 0) { c.pos = tail_; c.lapped++; }   // consumer was overtaken
  uint16_t total = 0;
  while (c.pos != head_) {
    uint16_t l = peek16(c.pos);
    if ((uint32_t)total + l > maxOut) break;
    copyOut(c.pos + 2, out + total, l);
    c.pos += (uint32_t)l + 2;
    total += l;
  }
  GIVE();
  return total;
}

bool RtcmBus::lastReference(uint8_t* out, uint16_t& len) {
  if (!TAKE()) return false;
  bool ok = last1005Len_ > 0;
  if (ok) { memcpy(out, last1005_, last1005Len_); len = last1005Len_; }
  GIVE();
  return ok;
}

void RtcmBus::addWaiter(void* h) {
  if (!TAKE()) return;
  if (nWaiters_ < MAX_WAITERS) waiters_[nWaiters_++] = h;
  GIVE();
}
