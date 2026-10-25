#pragma once
// RTCM output bus: ONE shared ring, many independent read cursors (fan-out without copies).
// Frames are stored as [len16 LE][frame bytes]; `tail_` always sits on a frame boundary so a
// consumer that falls behind is moved forward to the oldest intact frame (never mid-frame).
// Single producer (GNSS task); consumers: NTRIP clients, caster slots.
#include <stdint.h>
#include <stddef.h>
#include "UserConfig.h"

class RtcmBus {
public:
  static const uint32_t SIZE = RTCM_BUS_BYTES;
  static const uint8_t  MAX_WAITERS = 8;
  struct Cursor { uint32_t pos; uint32_t lapped; };

  bool     begin();
  bool     publish(const uint8_t* frame, uint16_t len);          // producer only
  void     attach(Cursor& c);                                     // start at "now"
  uint16_t read(Cursor& c, uint8_t* out, uint16_t maxOut);        // whole frames only
  bool     lastReference(uint8_t* out, uint16_t& len);            // cached last 1005 frame
  void     addWaiter(void* taskHandle);                           // task is notified on publish

  volatile uint32_t published = 0, publishedBytes = 0, overwritten = 0, lockFail = 0;

private:
  uint16_t peek16(uint32_t pos) const;
  void copyIn(uint32_t pos, const uint8_t* src, uint16_t n);
  void copyOut(uint32_t pos, uint8_t* dst, uint16_t n) const;

  uint8_t  ring_[SIZE];
  uint32_t head_ = 0, tail_ = 0;
  uint8_t  last1005_[32];
  uint16_t last1005Len_ = 0;
  void*    mtx_ = nullptr;
  void*    waiters_[MAX_WAITERS];
  uint8_t  nWaiters_ = 0;
};
