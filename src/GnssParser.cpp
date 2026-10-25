#include "GnssParser.h"
#include "Rtcm.h"
#include <string.h>

GnssParser::GnssParser(GnssSink& sink)
  : sink_(sink), state_(S_IDLE), len_(0), expect_(0), pendPos_(0), pendLen_(0) {
  memset(&st_, 0, sizeof(st_));
}

static int hexv(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// "$...*HH\r\n" or "$...*HH\n"
static bool nmeaChecksumOk(const uint8_t* b, uint16_t len) {
  int star = -1;
  for (int i = (int)len - 1; i > 0; --i) if (b[i] == '*') { star = i; break; }
  if (star < 0) return false;
  int tail = (int)len - (star + 3);
  if (tail < 1 || tail > 2) return false;
  int h1 = hexv(b[star + 1]), h2 = hexv(b[star + 2]);
  if (h1 < 0 || h2 < 0) return false;
  uint8_t x = 0;
  for (int i = 1; i < star; i++) x ^= b[i];
  return x == (uint8_t)((h1 << 4) | h2);
}

void GnssParser::feed(const uint8_t* d, size_t n) {
  st_.bytesIn += (uint32_t)n;
  for (size_t i = 0; i < n; i++) { step(d[i]); drain(); }
}

void GnssParser::drain() {
  // resync() may refill pend_ while we are replaying it; the loop condition re-reads the indices.
  while (pendPos_ < pendLen_) { uint8_t c = pend_[pendPos_++]; step(c); }
}

void GnssParser::abortPartial() {
  if (state_ == S_IDLE) return;
  st_.partialTimeouts++;
  resync();
  drain();
}

// Frame failed validation. The first byte can't be a real frame start; everything after it
// might contain one (e.g. a good RTCM frame inside a corrupted one), so it is replayed.
// Invariant: bytes in buf_ + unread bytes in pend_ never exceed FRAME_MAX.
void GnssParser::resync() {
  st_.resyncs++;
  uint16_t tail = (uint16_t)(len_ - 1);
  uint16_t remain = (uint16_t)(pendLen_ - pendPos_);
  if (tail + remain > FRAME_MAX) remain = (uint16_t)(FRAME_MAX - tail);   // defensive
  if (remain) memmove(pend_ + tail, pend_ + pendPos_, remain);
  if (tail) memcpy(pend_, buf_ + 1, tail);
  pendPos_ = 0;
  pendLen_ = (uint16_t)(tail + remain);
  reset();
}

void GnssParser::step(uint8_t b) {
  switch (state_) {
  case S_IDLE:
    if (b == 0xD3) begin(S_RTCM, b);
    else if (b == '$') begin(S_NMEA, b);
    else if (b == 0xB5) begin(S_UBX, b);
    else if (b != '\r' && b != '\n') st_.garbageBytes++;
    return;

  case S_RTCM:
    buf_[len_++] = b;
    if (len_ == 3) {
      if (buf_[1] & 0xFC) { st_.rtcmBadHeader++; resync(); return; }      // 6 reserved bits must be 0
      uint16_t pl = (uint16_t)(((buf_[1] & 3) << 8) | buf_[2]);
      if (pl < 2) { st_.rtcmBadHeader++; resync(); return; }               // need a message number
      expect_ = (uint16_t)(pl + 6);
    } else if (len_ > 3 && len_ == expect_) {
      uint32_t c = Rtcm::crc24q(buf_, (size_t)len_ - 3);
      uint32_t r = ((uint32_t)buf_[len_ - 3] << 16) | ((uint32_t)buf_[len_ - 2] << 8) | buf_[len_ - 1];
      if (c == r) {
        st_.rtcmOk++;
        sink_.onRtcm(buf_, len_, Rtcm::msgNumber(buf_));
        reset();
      } else {
        st_.rtcmCrcErr++;
        resync();
      }
    }
    return;

  case S_NMEA:
    if (b == '\n') {
      buf_[len_++] = b;
      if (nmeaChecksumOk(buf_, len_)) { st_.nmeaOk++; sink_.onNmea((const char*)buf_, len_); reset(); }
      else { st_.nmeaBadChecksum++; resync(); }
    } else if (b == '$' || (b < 0x20 && b != '\r') || b > 0x7E || len_ >= NMEA_MAX - 1) {
      buf_[len_++] = b;
      st_.nmeaAbort++;
      resync();
    } else {
      buf_[len_++] = b;
    }
    return;

  case S_UBX:
    buf_[len_++] = b;
    if (len_ == 2) {
      if (b != 0x62) resync();
    } else if (len_ == 6) {
      uint16_t pl = (uint16_t)(buf_[4] | (buf_[5] << 8));
      if (pl > UBX_MAX_PAYLOAD) { st_.ubxOversize++; resync(); }
      else expect_ = (uint16_t)(pl + 8);
    } else if (len_ > 6 && len_ == expect_) {
      uint8_t a = 0, c = 0;
      for (uint16_t i = 2; i < len_ - 2; i++) { a = (uint8_t)(a + buf_[i]); c = (uint8_t)(c + a); }
      if (a == buf_[len_ - 2] && c == buf_[len_ - 1]) {
        st_.ubxOk++;
        sink_.onUbx(buf_, len_, buf_[2], buf_[3]);
        reset();
      } else { st_.ubxBadChecksum++; resync(); }
    }
    return;
  }
}
