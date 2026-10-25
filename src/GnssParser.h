#pragma once
// Streaming demultiplexer for the LG290P byte stream: RTCM 3.x, NMEA 0183, UBX.
// No Arduino dependencies (host-testable). Callbacks run inline; the buffers passed
// to them are only valid for the duration of the call.
#include <stdint.h>
#include <stddef.h>

class GnssSink {
public:
  virtual ~GnssSink() {}
  virtual void onRtcm(const uint8_t* frame, uint16_t len, uint16_t msgNum) = 0;  // CRC-verified
  virtual void onNmea(const char* line, uint16_t len) = 0;                        // checksum-verified, incl. CR/LF
  virtual void onUbx(const uint8_t* frame, uint16_t len, uint8_t cls, uint8_t id) = 0; // Fletcher-verified
};

struct ParserStats {
  uint32_t bytesIn, rtcmOk, rtcmCrcErr, rtcmBadHeader;
  uint32_t nmeaOk, nmeaBadChecksum, nmeaAbort;
  uint32_t ubxOk, ubxBadChecksum, ubxOversize;
  uint32_t garbageBytes, resyncs, partialTimeouts;
};

class GnssParser {
public:
  static const uint16_t UBX_MAX_PAYLOAD = 2048;
  static const uint16_t NMEA_MAX = 300;
  static const uint16_t FRAME_MAX = UBX_MAX_PAYLOAD + 8;   // >= RTCM max (1029)

  explicit GnssParser(GnssSink& sink);
  void feed(const uint8_t* d, size_t n);
  void abortPartial();                       // call when the line went idle mid-frame
  bool midFrame() const { return state_ != S_IDLE; }
  const ParserStats& stats() const { return st_; }

private:
  enum State : uint8_t { S_IDLE, S_RTCM, S_NMEA, S_UBX };
  void step(uint8_t b);
  void begin(State s, uint8_t b) { state_ = s; buf_[0] = b; len_ = 1; expect_ = 0; }
  void reset() { state_ = S_IDLE; len_ = 0; expect_ = 0; }
  void resync();                             // drop first byte of the current frame, replay the rest
  void drain();

  GnssSink& sink_;
  ParserStats st_;
  State state_;
  uint16_t len_, expect_;
  uint16_t pendPos_, pendLen_;
  uint8_t buf_[FRAME_MAX];
  uint8_t pend_[FRAME_MAX];
};
