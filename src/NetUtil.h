#pragma once
// Non-blocking socket helpers on raw lwIP fds. WiFiClient::write() can block for seconds on a stalled
// peer; the caster multiplexes many clients in one task, so it must never use it.
#include <lwip/sockets.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>

// >=0 bytes sent (0 = would block), -1 = fatal error
static inline int sockSend(int fd, const uint8_t* p, size_t n) {
  int r = ::send(fd, (const void*)p, n, MSG_DONTWAIT);
  if (r >= 0) return r;
  return (errno == EWOULDBLOCK || errno == EAGAIN) ? 0 : -1;
}
// >0 bytes, 0 = nothing pending, -1 = peer closed / error
static inline int sockRecv(int fd, uint8_t* p, size_t n) {
  int r = ::recv(fd, (void*)p, n, MSG_DONTWAIT);
  if (r > 0) return r;
  if (r == 0) return -1;
  return (errno == EWOULDBLOCK || errno == EAGAIN) ? 0 : -1;
}
// Detect dead peers (cable pulled / NAT timeout) in ~30 s.
static inline void sockKeepAlive(int fd) {
  int v = 1; setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &v, sizeof v);
  v = 15; setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &v, sizeof v);
  v = 5;  setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &v, sizeof v);
  v = 3;  setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &v, sizeof v);
}
