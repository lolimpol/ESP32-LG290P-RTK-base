#pragma once
// Command channel ESP32 TX -> LG290P RX. Only Quectel PQTM* proprietary NMEA commands are accepted
// (e.g. "PQTMCFGMSGRATE,W,RMC,1", "PQTMSAVEPAR"). The caller supplies the text between '$' and '*';
// '$', the XOR checksum and CR/LF are added here. Replies are captured by GnssRouter (see Status::getReplies).
#include <stddef.h>
#include <stdint.h>
namespace GnssCmd {
void     begin();
bool     send(const char* text, char* sentLine, size_t sentLen, char* err, size_t errLen);
uint32_t lastSendMs();      // 0 = nothing sent yet
}
