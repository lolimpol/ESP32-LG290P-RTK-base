#include "ConfigStorage.h"
#include <Preferences.h>
#include <string.h>

namespace ConfigStorage {

static const uint32_t MAGIC = 0x424B5452;   // "RTKB"
static const char* NS = "rtkbase";
static const char* KEY = "cfg";

struct Hdr { uint32_t magic; uint16_t version; uint16_t bodySize; uint32_t crc; };
static const size_t MAX_BLOB = sizeof(Hdr) + sizeof(AppConfig) + 512;   // tolerate newer/larger layouts

static uint32_t crc32(const uint8_t* p, size_t n) {       // IEEE 802.3, bitwise (config only, ~1 KB)
  uint32_t c = 0xFFFFFFFFu;
  while (n--) { c ^= *p++; for (int i = 0; i < 8; i++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(c & 1)); }
  return ~c;
}

const char* resultStr(LoadResult r) {
  switch (r) {
    case LR_OK: return "loaded";
    case LR_NO_DATA: return "no stored config, defaults";
    case LR_CORRUPT: return "stored config corrupt, defaults";
    case LR_MIGRATED: return "stored config migrated";
    default: return "stored config invalid, defaults";
  }
}

LoadResult load(AppConfig& out) {
  Config::setDefaults(out);
  Preferences p;
  if (!p.begin(NS, true)) return LR_NO_DATA;
  size_t len = p.getBytesLength(KEY);
  if (len == 0) { p.end(); return LR_NO_DATA; }
  if (len < sizeof(Hdr) || len > MAX_BLOB) { p.end(); return LR_CORRUPT; }
  uint8_t* blob = (uint8_t*)malloc(len);
  if (!blob) { p.end(); return LR_CORRUPT; }
  size_t got = p.getBytes(KEY, blob, len);
  p.end();
  LoadResult res = LR_OK;
  Hdr h; memcpy(&h, blob, sizeof(h));
  if (got != len || h.magic != MAGIC || (size_t)h.bodySize + sizeof(Hdr) != len ||
      crc32(blob + sizeof(Hdr), h.bodySize) != h.crc) res = LR_CORRUPT;
  else if (h.version > CFG_VERSION) res = LR_CORRUPT;        // written by a newer firmware: don't guess
  else {
    // Migration rule (append-only struct): copy the common prefix, new fields keep their defaults.
    size_t n = h.bodySize < sizeof(AppConfig) ? h.bodySize : sizeof(AppConfig);
    memcpy(&out, blob + sizeof(Hdr), n);
    if (h.version != CFG_VERSION || h.bodySize != sizeof(AppConfig)) res = LR_MIGRATED;
    char err[64];
    if (!Config::validate(out, err, sizeof(err))) res = LR_INVALID;
  }
  free(blob);
  if (res == LR_CORRUPT || res == LR_INVALID) Config::setDefaults(out);
  else Config::updateDerived(out);
  return res;
}

bool save(const AppConfig& c) {
  size_t len = sizeof(Hdr) + sizeof(AppConfig);
  uint8_t* blob = (uint8_t*)malloc(len);
  if (!blob) return false;
  Hdr h = { MAGIC, CFG_VERSION, (uint16_t)sizeof(AppConfig), 0 };
  memcpy(blob + sizeof(Hdr), &c, sizeof(AppConfig));
  h.crc = crc32(blob + sizeof(Hdr), sizeof(AppConfig));
  memcpy(blob, &h, sizeof(h));

  bool ok = false;
  Preferences p;
  if (p.begin(NS, false)) {
    uint8_t* old = (uint8_t*)malloc(len);
    bool same = false;
    if (old) {
      if (p.getBytesLength(KEY) == len && p.getBytes(KEY, old, len) == len) same = !memcmp(old, blob, len);
      free(old);
    }
    if (same) ok = true;                                     // nothing to write
    else if (p.putBytes(KEY, blob, len) == len) {            // NVS is power-fail safe (old value survives)
      uint8_t* chk = (uint8_t*)malloc(len);
      ok = chk && p.getBytes(KEY, chk, len) == len && !memcmp(chk, blob, len);
      free(chk);
    }
    p.end();
  }
  free(blob);
  return ok;
}

bool factoryReset() {
  Preferences p;
  if (!p.begin(NS, false)) return false;
  bool ok = p.clear();
  p.end();
  return ok;
}

}  // namespace ConfigStorage
