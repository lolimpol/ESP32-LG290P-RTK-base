#pragma once
#include "Config.h"

// Persistent storage in NVS (Preferences) as ONE versioned blob: [magic|version|bodySize|crc32][AppConfig].
// Writes happen only on explicit Save, and only if the blob actually changed (flash-wear protection).
namespace ConfigStorage {
enum LoadResult : uint8_t { LR_OK, LR_NO_DATA, LR_CORRUPT, LR_MIGRATED, LR_INVALID };
LoadResult  load(AppConfig& out);        // out always receives a usable (validated) config
bool        save(const AppConfig& c);    // verify-after-write
bool        factoryReset();              // erases the namespace
const char* resultStr(LoadResult r);
}
