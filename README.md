# ESP32 RTK Base Station for the Quectel LG290P

Firmware that turns an **ESP32 + Quectel LG290P** into an unattended RTK **base station appliance**: it reads the receiver's RTCM 3.x and NMEA output over UART, generates the missing **RTCM 1005** reference-position message itself, and distributes the corrections to **two remote NTRIP casters** and a **local NTRIP caster**, and serves **GNSS-derived time over NTP**. A small web UI handles monitoring, configuration, receiver commands and OTA updates.

Written for the Arduino framework (arduino-esp32 2.0.x, built with PlatformIO). Original code; Vibe coded as hell.

---

## Table of contents
1. [Features](#1-features)
2. [Hardware](#2-hardware)
3. [Quick start](#3-quick-start)
4. [Architecture](#4-architecture)
5. [Configuration](#5-configuration)
6. [Web UI](#6-web-ui)
7. [REST API](#7-rest-api)
8. [RTCM handling](#8-rtcm-handling)
9. [NTRIP upload services](#9-ntrip-upload-services-two-instances)
10. [Local NTRIP caster](#10-local-ntrip-caster)
11. [NTP server and GNSS time](#11-ntp-server-and-gnss-time)
12. [Talking to the LG290P](#12-talking-to-the-lg290p)
13. [USB / PPP-PPK stream](#13-usb--pppppk-stream)
14. [Serial console](#14-serial-console)
15. [Persistence](#15-persistence)
16. [OTA updates](#16-ota-updates)
17. [Reliability design](#17-reliability-design)
18. [Resource budget](#18-resource-budget)
19. [Security](#19-security)
20. [Testing](#20-testing)
21. [Troubleshooting](#21-troubleshooting)
22. [Known limitations](#22-known-limitations)
23. [Repository layout](#23-repository-layout)
24. [Contributing / license](#24-contributing--license)

---

## 1. Features

| Area | What it does |
|---|---|
| **GNSS input** | LG290P over a dedicated hardware UART (default 460 800 Bd), 8 KB ISR-fed RX ring, parser task pinned to core 1 |
| **Stream parser** | RTCM 3.x (CRC-24Q verified), NMEA 0183 (checksum verified), UBX framing (Fletcher verified); corrupt frames dropped, sync recovered by *replaying* the bytes after a bad frame start |
| **RTCM 1005** | Generated on the ESP32 from your configured ECEF X/Y/Z, published immediately on change and every N s (default 5); cached so new rovers get it on connect |
| **RTCM routing** | Configurable catalogue of 18 messages; one shared ring buffer with per-consumer cursors (no per-client copies) |
| **2× NTRIP uploads** | Independent tasks pushing to remote casters (NTRIP Rev1 `SOURCE` / Rev2 `POST`), optional TLS, exponential backoff, resilient DNS |
| **Local NTRIP caster** | One mountpoint, Basic auth, sourcetable, non-blocking sockets, stalled rovers dropped without affecting others |
| **NTP server** | Stratum 1 (GNSS) from NMEA RMC, optional 1PPS input for ~ms accuracy, timestamps independent of the ESP32 system clock |
| **LG290P command channel** | Send `PQTM…` commands (e.g. enable RMC) from the web UI or serial console; auto-checksum; replies shown |
| **USB log/stream** | Select NMEA sentences / UBX messages to stream raw over the serial port for PPP/PPK logging |
| **Web UI** | Dashboard, settings, receiver commands, OTA — ~7 KB gzip, no frameworks, no maps |
| **Persistence** | Versioned, CRC-protected NVS blob, explicit Save, migration, factory reset |
| **OTA** | Upload `firmware.bin` in the browser; written to the inactive slot, validated before switching |
| **Self-healing** | Per-task heartbeats + ESP-IDF task watchdog, Wi-Fi escalation ladder, UART silence recovery, heap guard, persistent reset counters |

Deliberately **not** included: maps, Bluetooth/BLE, Ethernet, cloud telemetry, antenna calibration, receiving GNSS data over the network, JS frameworks, HTTPS.

---

## 2. Hardware

* **ESP32** (classic, e.g. ESP32 DevKit, 4 MB flash). Other variants may work, but pin validation, the partition table and the USB notes assume the classic ESP32.
* **Quectel LG290P** (or a board carrying it) with a good antenna and clear sky view.
* Wiring (3.3 V logic, common GND):

| LG290P | ESP32 (default) | Purpose |
|---|---|---|
| TX | GPIO 16 (RX2) | GNSS data → ESP32 (**required**) |
| RX | GPIO 17 (TX2) | ESP32 → receiver, for the command channel (§12). Optional unless you want to send commands |
| 1PPS (optional) | `GNSS_PPS_PIN` (default off) | Precise NTP second edge |
| GND | GND | |

Pins are configurable in the web UI (a pin change needs a restart). Pins 6–11 (flash), 1/3 (console) and input-only pins as TX are rejected.

---

## 3. Quick start

1. **Install** [PlatformIO](https://platformio.org/) (CLI or VS Code).
2. **Edit `src/UserConfig.h`** — at minimum Wi-Fi SSID/password, web password, and your base **ECEF X/Y/Z** (metres). These are *factory defaults*, used on first boot and after a factory reset.
3. **Build & flash** the first time over USB:
   ```bash
   pio run -t upload
   pio device monitor        # 115200 Bd
   ```
4. In the console type `status` to find the IP (or `wifi <ssid> <pass>` then `save` if Wi-Fi failed).
5. Open `http://<ip>/`, log in (default `admin` / `admin` — **change it**).
6. **Settings** → check the base position, enable your NTRIP upload(s), **Apply & Save**.
7. **Receiver** tab → enable the messages you need on the LG290P (§12) and **make sure observation messages (MSM) are enabled** — see §8.

Web files in `data/` are gzip-embedded into `src/WebAssets.h` automatically before every build (`tools/embed_web.py`); there is no filesystem upload step.

**Arduino IDE:** install *esp32 by Espressif 2.0.17*; copy `src/*` flat into a sketch folder; rename `main.cpp` → `<sketch>.ino`; board *ESP32 Dev Module*, CPU 160 MHz, partition *Minimal SPIFFS (1.9 MB APP with OTA)*; run `python3 tools/embed_web.py` after editing `data/`.

---

## 4. Architecture

```
LG290P TX ─UART2─▶ [driver RX ring 8 KB, ISR-fed]
                          │ consumer
              ┌───────────▼───────────┐  core 1 · prio 6 · "gnss" task
              │ GnssParser            │  RTCM (CRC24Q) | NMEA (XOR) | UBX (Fletcher)
              └───────┬───────────────┘
                      ▼
              GnssRouter ─ NMEA: counters · last GGA · RMC→GnssTime · selected lines→log stream
                      │    $PQTM… replies → command-reply ring
                      │    RTCM: catalogue mask · drop upstream 1005/1006 if ESP32 generates 1005
                      │    + ESP32-generated RTCM 1005 every N s
                      ▼
              RtcmBus — ONE 16 KB ring, frames [len16|bytes], per-consumer cursors, task-notify
        ┌─────────────┼────────────────┐
  NtripClient #1  NtripClient #2   NtripCaster (≤2 rovers)      core 0 · prio 3
  (own task)      (own task)       (one non-blocking task)
```

Side services (never on the data path): `NtpServer`, `NetworkManager`, web server (`esp_http_server`, core 0), `LogManager` (console + USB stream, core 1), `loop()` housekeeping (core 1, lowest priority).

**Core allocation.** Wi-Fi/lwIP run on core 0 in arduino-esp32, so everything network-facing lives there; the timing-critical GNSS parser owns core 1 at the highest application priority. Only the bus (a ≤1 KB memcpy under a mutex with a 5 ms bound) and task notifications cross cores. Mutexes are never held across network calls.

**Producer/consumer.** UART ISR → driver ring → GNSS task (reads ≤512 B chunks) → parser → router → bus / log stream. The GNSS task never touches Wi-Fi, HTTP, NVS, the filesystem, or blocking serial output.

**Shared state.** All status flows through one `Status::g` block with **one writer per field**; the web UI only reads, so it cannot stall the receive path.

### Modules

| Module | Responsibility |
|---|---|
| `main.cpp` | Wiring, 1 Hz housekeeping (rates, supervision) |
| `UserConfig.h` | All factory defaults and compile-time limits |
| `Config` / `ConfigStorage` | Validated runtime config, generation counter, NVS persistence |
| `GnssUart` | UART ownership, GNSS task, silence recovery |
| `GnssParser` | Stream demux/framing/validation/resync (hardware-independent) |
| `GnssRouter` | RTCM filtering, 1005 generation, NMEA handling, reply capture |
| `Rtcm`, `Rtcm1005` | CRC-24Q, catalogue, 1005 encoder, ECEF/geodetic maths (hardware-independent) |
| `RtcmBus` | Shared ring + cursors (hardware-independent apart from mutex/notify) |
| `NtripClient` | Outbound NTRIP source service |
| `NtripCaster` | Local caster |
| `GnssTime`, `NtpServer` | GNSS time base, NTP responder |
| `GnssCmd` | LG290P command channel |
| `WebUi`, `WebApi`, `OtaManager` | HTTP server, JSON/forms, OTA |
| `LogManager` | Diagnostics, serial console, USB stream |
| `WatchdogManager`, `NetworkManager`, `SystemStatus`, `NetUtil` | Supervision, Wi-Fi manager, shared status, socket helpers |

---

## 5. Configuration

Two layers:

1. **`src/UserConfig.h`** — compile-time factory defaults (Wi-Fi, passwords, pins, base position, NTRIP presets, caster, NTP, logging) and resource limits. Used on first boot / after factory reset.
2. **Web UI → Settings** — live configuration, stored in NVS.

**Apply** validates and applies live (no reboot; only a GNSS UART *pin* change needs a restart). **Apply & Save** also writes to flash. Unsaved changes are lost at reboot.

### Base position (important)

Entered as **ECEF Cartesian X/Y/Z in metres** (0.1 mm resolution) of the **antenna reference point (ARP)**, in the frame/epoch your rovers expect (e.g. ITRF/ETRS89 coordinates of your survey or CORS). Latitude/longitude are *not* used as input (they are derived internally only for the caster sourcetable). Points not near the Earth's surface (all zeros, typos) are rejected. Station ID (0–4095), ITRF year code and GPS/GLONASS/Galileo indicators are part of RTCM 1005 and are configurable.

> If you only have lat/lon/height, convert with a proper tool first (geodetic latitude, **ellipsoidal** height, WGS-84) — the firmware does not convert for input.

### Settings reference

| Section | Fields |
|---|---|
| Web login & network | web user/password, hostname, Wi-Fi SSID/password |
| Base position | ECEF X/Y/Z, station ID, ITRF year, GPS/GLO/GAL indicators, generate-1005 on/off, 1005 interval (1–600 s) |
| GNSS UART | baud, RX pin, TX pin (−1 = none) |
| NTRIP upload 1 / 2 | enabled, host, port, mountpoint, version (1/2), user, password, TLS, reconnect interval |
| NTRIP caster | enabled, port, mountpoint, user, password, max clients, stall timeout |
| NTP | enabled, UDP port, NMEA latency compensation (ms) |
| USB / logging | mode (console / raw stream), stream baud, log level, selected stream messages |
| RTCM messages | one checkbox per catalogue message |

**Passwords are displayed in clear text in the settings panel** (by design choice) — see [Security](#19-security).

---

## 6. Web UI

* **Dashboard** (auto-refresh 2 s, with an "updated hh:mm:ss" stamp): system (heap, uptime, CPU, reset reason, per-task stack/heartbeat, reset counters), Wi-Fi, GNSS/UART/parser/NMEA counters/last GGA, RTCM table (source, rx/gen/fwd, msgs/s, B/s, age), both NTRIP uploads, caster, NTP (incl. *why* time isn't valid), USB/log streams, and an **Attention** card for factory defaults / silent UART.
* **Settings** — all configuration above.
* **Receiver** — LG290P command box, quick buttons, receiver replies.
* **Firmware** — OTA upload with progress.

Responsive layout, light/dark via `prefers-color-scheme`, no external assets.

---

## 7. REST API

All endpoints require HTTP Basic auth. Bodies are small and size-capped.

| Method & path | Purpose |
|---|---|
| `GET /` , `/style.css`, `/app.js` | UI (gzip, embedded) |
| `GET /api/status` | Full live status as JSON (chunked, no heap use) |
| `GET /api/config` | Current configuration as flat JSON (keys such as `wifi_ssid`, `base_x`, `n1_host`, `rt_1077`, `lg_GGA`) |
| `POST /api/config?save=0\|1` | `application/x-www-form-urlencoded` body; validated, applied live; `save=1` also persists. Returns `{ok, saved, saveRequested, restart}` or `{ok:false, error}` |
| `POST /api/save` | Persist current live config |
| `POST /api/gnsscmd` | `text/plain` body = LG290P command text, e.g. `PQTMCFGMSGRATE,W,RMC,1` |
| `POST /api/ota` | Raw `.bin` body |
| `POST /api/restart`, `POST /api/factory` | Restart / erase saved config and restart |

Example:
```bash
curl -u admin:admin http://<ip>/api/status
curl -u admin:admin -d 'n1_en=1&n1_host=servers.onocoy.com&n1_mount=MYSTATION' \
     'http://<ip>/api/config?save=1'
```

---

## 8. RTCM handling

### Data origin — keep these distinct
| | Source |
|---|---|
| 1005 | **Generated by the ESP32** (the LG290P is not expected to send it). If it ever sends 1005/1006 while generation is on, it is dropped to avoid conflicting positions. |
| everything else | **Supplied by the LG290P.** Ticking a message in the UI only *forwards* it **if the receiver actually sends it**; it never makes the receiver output it. |

### Catalogue (bit order = config mask bit)
1005 (ESP32), 1019 GPS eph, 1020 GLONASS eph, 1042 BeiDou eph, 1045 Galileo F/NAV eph, 1046 Galileo I/NAV eph, 1033 receiver/antenna descriptors, 1230 GLONASS biases, MSM4: 1074/1084/1094/1124, MSM7: 1077/1087/1097/1127, 1117 (QZSS MSM7), 1137 (NavIC MSM7).
New messages are appended at the end so saved masks stay valid. Adding one = one line in `src/Rtcm.cpp`.

> **A base needs observations.** Ephemerides (1019/1020/1042/1046) alone give a rover nothing to solve. Enable MSM observation messages (e.g. 1077/1087/1097/1127, plus 1117/1137 and 1230 if needed — Onocoy asks for 1005/1077/1087/1097/1117/1127/1137/1230) **on the LG290P itself**.

### Parser behaviour
* **RTCM:** preamble `0xD3`, 6 reserved bits must be zero, length, CRC-24Q (poly `0x1864CFB`; check value of `"123456789"` = `0xCDE703`). A frame failing CRC is dropped **and the bytes after its first byte are re-scanned**, so a valid frame hidden inside a corrupted one is still found. A frame stalled mid-way is abandoned after a baud-rate-dependent timeout.
* **NMEA:** XOR checksum verified; binary or `$` inside a sentence aborts and resyncs. Only GGA and RMC are parsed (fields on demand); everything else is classified by its 3-letter ID and counted.
* **UBX:** framed and checksummed so RXM-RAWX/SFRBX can be forwarded to the log stream **if ever received** (the LG290P does not normally emit UBX); payloads > 2048 B are counted as oversize and dropped.

### RTCM 1005 encoding
Payload = 152 bits (19 bytes), MSB first: DF002 msg no. (12) · DF003 station ID (12) · DF021 ITRF year (6) · DF022/23/24 GPS/GLO/GAL indicators (1+1+1) · DF141 reference-station indicator (1; 0 = physical) · DF025 ECEF-X (38, signed, 0.0001 m) · DF142 single-oscillator (1) · reserved (1) · DF026 ECEF-Y (38) · DF364 quarter-cycle (2) · DF027 ECEF-Z (38). Frame = `D3 00 13` + payload + CRC-24Q = 25 bytes. 1005 carries the **ARP** (antenna height above marker is 1006 and is not implemented); BeiDou has no indicator bit in 1005.

---

## 9. NTRIP upload services (two instances)

**Terminology:** a base station that *pushes* corrections to a caster takes the NTRIP **Server (source)** role. These are the firmware's "NTRIP client services" (they dial out) speaking:

* **Rev1:** `SOURCE <password> /<mount>` → `ICY 200 OK` (user name unused)
* **Rev2:** `POST /<mount> HTTP/1.1` + `Ntrip-Version: Ntrip/2.0` + Basic auth → `HTTP/1.1 200 OK`, then raw RTCM

No GGA is sent (GGA is a rover→caster message and would corrupt an uploaded RTCM stream).

Each instance has its **own task**, so a hung DNS/TLS/TCP operation in one never affects the other or the GNSS path. Features: DNS → connect (8 s cap) → login → stream from a private bus cursor attached at "now" (no stale replay); exponential backoff (base interval ×2ⁿ, cap 300 s, ±10 % jitter, reset after 30 s healthy); per-service rx/tx bytes, reconnects, failures, last error, last-OK time.

**Resilient DNS:** DHCP resolver (twice) → public 1.1.1.1/8.8.8.8 for that lookup (DHCP servers restored) → last known address. The console command `dns <host>` shows the resolvers and tests a lookup.

**TLS** encrypts the link but **does not verify the server certificate**.

**Onocoy example** (check their docs for your station): host `servers.onocoy.com`, port `2101`. NTRIP v2: your credential name as *user* + password. NTRIP v1: credential name in the *mountpoint* field + password.

---

## 10. Local NTRIP caster

* `GET /` → sourcetable (`STR;…`, `ENDSOURCETABLE`); `GET /<mount>` → Basic auth (constant-time compare; empty user **and** password = open) → `ICY 200 OK` (Rev1) or `HTTP/1.1 200 OK` (Rev2), then raw RTCM.
* One task multiplexes all rovers over **non-blocking** `send(MSG_DONTWAIT)`; each rover owns only a bus cursor + a ~2 KB staging buffer.
* A rover with no TCP progress for the configured timeout (default 15 s) is dropped; handshake limit 5 s; header limit 512 B; TCP keep-alive on every socket.
* New rovers immediately receive the cached 1005.
* **Max 2 rovers** on the stock lwIP (see §18).

---

## 11. NTP server and GNSS time

**The only message NTP relies on is NMEA RMC** (not RTCM, not GGA). An RMC is accepted only if status = `A` (valid), time and date fields are present, and the year is ≥ 2024. If the LG290P isn't sending RMC (note: Quectel's base-station mode turns NMEA output off automatically), enable it with `PQTMCFGMSGRATE,W,RMC,1` (§12).

**Time base:** `unix_us = esp_timer_us + offset`, with `offset` re-estimated from each accepted RMC (step if >200 ms off, otherwise EMA-slewed) or from the optional **1PPS** edge. The libc clock is set once (re-stepped only if >2 s wrong) purely for log/UI timestamps; NTP never reads it.

**Served state:** *locked* (RMC < 5 s old, stratum 1, ref-id `GPS`), *holdover* (< 120 s), otherwise LI=3 / stratum 16 (unsynchronised). Only mode-3 requests are answered; global limit 50 req/s.

**Realistic accuracy:** ~10–100 ms without PPS (NMEA latency + Wi-Fi jitter; calibrate with the latency-compensation setting), ~1–5 ms with PPS. This is a convenient LAN time source, not a lab reference.

The dashboard's NTP card shows `rmcReceived`, `rmcNoFix`, `rmcRejected`, `lastRmcStatus` and a plain-language **`why`** when time is invalid.

---

## 12. Talking to the LG290P

Wire **ESP32 TX → LG290P RX** and make sure the TX pin isn't −1. Use the **Receiver** tab or the console command `gnss <command>`.

* You enter the text between `$` and `*`; `$`, the XOR checksum and CR/LF are added automatically (the routine matches the examples in Quectel's protocol specification).
* Only Quectel **`PQTM…`** commands are accepted; ≥100 ms spacing is enforced.
* Replies (`$PQTMCFGMSGRATE,OK*29`, `…,ERROR,<code>`) are captured and shown, newest first.

Useful commands (see Quectel's *LG290P/LGx80P GNSS Protocol Specification* for the full list):

| Command | Effect |
|---|---|
| `PQTMCFGMSGRATE,W,RMC,1` | Output RMC every fix (needed for NTP) |
| `PQTMCFGMSGRATE,W,GGA,1` | Output GGA every fix |
| `PQTMCFGMSGRATE,R,RMC` | Read the RMC rate |
| `PQTMVERNO` | Firmware version |
| `PQTMSAVEPAR` | Persist settings in the receiver (restart it afterwards, per Quectel) |

⚠️ `PQTMCFGUART` (baud) and `PQTMCFGPROT` (protocols) can cut the link. If you change the receiver's baud rate, change the UART baud in Settings to match.

---

## 13. USB / PPP–PPK stream

Two modes on the console UART (UART0 / USB-serial bridge on a classic ESP32; native USB CDC on S2/S3/C3 with "USB CDC on boot"):

* **Diagnostic console** (default): human-readable log + CLI at 115 200 Bd.
* **Raw GNSS stream:** forwards the selected messages verbatim (diagnostics muted so text cannot corrupt a binary stream; CLI input still works, `stream off` returns).

Selectable messages: NMEA **GGA, SGA, GST, GSV, RMC, GSA**, UBX **RXM-RAWX, RXM-SFRBX**. "SGA" is not a standard NMEA 0183 sentence — it is matched by its 3-letter ID and forwarded only if received; GSA was added in case that was the intent. The UI distinguishes **selected** from **currently receiving**; nothing is fabricated. The path is a non-blocking 6 KB stream buffer drained by one task: a slow host drops log lines (counted), never RTCM.

---

## 14. Serial console

115 200 Bd (when in console mode):

| Command | |
|---|---|
| `help` | list commands |
| `status` | boot/reset info, heap, Wi-Fi, GNSS, RTCM, NTRIP, caster, NTP, task stacks |
| `dns <host>` | show resolvers, test a lookup |
| `gnss <PQTM…>` | send a command to the LG290P |
| `level <e\|w\|i\|d>` | log level |
| `wifi <ssid> [pass]` | set Wi-Fi (rescue when you're locked out) |
| `stream <on\|off>` | raw GNSS stream mode |
| `save` / `factory` / `reboot` | persist / erase config / restart |

Log levels: ERROR, WARN, INFO (default), DEBUG.

---

## 15. Persistence

* One blob in NVS (`magic | version | size | CRC32` + struct), written **only on explicit Save**, skipped if unchanged, verified after write (NVS itself is power-fail safe).
* Corrupt / invalid / written-by-newer-firmware → **defaults** (the boot log says which).
* **Migration rule:** the config struct is append-only; an older blob copies the common prefix and new fields keep their defaults. (Changing the *meaning* of existing fields — as happened when the position moved from lat/lon to XYZ during development — makes old blobs fail validation and fall back to defaults.)
* Factory reset erases the namespace.
* Passwords are stored in clear in NVS (enable ESP32 flash/NVS encryption if physical access is a concern).

---

## 16. OTA updates

1. Build: `pio run` → `.pio/build/esp32dev/firmware.bin` (**not** `bootloader.bin` / `partitions.bin`).
2. Web UI → **Firmware** → choose file → **Upload**.
3. The image goes to the inactive slot; the boot slot switches only after the whole image validates; then the device restarts.

A failed or interrupted upload leaves the running firmware untouched. Notes:
* Flash erase/write briefly stalls interrupts: a few RTCM bytes may be lost during OTA and **Save** (CRC check + resync absorb this; prefer ≤230 400 Bd if you want zero loss during Save).
* The prebuilt Arduino bootloader has **no automatic rollback**: a valid-but-broken image won't revert. Keep serial access.
* Partition table (`partitions.csv`, 4 MB flash): NVS 0x5000 @0x9000, otadata 0x2000 @0xE000, two 1.875 MB app slots, 192 KB spare.

---

## 17. Reliability design

| Mechanism | Detail |
|---|---|
| Isolation | Each NTRIP upload has its own task; caster uses non-blocking sockets; web server has its own task, 2 sockets, LRU purge; the GNSS task never blocks on any of them |
| Soft recovery | Every worker task kicks a heartbeat; a 1 Hz supervisor flags overdue tasks and each task resets its own resource (UART, sockets, listener) |
| Hard recovery | All workers also subscribed to the ESP-IDF task watchdog (30 s) → panic reset as a last resort |
| Wi-Fi ladder | Reconnect with backoff 1→60 s → radio restart after 10 min down → clean reboot after 60 min (`REBOOT_AFTER_MS` in `NetworkManager.cpp`, 0 = never) |
| UART | Silent > 5 s → port re-opened every 5 s; 1005 keeps flowing; partial frames abandoned by timeout |
| Memory | Static buffers on hot paths, no `String`, no JSON library; warn < 30 kB free; clean counted reboot if < 14 kB for 60 s |
| Network hygiene | TCP keep-alive on all sockets, stall timeouts, handshake timeouts, bounded headers/bodies |
| Observability | Reset reason + persistent (RTC-memory) counters of boots / WDT / panic / brown-out / heap / Wi-Fi resets; per-task stack headroom and heartbeat age |

No periodic "just reboot" strategy is used.

---

## 18. Resource budget

* **CPU clock: 160 MHz** (`CPU_FREQ_MHZ`; keep `board_build.f_cpu` in sync). 80 MHz is possible for Wi-Fi but leaves little margin for TLS handshakes, HTTP and software-emulated doubles; soak-test before lowering it.
* **Buffers:** UART RX ring 8 KB (178 ms at 460 800 Bd), RTCM bus 16 KB (several seconds of MSM7 for the slowest consumer), parser 2×2 KB, caster ≈2 KB per rover, log stream 6 KB.
* **Sockets:** stock arduino-esp32 has **10 lwIP sockets** — web server 4 (2 internal + 2 open), NTRIP uploads 2, NTP 1, caster listener 1 → **2 caster rovers**. Enforced with a `static_assert`; more requires rebuilding lwIP with a higher `CONFIG_LWIP_MAX_SOCKETS`.
* **Counters** are 32-bit (wrap at 4.29 × 10⁹).
* **Flash:** web UI ~7 KB gzip embedded in the app image.

---

## 19. Security

Be realistic — this is an embedded device on a LAN:

* The web UI is **plain HTTP with Basic auth**: credentials and settings cross the network unencrypted. Use a trusted/isolated network. HTTPS is not implemented.
* **Passwords are shown in clear text in Settings and sent in clear in the settings response.** Anyone sniffing the LAN or seeing your screen can read them.
* Mitigations present: auth on every endpoint incl. OTA and static files, 250 ms throttle per failed login, constant-time credential compares, 3 KB body cap, bounded headers, field length/range validation, separate caster credentials.
* Outbound NTRIP TLS does not verify the server certificate.
* NTP and caster ports are reachable by anything on the LAN (NTP is rate-limited; the caster can require auth).
* Change the default `admin`/`admin` password — the dashboard warns until you do.

---

## 20. Testing

Host tests for the hardware-independent core (no ESP32 needed):

```bash
cd test/host && make
```
They build with `-Wall -Wextra -fsanitize=address,undefined` and check: CRC-24Q vs the standard check value; the 1005 bit layout against an independent decoder (incl. negative coordinates and range rejection); ECEF↔geodetic round-trips; catalogue contents; parser handling of mixed RTCM/NMEA/UBX, corrupted frames, a valid frame hidden inside a corrupted one, truncated frames, 200 kB of random noise, `$` inside RTCM payloads, oversize UBX; bus wrap-around, ordering, overtaken-consumer recovery and the cached 1005.

On-device behaviour (tasks, Wi-Fi, caster, NTRIP, NTP, OTA) needs bench testing. Suggested checklist: continuous RTCM+NMEA run; unplug the LG290P TX (UART recovery, 1005 continues); pull Wi-Fi for minutes and restore; take one NTRIP upload target down; connect 2 rovers then stall one; hammer the web UI; OTA success and a deliberately interrupted OTA; corrupt NVS (factory-reset path); multi-day soak watching heap and counters.

---

## 21. Troubleshooting

| Symptom | Check |
|---|---|
| Dashboard "UART SILENT" | RX/TX swapped, baud mismatch, 3.3 V levels, common GND |
| CRC errors rising / `backlogMax` near 8192 | Baud/noise; consumer starved |
| Rover gets no fix | No MSM observations enabled on the LG290P; wrong ARP/frame; 1005 disabled |
| NTP "unsynchronised" | Needs RMC with status `A` — see the NTP card's `why`; enable RMC (§12); check antenna/fix |
| NTRIP "DNS lookup failed" | Try `dns <host>`; check host spelling; router DNS/filtering (firmware falls back to 1.1.1.1/8.8.8.8) |
| NTRIP 401 / "Bad Password" | Rev1 uses only the password (mount = credential name on Onocoy); Rev2 uses user + password |
| Caster returns 503 | Both rover slots used (§18) |
| Web UI unreachable after changing Wi-Fi | Console: `wifi <ssid> <pass>` then `save` |
| Link lost after changing receiver baud | Set the same baud in Settings, or `factory` via console |
| Build uses the wrong core | Use `espressif32@6.9.0` (arduino-esp32 2.0.17) |

---

## 22. Known limitations

* No HTTPS; passwords visible in the UI; outbound TLS unauthenticated.
* Caster limited to 2 rovers on stock lwIP.
* NTP accuracy limited by NMEA latency/Wi-Fi unless a 1PPS line is wired.
* No bootloader rollback.
* No per-core CPU-load figure (FreeRTOS run-time stats aren't enabled in the prebuilt core); the dashboard shows core pinning, stack headroom and heartbeats instead.
* RTCM 1005 only (no 1006 antenna height, no ESP32-generated 1033).
* RXM-RAWX/SFRBX frames above ~60 measurements are dropped as oversize.
* Rover-role NTRIP (downloading corrections) and survey-in are not implemented.

---

## 23. Repository layout

```
├── platformio.ini          PlatformIO env (espressif32@6.9.0 → arduino-esp32 2.0.17)
├── partitions.csv          4 MB flash, 2 × OTA slots
├── src/
│   ├── UserConfig.h        ← edit factory defaults here
│   ├── main.cpp            setup + housekeeping loop
│   ├── Config.* ConfigStorage.*
│   ├── GnssUart.* GnssParser.* GnssRouter.* GnssCmd.* GnssTime.*
│   ├── Rtcm.* Rtcm1005.* RtcmBus.*
│   ├── NtripClient.* NtripCaster.* NtpServer.*
│   ├── WebUi.* WebApi.* OtaManager.* WebAssets.h (generated)
│   └── LogManager.* WatchdogManager.* NetworkManager.* SystemStatus.* NetUtil.h
├── data/                   index.html · style.css · app.js  (embedded at build time)
├── tools/embed_web.py      gzip + embed data/ → src/WebAssets.h
└── test/host/              hardware-independent tests (make)
```

---

## 24. License
GNU General Public License v3.0
