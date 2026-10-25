# ESP32 RTK Base Station (Quectel LG290P)

Production-oriented ESP32 / Arduino-framework firmware: LG290P (UART) → RTCM 3.x + NMEA parser → shared RTCM bus → two outbound NTRIP uploads + a local NTRIP caster, an NTP server (GNSS time), USB logging for PPP/PPK, a small web UI with OTA. Original code; RTKBase (Stefal) was only a UX reference. **No** maps, Bluetooth, Ethernet, cloud, frameworks, or network control of the receiver.

> **Verification status (read this).** The hardware-independent core — CRC-24Q, RTCM 1005 encoder, the RTCM/NMEA/UBX stream parser with resynchronisation, and the RTCM fan-out bus — is covered by host tests (`test/host`, run under ASan/UBSan: all pass). The Arduino/ESP-IDF layers (tasks, Wi-Fi, sockets, web server, OTA) were **written carefully against arduino-esp32 2.0.17 but not compiled or run on hardware in this environment** (no ESP32 toolchain available). Expect to fix a few compile-level nits on first build, and **do a bench test with your LG290P before deploying** (see §10).

## 1. Assumptions you must check

| # | Assumption | Why it matters |
|---|---|---|
| A1 | **"NTRIP client" = outbound upload.** A base pushing data to a remote caster is, in NTRIP terms, an NTRIP *Server* (source). The two services send `SOURCE` (Rev1) or `POST` + Basic auth (Rev2) and then stream raw RTCM. The "send GGA" option was **deliberately omitted**: GGA is a rover→caster message, and injecting NMEA into an uploaded RTCM stream would corrupt it. Rover-role (download) mode is a ~20-line addition if you ever want it. | Protocol correctness |
| A2 | **Your listed RTCM set (1019/1020/1042/1046) contains no observations.** A rover needs MSM observations (1074/1077, 1084/1087, 1094/1097, 1124/1127) to get a fix. The catalogue therefore also contains 1045, 1033, 1230 and all MSM4/MSM7, **all enabled by default but forwarded only if the receiver actually sends them**. Enable the MSM output on the LG290P itself (the ESP32 does not configure the receiver). | A base that forwards only ephemerides gives rovers nothing to solve |
| A3 | Base position = **ECEF Cartesian X/Y/Z in metres** of the **antenna reference point (ARP)**, in the same frame/epoch the rover expects (e.g. ITRF/ETRS89 coordinates of your survey/CORS). 1005 carries no antenna height (that is 1006). DF021 (ITRF year) defaults to 0 = unspecified. BeiDou has no indicator bit in 1005. | Absolute accuracy of the rover |
| A4 | "SGA" is **not a standard NMEA 0183 sentence**. It is matched by its 3-letter ID and forwarded only if received. I also added **GSA** as a 8th selectable stream (probably what was meant). | No fabricated messages |
| A5 | **RXM-RAWX / RXM-SFRBX are u-blox UBX messages.** The parser frames UBX (sync B5 62, Fletcher checksum, payload ≤ 2048 B) and forwards them if they ever arrive. The LG290P does not normally emit them; the UI shows *selected* vs *receiving* separately. Larger RAWX frames (> ~60 measurements) are counted as oversize and dropped. | Honest reporting |
| A6 | NTP accuracy without 1PPS is limited by NMEA latency + Wi-Fi jitter (**≈10–100 ms**, calibrate with `ntp_lat_ms`). With a 1PPS wire (`GNSS_PPS_PIN`) ≈1–5 ms. Assumes the RMC for second T is sent *after* the PPS edge of T. | Don't over-trust the clock |
| A7 | Stock arduino-esp32 has **10 lwIP sockets**: HTTP server 4 (2 internal + 2 open), NTRIP 2, NTP 1, caster listener 1 → **2 caster clients max** (compile-time `CASTER_MAX_CLIENTS`, checked by `static_assert`). More needs a rebuilt lwIP (`CONFIG_LWIP_MAX_SOCKETS`). | Hard platform limit |

## 2. Architecture

```
LG290P TX ─UART2─▶ [driver RX ring 8 KB, ISR-fed]
                          │  (consumer)
              ┌───────────▼───────────┐   core 1, prio 6  "gnss" task
              │ GnssParser (stream)   │   RTCM(CRC24Q) | NMEA(checksum) | UBX(Fletcher), resync-by-replay
              └───────┬───────────────┘
                      ▼
              GnssRouter ── NMEA: counters, GGA snapshot, RMC→GnssTime, selected lines → LogManager stream
                      │  RTCM: catalogue filter (mask) ; upstream 1005/1006 dropped if ESP32 generates 1005
                      │  + ESP32-generated 1005 every N s (encoded once per config change)
                      ▼
              RtcmBus: ONE 16 KB ring, frames [len16|frame], per-consumer cursors, task-notify on publish
        ┌─────────────┼───────────────┐
 NtripClient#1   NtripClient#2   NtripCaster (≤2 rovers)      core 0 tasks, prio 3
 (own task)      (own task)      (one non-blocking task)
```
Side services (never on the data path): `NtpServer` (core 0, prio 4), `NetworkManager`, `WebUi`/`WebApi` (esp_http_server task, core 0, prio 3), `LogManager` (console + USB stream, core 1, prio 2), `loop()` (housekeeping, core 1, prio 1). All status flows through `Status::g`, **one writer per field**, readers never lock the GNSS path.

**Core/task allocation.** Wi-Fi/lwIP live on core 0 in Arduino-ESP32, so everything network-facing sits there and the timing-critical GNSS parser owns core 1 with the highest app priority; only the bus (a ≤1 KB memcpy under a 5 ms-bounded mutex) and task notifications cross cores. Mutexes are never held across network calls.

**GNSS byte pipeline.** ISR → driver ring → `gnss` task reads ≤512 B chunks → parser → router → bus/log. The task never touches Wi-Fi, HTTP, NVS, or the filesystem. Buffer sizing: 8 KB RX ring = 178 ms of headroom at 460 800 Bd (1.4 s at 57 600) vs. a typical drain latency of ~2 ms; the bus holds ≥ 3 s of even MSM7 traffic (~5 kB/s) for the slowest consumer; parser buffers 2×2 KB (largest frame + replay space); caster ≈ 2 KB per rover.

**RTCM 1005.** `Rtcm1005::encode` builds the 152-bit payload MSB-first: DF002=1005 (12), DF003 station ID (12), DF021 ITRF year (6), DF022/23/24 GPS/GLO/GAL (1 each), DF141 reference-station indicator (1, 0 = physical), DF025 X (38, signed, 0.1 mm), DF142 single-oscillator (1), reserved (1), DF026 Y (38), DF364 quarter-cycle (2), DF027 Z (38). Frame = `D3 00 13` + 19 B + CRC-24Q (poly 0x1864CFB, init 0, over header+payload; verified against the standard check value 0xCDE703). The position is configured directly as ECEF X/Y/Z (metres, rounded to the 0.1 mm RTCM unit when config changes); lat/lon are derived only for the caster sourcetable. 38-bit two's-complement range is checked, and the config validator rejects points not near the Earth's surface (catches typos / all-zero). It is published immediately after any change, then every `gen1005_s` (default 5 s), and cached so every new caster rover receives it right on connect.

**Message selection.** `rt_<num>` checkboxes set `rtcmMask` (bit = catalogue index in `Rtcm.cpp`; adding a message = one table line). Messages are forwarded **only if the receiver sends them**; 1005 is the only ESP32-generated one. Dashboard rows show source (LG290P / ESP32), rx/gen/fwd counters, msgs/s, B/s, age.

**NTRIP clients.** One task each: DNS → TCP/TLS connect (8 s cap) → login → stream from a private bus cursor (attached at "now": no stale replay). Failures use exponential backoff (base interval ×2ⁿ, cap 300 s, ±10 % jitter; reset after a 30 s healthy connection). Wi-Fi down → state "waiting for Wi-Fi", no tight loops. Per-service rx/tx bytes, reconnects, failures, last error, last OK time. TLS encrypts but does **not verify** the server certificate.

**NTRIP caster.** `GET /` → sourcetable; `GET /<mount>` → Basic auth (constant-time compare; empty user+pass = open) → `ICY 200 OK` (Rev1) or `HTTP/1.1 200` (Rev2). Non-blocking `send(MSG_DONTWAIT)` per rover from its own cursor; a rover making no TCP progress for `caster_timeout` s is dropped; handshake limit 5 s; header limit 512 B; TCP keep-alive on every socket.

**NTP/GNSS time.** `unix_us = esp_timer_us + offset`; offset re-estimated from each valid RMC (status A, plausible date) — step if >200 ms off, otherwise EMA-slewed — or from the 1PPS edge when wired. The libc clock is set once and re-stepped only if >2 s wrong (UI timestamps only); NTP never reads it. States: locked (<5 s old, stratum 1 "GPS"), holdover (<120 s), else LI=3/stratum 16. Only mode-3 requests are answered; global 50 req/s limit.

**Persistence.** One versioned blob in NVS (`magic|version|size|CRC32` + struct), written only on explicit **Save**, skipped when unchanged, verified after write; corrupt/invalid/newer → defaults. Migration rule: append-only struct, older blobs copy the common prefix. Factory reset erases the namespace. Passwords are shown in the settings panel; a blank field means an empty password.

**Watchdogs/recovery.** Soft tier: per-task heartbeats, supervisor flags overdue tasks → each tears down and reopens its own resource (UART, sockets, caster listener). Hard tier: every worker also subscribed to the ESP-IDF task WDT (30 s). Plus: Wi-Fi escalation (reconnect → radio restart at 10 min → clean reboot at 60 min, `REBOOT_AFTER_MS`), UART silence re-open every 5 s, heap < 14 kB for 60 s → clean counted reboot. Boot/WDT/panic/brown-out/heap/Wi-Fi reset counters persist in RTC memory and show on the dashboard.

**RAM/CPU.** Static buffers everywhere on hot paths, no `String`, no JSON library (chunked writer), gzip'd web assets in flash, CRC via 16-entry table, doubles only on config change/1 Hz GGA.

**CPU clock: 160 MHz.** 80 MHz works for Wi-Fi but leaves little margin for TLS handshakes, the HTTP server and software-emulated doubles; at 460 800 Bd the parse/CRC load is a few percent either way. Set `CPU_FREQ_MHZ` (and `board_build.f_cpu`) to 80 only after soak-testing your traffic.

## 3. Build & flash

PlatformIO (recommended): `pio run -t upload`. Libraries: none beyond the arduino-esp32 core (2.0.17, via `espressif32@6.9.0`). Web files in `/data` are gzip-embedded into `src/WebAssets.h` automatically (`tools/embed_web.py`); there is **no** LittleFS upload step.
Arduino IDE: install *esp32 by Espressif 2.0.17*; copy `src/*` flat into a sketch folder, rename `main.cpp` → `<sketch>.ino`; board *ESP32 Dev Module*, CPU 160 MHz, partition *Minimal SPIFFS (1.9 MB APP with OTA)*; run `python3 tools/embed_web.py` first if you edited `/data`.
Partitions (`partitions.csv`, 4 MB flash): 2 × 1.875 MB OTA slots + NVS + otadata.

## 4. First start
1. Edit **`src/UserConfig.h`** (Wi-Fi, web password, pins, base position, NTRIP presets). 2. Flash. 3. Serial 115 200 Bd: `status`; find the IP. If Wi-Fi fails: `wifi <ssid> <pass>` then `save`. 4. Browse `http://<ip>/` → login (default `admin`/`admin`, **change it**). 5. Settings → enter your ECEF X/Y/Z, enable the MSM messages on the receiver, **Apply & Save**.
Console commands: `help status dns level wifi stream save factory reboot` (`dns <host>` shows the resolvers and tests a lookup).

## 5. Web UI
Dashboard (2 s refresh): system, Wi-Fi, GNSS/parser/NMEA/last GGA, RTCM table, both NTRIP services, caster, NTP, USB/log streams, warnings (default password/position/SSID, UART silent). Settings: **Apply** (validated, applied live, not persisted), **Apply & Save**, Reload, Restart, Factory reset. Only a GNSS UART *pin* change needs a restart. Firmware tab: OTA upload with progress.

## 5b. Talking to the LG290P (Receiver tab / `gnss` console command)
Wire **ESP32 TX (GPIO 17 by default) → LG290P RX** (the TX pin must not be -1). Commands are Quectel `PQTM…` strings; `$`, `*checksum` and CR/LF are added automatically (checksum routine verified against the examples in Quectel's protocol specification). Replies (`$PQTMCFGMSGRATE,OK*29`, `…,ERROR,<code>`) appear in the Receiver tab. Useful ones: `PQTMCFGMSGRATE,W,RMC,1` (enable RMC, needed for NTP time), `PQTMCFGMSGRATE,R,RMC` (read rate), `PQTMSAVEPAR` (persist to receiver flash). Note from Quectel's base-station application note: the module turns NMEA output off automatically when it enters base mode, so individual NMEA messages must be re-enabled this way.

## 6. OTA
Upload the app `.bin` (`.pio/build/esp32dev/firmware.bin`). Written to the inactive slot, header-checked, full-image validated by `esp_ota_end`; only then does the boot slot switch. A failed/aborted upload leaves the running slot untouched. (Bootloader rollback is not enabled in the prebuilt Arduino bootloader, so a *valid but non-working* image will not auto-revert — keep serial access.) Flash erase/write briefly stalls ISRs: expect lost RTCM bytes during OTA and during **Save** (CRC check + resync handle it; prefer ≤230 400 Bd on the LG290P if you want zero loss during Save).

## 7. USB/PPP stream
`usb_mode=1` turns the console UART into a raw stream of the selected sentences/UBX frames (diagnostics are muted so text cannot corrupt the binary; CLI input still works, `stream off` returns). Classic ESP32 has no native USB: this is the USB-UART bridge on UART0. On S2/S3/C3 choose "USB CDC On Boot" so `Serial` is native. Everything goes through a non-blocking 6 KB StreamBuffer; if the host is slow, lines are dropped and counted — never the RTCM path.

## 8. Security (be realistic)
HTTP Basic over **plain HTTP**: credentials and settings cross the LAN unencrypted — use a trusted/isolated VLAN; HTTPS on a 160 MHz ESP32 with OTA + two TLS uploads is not worth the RAM/latency risk and is not implemented. Mitigations: auth on every endpoint incl. static files and OTA, 250 ms throttle per failed login, constant-time compares, 3 KB body cap, bounded headers, field-length/range validation, caster auth separate. **Passwords are shown in clear in the Settings panel** (by design choice) and are therefore also sent in clear in the settings response over plain HTTP; anyone who can sniff the LAN or who sees your screen gets them. Passwords are stored in NVS in clear (enable NVS/flash encryption for physical-attack resistance). NTP/caster ports are unauthenticated by nature (NTP is rate-limited).

## 9. Troubleshooting
*Dashboard "UART SILENT"*: swap RX/TX, check baud and 3.3 V levels, GND. *CRC errors rising*: baud mismatch or noise; `backlogMax` near 8192 means the consumer is starved. *Rover gets no fix*: MSM not enabled on the LG290P (A2), wrong ARP/height, 1005 disabled. *NTRIP "401/ERROR - Bad Password"*: Rev1 uses only the password, Rev2 user+password; some casters want the other revision. *"DNS lookup failed"*: DHCP DNS. *Caster 503*: slots exhausted (A7). *NTP unsynchronised*: needs RMC status A. *Compile error on `GNSS_SERIAL`/`WiFiClient::connect(…, timeout)`*: you are not on arduino-esp32 2.0.x.

## 10. Test scenarios — expected behaviour
(* = covered by host tests; others reasoned from the design and need a bench check)

| # | Scenario | Expected |
|---|---|---|
|1| Continuous RTCM+NMEA | parsed inline, no drops; `backlogMax` small; counters/rates advance |
|2*| Corrupted RTCM frame | CRC fails → `rtcmCrcErr++`, frame dropped, bytes after its first byte are replayed so a good frame inside is still found |
|3*| Sync lost / noise / truncated frame | resync by replay; idle-gap timeout (`partialTimeouts`) clears a half frame; false-accept chance ≈2⁻²⁴ per candidate |
|4/5| NTRIP#1 drops, #2 up | #1 → backoff/reconnect with its own task & cursor; #2 untouched |
|6/7| Wi-Fi gone minutes / back | GNSS parsing continues, bus keeps overwriting oldest frames; services show "waiting for Wi-Fi"; reconnect backoff 1→60 s; on return clients reconnect, attach at "now"; radio restart at 10 min, reboot at 60 min |
|8| Several caster rovers | each has its own cursor; max 2 (A7), 3rd gets 503 |
|9| One rover hangs | non-blocking send; no TCP progress for `caster_timeout` → dropped; others unaffected |
|10| Many web requests | httpd has own task/2 sockets/LRU purge; GNSS task unaffected; slow login attempts throttled |
|11/12| Edit + Save | validated → applied live (generation counter) → NVS write only on Save, verify-after-write |
|13/14| Reboot | config reloaded from NVS |
|15| Corrupt config | CRC/magic/validation fails → defaults, boot log says so |
|16/17| OTA ok / aborted | ok: validated, slot switched, reboot. Abort: `Update.abort()`, old firmware untouched |
|18| UART stops | `uartOk=false` after 5 s, port re-opened every 5 s, `uartRecoveries++`; 1005 keeps flowing |
|19| NMEA only | NMEA/time/GGA work; RTCM rows stay 0 (only 1005 generated) |
|20| RTCM but no GGA | RTCM flows; GGA panel "invalid"; NTP needs RMC |
|21/22| 1005 on/off | on: immediate + periodic frame, upstream 1005/1006 suppressed; off: none generated (upstream 1005 forwarded only if sent and enabled) |
|23| Selected stream not received | UI: *selected* yes, *receiving* no, counters 0 — nothing fabricated |
|24| Heap creeping | warning <30 kB, clean counted reboot after 60 s <14 kB |
|25/26| Task stuck | soft flag → task self-recovers; if wedged, 30 s TWDT reset (counted, shown) |
|27| Days of operation | no heap use on hot paths, bounded buffers, counters wrap at 2³², RTC-persisted reset statistics |

## 11. Limitations
32-bit counters wrap at 4.29 GB. No per-core CPU-load figure (FreeRTOS run-time stats are not enabled in the prebuilt core); the dashboard shows core pinning, stack headroom and heartbeats instead. No HTTPS, no rollback bootloader, TLS without server verification, caster limited to 2 rovers on stock lwIP.

Host tests: `cd test/host && make`.
