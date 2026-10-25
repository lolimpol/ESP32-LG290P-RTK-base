#pragma once
// =============================================================================
//  USER CONFIGURATION  -  EDIT THIS FILE BEFORE COMPILING
// -----------------------------------------------------------------------------
//  The DEF_* values are FACTORY DEFAULTS. They are used on first boot and after
//  a factory reset. Afterwards the live configuration lives in NVS and is edited
//  through the web UI. The other macros are compile-time hardware/resource limits.
// =============================================================================
#define FW_NAME     "ESP32-RTK-Base"
#define FW_VERSION  "1.0.0"

// ---- CPU clock (80 or 160). 160 MHz recommended, see README. ---------------
#define CPU_FREQ_MHZ 160

// ---- Wi-Fi (station mode only) ----------------------------------------------
#define DEF_WIFI_SSID  "CHANGE_ME_SSID"
#define DEF_WIFI_PASS  "CHANGE_ME_PASSWORD"      // empty string = open network
#define DEF_HOSTNAME   "rtk-base"

// ---- Web UI login (HTTP Basic) - CHANGE THE PASSWORD ------------------------
#define DEF_WEB_USER   "admin"
#define DEF_WEB_PASS   "admin"

// ---- LG290P UART ------------------------------------------------------------
#define GNSS_SERIAL        Serial2      // UART2 on classic ESP32 (UART0 = USB/console)
#define DEF_GNSS_RX_PIN    16           // ESP32 RX  <- LG290P TX
#define DEF_GNSS_TX_PIN    17           // ESP32 TX  -> LG290P RX (unused; -1 = none)
#define DEF_GNSS_BAUD      460800
#define GNSS_UART_RX_BUF   8192         // driver ring buffer, see README sizing
#define GNSS_PPS_PIN       -1           // optional 1PPS input GPIO (-1 = not wired)

// ---- Base-station reference position: ECEF Cartesian X/Y/Z in METRES ------------
//  Antenna reference point (ARP), in the frame/epoch your rovers expect (e.g. ITRF/ETRS89
//  coordinates of your CORS/survey). These are EXAMPLE VALUES (a point near 50N 5E) - CHANGE.
#define DEF_BASE_X         4092296.4623
#define DEF_BASE_Y         358029.5482
#define DEF_BASE_Z         4862865.6422
#define DEF_STATION_ID     0            // RTCM DF003, 0..4095
#define DEF_ITRF_YEAR      0            // RTCM DF021, 0 = unspecified
#define DEF_1005_ENABLE    1
#define DEF_1005_INTERVAL_S 5

// ---- NTRIP upload services (ESP32 acts as NTRIP *Server*/source, see README) --
#define DEF_N1_ENABLED 0
#define DEF_N1_HOST    "caster1.example.com"
#define DEF_N1_PORT    2101
#define DEF_N1_MOUNT   "BASE1"
#define DEF_N1_USER    ""
#define DEF_N1_PASS    ""
#define DEF_N2_ENABLED 0
#define DEF_N2_HOST    "caster2.example.com"
#define DEF_N2_PORT    2101
#define DEF_N2_MOUNT   "BASE2"
#define DEF_N2_USER    ""
#define DEF_N2_PASS    ""
#define DEF_N_RETRY_S  10

// ---- Local NTRIP caster ------------------------------------------------------
#define DEF_CASTER_ENABLED 1
#define DEF_CASTER_PORT    2101
#define DEF_CASTER_MOUNT   "BASE"
#define DEF_CASTER_USER    "rover"          // both empty = open caster
#define DEF_CASTER_PASS    "change-me"
#define DEF_CASTER_MAXCLI  2
#define CASTER_COUNTRY     "XXX"            // ISO 3166 alpha-3 for the sourcetable

// ---- NTP server --------------------------------------------------------------
#define DEF_NTP_ENABLED    1
#define DEF_NTP_PORT       123
#define DEF_NTP_LATENCY_MS 0                // NMEA-latency compensation (no PPS)

// ---- Logging / USB stream ------------------------------------------------------
#define DEF_LOG_LEVEL      2                // 0=ERROR 1=WARN 2=INFO 3=DEBUG
#define DEF_USB_BAUD       921600
#define DEF_LOG_MASK       0x0011           // GGA|RMC  (bit values in LogManager.h)

// =============================================================================
//  Resource limits (compile time) - change only if you know why
// =============================================================================
#define RTCM_BUS_BYTES        16384         // shared RTCM fan-out ring (power of 2)
// lwIP on the stock Arduino-ESP32 build has 10 sockets:
//   httpd 2 internal + 2 open, NTRIP 2, NTP 1, caster listener 1  => 8, leaves 2.
#define LWIP_SOCKET_BUDGET    10
#define CASTER_MAX_CLIENTS    2
#define HTTPD_OPEN_SOCKETS    2
#define TWDT_TIMEOUT_S        30
static_assert(HTTPD_OPEN_SOCKETS + 2 + 2 + 1 + 1 + CASTER_MAX_CLIENTS <= LWIP_SOCKET_BUDGET,
              "socket budget exceeded: raise CONFIG_LWIP_MAX_SOCKETS or lower limits");
