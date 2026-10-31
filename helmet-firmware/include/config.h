#pragma once

#include <cstdint>

// ==============================================================================
// SMART HELMET FIRMWARE CONFIGURATION CONSTANTS
// ==============================================================================

#define FIRMWARE_VERSION            "1.0.0"
#define DEFAULT_DEVICE_ID           "HLM-2026-0001"
#define DEFAULT_PAIRED_IGNITION_ID  "IGN-2026-0001"

// Default Pre-Shared Key (32-byte hex string) for HMAC-SHA256 local pairing
#define DEFAULT_PAIRING_PSK         "d4a8f9c1e2b3a4c5d6e7f8091a2b3c4d5e6f7a8b9c0d1e2f3a4b5c6d7e8f9a0b"

// Local Link Timings
#define LOCAL_HEARTBEAT_INTERVAL_MS 500     // 500 ms heartbeat
#define LOCAL_LINK_TIMEOUT_MS       2000    // 2000 ms link loss threshold
#define LOCAL_SERVER_PORT           8080
#define LOCAL_SERVER_MDNS_HOST      "smarthelmet-ignition.local"
#define DEFAULT_WIFI_SSID           "SmartHelmet_AP"
#define DEFAULT_WIFI_PASS           "helmet1234"
#define DEFAULT_SOFTAP_PASS         "helmet1234"
#define DEFAULT_SOFTAP_PREFIX       "SH-IGN-"
#define SOFTAP_ONLY_SSID            "__SOFTAP__"   // NVS sentinel: skip STA, peer SoftAP only
#define WIFI_STA_TIMEOUT_MS         12000
#define WIFI_RECONNECT_INTERVAL_MS  8000
#define AUTO_BLE_AFTER_NO_LINK_MS   25000
#define INTERNET_PROBE_INTERVAL_MS  20000

// Cloud Telemetry Timings
#define CLOUD_TELEMETRY_INTERVAL_MS 3000    // 3 seconds cloud update interval
#define CLOUD_BACKEND_HOST          "big-mariejeanne-cybergenii-b60e7032.koyeb.app"
#define CLOUD_BACKEND_PORT          443
#define CLOUD_USE_HTTPS             1
#define CLOUD_TELEMETRY_ENDPOINT    "/api/v1/telemetry/ingest"
#define CLOUD_EVENTS_ENDPOINT       "/api/v1/events/"

// Safety Thresholds
#define DEFAULT_BAC_THRESHOLD       0.05f   // 0.05% BAC limit
#define DEFAULT_IMPACT_THRESHOLD_G  4.0f    // 4.0 G peak impact
#define DEFAULT_TILT_THRESHOLD_DEG  60.0f   // 60 degrees tilt
#define CRITICAL_BATTERY_PCT        15      // 15% critical low battery warning

// BLE Provisioning UUIDs (PROTOCOL.md §5)
#define BLE_ADV_NAME                "SH-HLM-PROV"
#define BLE_SERVICE_UUID            "0000FE20-0000-1000-8000-00805F9B34FB"
#define BLE_CHAR_SSID_UUID          "0000FE21-0000-1000-8000-00805F9B34FB"
#define BLE_CHAR_PASS_UUID          "0000FE22-0000-1000-8000-00805F9B34FB"
#define BLE_CHAR_PSK_UUID           "0000FE23-0000-1000-8000-00805F9B34FB"
#define BLE_CHAR_PAIRID_UUID        "0000FE24-0000-1000-8000-00805F9B34FB"
#define BLE_CHAR_STATUS_UUID        "0000FE25-0000-1000-8000-00805F9B34FB"

#define LINK_LED_BLINK_FAST_MS      150
#define LINK_LED_BLINK_SLOW_MS      400

// Hardware Watchdog Timeout (seconds)
#define WDT_TIMEOUT_SECONDS         8
