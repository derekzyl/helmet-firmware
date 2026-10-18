#pragma once

#include <cstdint>

// ==============================================================================
// SMART BIKE IGNITION FIRMWARE CONFIGURATION CONSTANTS
// ==============================================================================

#define FIRMWARE_VERSION            "1.0.0"
#define DEFAULT_DEVICE_ID           "IGN-2026-0001"
#define DEFAULT_PAIRED_HELMET_ID    "HLM-2026-0001"

// Default Pre-Shared Key (32-byte hex string) for HMAC-SHA256 pairing
#define DEFAULT_PAIRING_PSK         "d4a8f9c1e2b3a4c5d6e7f8091a2b3c4d5e6f7a8b9c0d1e2f3a4b5c6d7e8f9a0b"

// Local Link & Networking
#define LOCAL_SERVER_PORT           8080
#define SOFTAP_TIMEOUT_MS           15000   // 15 seconds before starting fallback SoftAP
#define DEFAULT_SOFTAP_PASS         "helmet1234"

// LCD & UI Display
#define LCD_COLUMNS                 16
#define LCD_ROWS                    2
#define TOTAL_LCD_PAGES             5

// Safety & Analog Sensing
#define ADC_12V_DIVIDER_RATIO       5.7f    // (47k + 10k) / 10k = 5.7
#define V12_CRITICAL_LOW_VOLTS      11.5f   // Weak 12V battery warning
#define WDT_TIMEOUT_SECONDS         8       // 8 seconds hardware watchdog
