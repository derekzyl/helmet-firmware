#pragma once

#include <cstdint>
#include <string>

// ==============================================================================
// COMMON DATA STRUCTURES & PROTOCOL ENUMS
// ==============================================================================

enum class IgnitionState {
    LOCKED,
    UNLOCKED,
    RUNNING_LATCHED,
    FAULT
};

enum class LockReason {
    NONE,
    NO_LINK,
    NO_HELMET,
    ALCOHOL_DETECTED,
    WARMING_UP,
    CRASH_EVENT,
    LOW_BATTERY,
    TAMPER_ALERT
};

inline const char* ignition_state_to_str(IgnitionState state) {
    switch (state) {
        case IgnitionState::LOCKED: return "LOCKED";
        case IgnitionState::UNLOCKED: return "UNLOCKED";
        case IgnitionState::RUNNING_LATCHED: return "RUNNING_LATCHED";
        case IgnitionState::FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

inline const char* lock_reason_to_str(LockReason reason) {
    switch (reason) {
        case LockReason::NONE: return "NONE";
        case LockReason::NO_LINK: return "NO_LINK";
        case LockReason::NO_HELMET: return "NO_HELMET";
        case LockReason::ALCOHOL_DETECTED: return "ALCOHOL_DETECTED";
        case LockReason::WARMING_UP: return "WARMING_UP";
        case LockReason::CRASH_EVENT: return "CRASH_EVENT";
        case LockReason::LOW_BATTERY: return "LOW_BATTERY";
        case LockReason::TAMPER_ALERT: return "TAMPER_ALERT";
        default: return "UNKNOWN";
    }
}

struct HelmetTelemetryFrame {
    uint32_t timestamp = 0;
    uint32_t seq = 0;
    
    // Helmet sensor readings
    bool helmet_worn = false;
    uint32_t worn_duration_ms = 0;
    float bac_level = 0.0f;
    uint16_t mq3_raw = 0;
    bool mq3_warmed_up = false;
    
    // IMU & Dynamics
    bool impact_flag = false;
    float impact_g = 1.0f;
    float tilt_deg = 0.0f;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float ax = 0, ay = 0, az = 1;
    float gx = 0, gy = 0, gz = 0;
    bool imu_anomaly = false;
    bool tilt_warning = false;
    bool rider_upright = true;
    const char* imu_state = "IDLE";

    // Alcohol diagnostics
    float alcohol_warmup_pct = 0.0f;
    float bac_rise_rate = 0.0f;
    bool alcohol_rising_anomaly = false;
    bool alcohol_calibrated = false;
    
    // GPS
    bool gps_fixed = false;
    double latitude = 0.0;
    double longitude = 0.0;
    float speed_kmh = 0.0f;
    float heading_deg = 0.0f;
    float hdop = 99.9f;
    
    // Power & Diagnostics
    uint8_t battery_pct = 100;
    uint16_t battery_mv = 4100;
    bool is_charging = false;
    
    // Connectivity
    bool wifi_connected = false;
    bool gprs_connected = false;
    bool local_link_connected = false;
    bool internet_ok = false;
    bool prefer_softap = true;          // peer SoftAP after pairing (no phone Wi‑Fi required)
    const char* wifi_mode = "NONE";     // STA | SOFTAP | NONE
    int8_t rssi = -99;
};
