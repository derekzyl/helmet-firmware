#pragma once

#include <cstdint>
#include <string>
#include <algorithm>

// ==============================================================================
// SAFETY-CRITICAL BIKE IGNITION STATE MACHINE
// Implements Fail-Safe Default, 5-Point Pre-Ride Gate, and Anti-Stall Latch
// ==============================================================================

enum class IgnitionState {
    LOCKED,             // SSR OFF (LOW) - Default safe state
    UNLOCKED,           // SSR ON (HIGH) - All pre-ride conditions satisfied
    RUNNING_LATCHED,    // SSR ON (HIGH) - Engine running mid-ride (NEVER CUT MID-RIDE)
    FAULT               // SSR OFF (LOW) - Severe accident / safety fault cutoff
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

struct HeartbeatInput {
    bool worn = false;
    uint32_t worn_ms = 0;
    float bac = 0.0f;
    bool mq3_warm = false;
    bool impact = false;
    uint8_t batt_pct = 100;
    float speed_kmh = 0.0f;
    // Diagnostic mirror (does not gate FSM by itself)
    bool ir_near = false;
    int ir_raw = 1;
    uint16_t mq3_raw = 0;
    float tilt_deg = 0.0f;
    float impact_g = 1.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 1.0f;
    char imu_state[16] = "IDLE";
};

struct SafetyConfig {
    float alcohol_threshold = 0.05f;    // 0.05% BAC limit
    uint32_t link_timeout_ms = 2000;    // 2000 ms link loss threshold
    uint32_t min_worn_ms = 1000;        // 1000 ms stable IR detection required
    uint8_t min_battery_pct = 0;        // battery not gated (0 = ignore %)
    uint32_t latch_duration_ms = 5000;  // 5 seconds unlocked before latching to RUNNING
};

class SafetyFSM {
public:
    using Config = SafetyConfig;

    SafetyFSM() : SafetyFSM(SafetyConfig{}) {}

    explicit SafetyFSM(const SafetyConfig& config)
        : config_(config),
          state_(IgnitionState::LOCKED),
          lock_reason_(LockReason::NO_LINK),
          unlocked_start_ms_(0),
          last_heartbeat_ms_(0),
          mid_ride_warning_(false),
          maintenance_active_(false),
          maintenance_end_ms_(0) {}

    void update(const HeartbeatInput& hb, uint32_t now_ms, float v12_volts) {
        // Handle Maintenance Override Expiry
        if (maintenance_active_) {
            if (now_ms >= maintenance_end_ms_) {
                maintenance_active_ = false;
            } else {
                state_ = IgnitionState::UNLOCKED;
                lock_reason_ = LockReason::NONE;
                return;
            }
        }

        // Check Freshness of Helmet Link
        bool link_alive = (last_heartbeat_ms_ > 0) && (now_ms - last_heartbeat_ms_ <= config_.link_timeout_ms);

        switch (state_) {
            case IgnitionState::LOCKED: {
                mid_ride_warning_ = false;

                // Evaluate 5 Pre-Ride Gate Conditions
                if (!link_alive) {
                    lock_reason_ = LockReason::NO_LINK;
                } else if (!hb.worn || hb.worn_ms < config_.min_worn_ms) {
                    lock_reason_ = LockReason::NO_HELMET;
                } else if (!hb.mq3_warm) {
                    lock_reason_ = LockReason::WARMING_UP;
                } else if (hb.bac > config_.alcohol_threshold) {
                    lock_reason_ = LockReason::ALCOHOL_DETECTED;
                } else if (hb.impact) {
                    lock_reason_ = LockReason::CRASH_EVENT;
                } else if (hb.batt_pct < config_.min_battery_pct) {
                    lock_reason_ = LockReason::LOW_BATTERY;
                } else {
                    // ALL PRE-RIDE CONDITIONS SATISFIED -> UNLOCK!
                    state_ = IgnitionState::UNLOCKED;
                    lock_reason_ = LockReason::NONE;
                    unlocked_start_ms_ = now_ms;
                }
                break;
            }

            case IgnitionState::UNLOCKED: {
                // If conditions violated BEFORE ride starts, immediately re-lock
                if (!link_alive) {
                    state_ = IgnitionState::LOCKED;
                    lock_reason_ = LockReason::NO_LINK;
                } else if (!hb.worn || hb.worn_ms < config_.min_worn_ms) {
                    state_ = IgnitionState::LOCKED;
                    lock_reason_ = LockReason::NO_HELMET;
                } else if (hb.bac > config_.alcohol_threshold) {
                    state_ = IgnitionState::LOCKED;
                    lock_reason_ = LockReason::ALCOHOL_DETECTED;
                } else if (hb.impact) {
                    state_ = IgnitionState::FAULT;
                    lock_reason_ = LockReason::CRASH_EVENT;
                } else {
                    // Check if engine is running / session latched
                    // Conditions: speed > 3 km/h OR 12V alternator voltage > 13.0V OR stable unlocked > 5s
                    bool engine_cranked = (hb.speed_kmh > 3.0f) || (v12_volts >= 13.0f) ||
                                          (now_ms - unlocked_start_ms_ >= config_.latch_duration_ms);

                    if (engine_cranked) {
                        state_ = IgnitionState::RUNNING_LATCHED;
                    }
                }
                break;
            }

            case IgnitionState::RUNNING_LATCHED: {
                // CRITICAL ANTI-STALL SAFETY RULE:
                // NEVER cut the engine mid-ride if link dropped or helmet removed!
                // Only warn rider (buzzer / LCD / app).
                if (!link_alive || !hb.worn || hb.batt_pct < config_.min_battery_pct) {
                    mid_ride_warning_ = true;
                } else {
                    mid_ride_warning_ = false;
                }

                // If collision confirmed mid-ride, safety cutoff permitted
                if (link_alive && hb.impact) {
                    state_ = IgnitionState::FAULT;
                    lock_reason_ = LockReason::CRASH_EVENT;
                }

                // Session Ended: Bike main ignition switch turned off (12V line drops < 11.0V)
                if (v12_volts > 0.1f && v12_volts < 11.0f) {
                    state_ = IgnitionState::LOCKED;
                    lock_reason_ = LockReason::NO_HELMET;
                    mid_ride_warning_ = false;
                }
                break;
            }

            case IgnitionState::FAULT: {
                mid_ride_warning_ = false;
                // Latched fault state - requires manual clearing or app reset
                break;
            }
        }
    }

    void register_heartbeat(uint32_t now_ms) {
        last_heartbeat_ms_ = now_ms;
    }

    void reset_link() {
        last_heartbeat_ms_ = 0;
    }

    void apply_maintenance_override(uint32_t now_ms, uint32_t duration_sec) {
        maintenance_active_ = true;
        maintenance_end_ms_ = now_ms + (duration_sec * 1000);
        state_ = IgnitionState::UNLOCKED;
        lock_reason_ = LockReason::NONE;
    }

    void clear_fault() {
        state_ = IgnitionState::LOCKED;
        lock_reason_ = LockReason::NO_LINK;
        mid_ride_warning_ = false;
    }

    // --- State Queries ---
    IgnitionState get_state() const { return state_; }
    LockReason get_lock_reason() const { return lock_reason_; }
    
    // SSR control pin must be active (HIGH) ONLY when UNLOCKED or RUNNING_LATCHED
    bool is_ssr_active() const {
        return (state_ == IgnitionState::UNLOCKED || state_ == IgnitionState::RUNNING_LATCHED);
    }

    bool is_mid_ride_warning() const { return mid_ride_warning_; }
    bool is_maintenance_active() const { return maintenance_active_; }

    void set_alcohol_threshold(float threshold) {
        config_.alcohol_threshold = threshold;
    }

private:
    SafetyConfig config_;
    IgnitionState state_;
    LockReason lock_reason_;
    uint32_t unlocked_start_ms_;
    uint32_t last_heartbeat_ms_;
    bool mid_ride_warning_;
    bool maintenance_active_;
    uint32_t maintenance_end_ms_;
};
