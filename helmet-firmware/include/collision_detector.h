#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

// ==============================================================================
// MPU-6050 6-AXIS COLLISION & INCIDENT DETECTOR
// Pure C++ algorithmic state machine with multi-stage verification
// ==============================================================================

struct CollisionConfig {
    float impact_threshold_g = 4.0f;        // Minimum peak magnitude (e.g. 4.0G)
    float tilt_threshold_deg = 60.0f;       // Minimum tilt angle indicating rider on ground
    float still_accel_tol_g = 0.35f;        // Maximum deviation from 1G during stillness
    float still_gyro_tol_dps = 20.0f;       // Maximum angular rate during stillness (deg/s)
    uint32_t stillness_window_ms = 2500;    // Time window rider must be motionless
    uint32_t cancel_window_ms = 15000;      // 15 seconds cancel window for Button A
};

class CollisionDetector {
public:
    using Config = CollisionConfig;

    enum class State {
        IDLE,               // Normal riding
        IMPACT_DETECTED,    // High G impact registered, checking for tilt & post-crash still
        PRE_ALERT,          // Incident verified, countdown window active for rider cancellation
        CONFIRMED_CRASH     // Countdown expired without cancel -> Emergency SOS dispatched
    };

    CollisionDetector() : CollisionDetector(CollisionConfig{}) {}

    explicit CollisionDetector(const CollisionConfig& config)
        : config_(config),
          state_(State::IDLE),
          impact_time_ms_(0),
          stillness_start_ms_(0),
          pre_alert_start_ms_(0),
          peak_impact_g_(1.0f),
          last_tilt_deg_(0.0f) {}

    void update(float ax, float ay, float az, float gx, float gy, float gz, uint32_t now_ms) {
        float a_mag = std::sqrt(ax * ax + ay * ay + az * az);

        float tilt_deg = 0.0f;
        if (a_mag > 0.1f) {
            float cos_tilt = std::max(-1.0f, std::min(1.0f, az / a_mag));
            tilt_deg = std::acos(cos_tilt) * 180.0f / 3.14159265f;
        }
        last_tilt_deg_ = tilt_deg;

        float g_mag = std::sqrt(gx * gx + gy * gy + gz * gz);

        switch (state_) {
            case State::IDLE:
                if (a_mag >= config_.impact_threshold_g) {
                    peak_impact_g_ = a_mag;
                    impact_time_ms_ = now_ms;
                    stillness_start_ms_ = 0;
                    state_ = State::IMPACT_DETECTED;
                }
                break;

            case State::IMPACT_DETECTED:
                if (a_mag > peak_impact_g_) {
                    peak_impact_g_ = a_mag;
                }

                {
                    bool is_still = (std::abs(a_mag - 1.0f) <= config_.still_accel_tol_g) &&
                                    (g_mag <= config_.still_gyro_tol_dps);

                    if (is_still && tilt_deg >= config_.tilt_threshold_deg) {
                        if (stillness_start_ms_ == 0) {
                            stillness_start_ms_ = now_ms;
                        } else if (now_ms - stillness_start_ms_ >= config_.stillness_window_ms) {
                            state_ = State::PRE_ALERT;
                            pre_alert_start_ms_ = now_ms;
                        }
                    } else if (!is_still && (now_ms - impact_time_ms_ > 6000)) {
                        state_ = State::IDLE;
                        peak_impact_g_ = 1.0f;
                    }
                }
                break;

            case State::PRE_ALERT:
                if (now_ms - pre_alert_start_ms_ >= config_.cancel_window_ms) {
                    state_ = State::CONFIRMED_CRASH;
                }
                break;

            case State::CONFIRMED_CRASH:
                break;
        }
    }

    bool cancel_alarm() {
        if (state_ == State::PRE_ALERT || state_ == State::IMPACT_DETECTED) {
            state_ = State::IDLE;
            peak_impact_g_ = 1.0f;
            return true;
        }
        return false;
    }

    void reset() {
        state_ = State::IDLE;
        peak_impact_g_ = 1.0f;
        stillness_start_ms_ = 0;
        pre_alert_start_ms_ = 0;
    }

    State get_state() const { return state_; }
    bool is_idle() const { return state_ == State::IDLE; }
    bool is_pre_alert() const { return state_ == State::PRE_ALERT; }
    bool is_confirmed_crash() const { return state_ == State::CONFIRMED_CRASH; }
    float get_peak_impact_g() const { return peak_impact_g_; }
    float get_tilt_deg() const { return last_tilt_deg_; }

    uint32_t get_remaining_cancel_sec(uint32_t now_ms) const {
        if (state_ != State::PRE_ALERT) return 0;
        uint32_t elapsed = now_ms - pre_alert_start_ms_;
        if (elapsed >= config_.cancel_window_ms) return 0;
        return (config_.cancel_window_ms - elapsed) / 1000;
    }

private:
    CollisionConfig config_;
    State state_;
    uint32_t impact_time_ms_;
    uint32_t stillness_start_ms_;
    uint32_t pre_alert_start_ms_;
    float peak_impact_g_;
    float last_tilt_deg_;
};
