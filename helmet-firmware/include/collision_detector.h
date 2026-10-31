#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

// ==============================================================================
// MPU-6050 6-AXIS: tilt, crash FSM, and riding anomaly detection
// ==============================================================================

struct CollisionConfig {
    float impact_threshold_g = 4.0f;
    float tilt_threshold_deg = 60.0f;
    float warn_tilt_deg = 35.0f;           // Soft warning while riding
    float still_accel_tol_g = 0.35f;
    float still_gyro_tol_dps = 20.0f;
    float anomaly_gyro_dps = 220.0f;       // Sudden spin / tumble
    float anomaly_jerk_g = 2.2f;           // Rapid accel change
    uint32_t stillness_window_ms = 2500;
    uint32_t cancel_window_ms = 15000;
    uint32_t anomaly_hold_ms = 800;
};

class CollisionDetector {
public:
    using Config = CollisionConfig;

    enum class State {
        IDLE,
        IMPACT_DETECTED,
        PRE_ALERT,
        CONFIRMED_CRASH
    };

    CollisionDetector() : CollisionDetector(CollisionConfig{}) {}

    explicit CollisionDetector(const CollisionConfig& config)
        : config_(config),
          state_(State::IDLE),
          impact_time_ms_(0),
          stillness_start_ms_(0),
          pre_alert_start_ms_(0),
          anomaly_until_ms_(0),
          peak_impact_g_(1.0f),
          last_tilt_deg_(0.0f),
          last_a_mag_(1.0f),
          ax_(0), ay_(0), az_(1),
          gx_(0), gy_(0), gz_(0),
          roll_deg_(0), pitch_deg_(0),
          anomaly_active_(false),
          tilt_warn_(false) {}

    void update(float ax, float ay, float az, float gx, float gy, float gz, uint32_t now_ms) {
        ax_ = ax; ay_ = ay; az_ = az;
        gx_ = gx; gy_ = gy; gz_ = gz;

        float a_mag = std::sqrt(ax * ax + ay * ay + az * az);
        float jerk = std::abs(a_mag - last_a_mag_);
        last_a_mag_ = a_mag;

        // Tilt from gravity axis (0° = upright)
        float tilt_deg = 0.0f;
        if (a_mag > 0.1f) {
            float cos_tilt = std::max(-1.0f, std::min(1.0f, az / a_mag));
            tilt_deg = std::acos(cos_tilt) * 180.0f / 3.14159265f;
        }
        last_tilt_deg_ = tilt_deg;
        tilt_warn_ = tilt_deg >= config_.warn_tilt_deg && tilt_deg < config_.tilt_threshold_deg;

        // Approximate roll / pitch for diagnostics (degrees)
        roll_deg_ = std::atan2(ay, az) * 180.0f / 3.14159265f;
        pitch_deg_ = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * 180.0f / 3.14159265f;

        float g_mag = std::sqrt(gx * gx + gy * gy + gz * gz);

        // Riding anomalies: high spin or jerk while not already in crash FSM
        bool anomaly = (g_mag >= config_.anomaly_gyro_dps) || (jerk >= config_.anomaly_jerk_g);
        if (anomaly && state_ == State::IDLE) {
            anomaly_active_ = true;
            anomaly_until_ms_ = now_ms + config_.anomaly_hold_ms;
        } else if (now_ms >= anomaly_until_ms_) {
            anomaly_active_ = false;
        }

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
                if (a_mag > peak_impact_g_) peak_impact_g_ = a_mag;
                {
                    bool is_still = (std::abs(a_mag - 1.0f) <= config_.still_accel_tol_g) &&
                                    (g_mag <= config_.still_gyro_tol_dps);
                    if (is_still && tilt_deg >= config_.tilt_threshold_deg) {
                        if (stillness_start_ms_ == 0) stillness_start_ms_ = now_ms;
                        else if (now_ms - stillness_start_ms_ >= config_.stillness_window_ms) {
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
            anomaly_active_ = false;
            return true;
        }
        return false;
    }

    void reset() {
        state_ = State::IDLE;
        peak_impact_g_ = 1.0f;
        stillness_start_ms_ = 0;
        pre_alert_start_ms_ = 0;
        anomaly_active_ = false;
    }

    State get_state() const { return state_; }
    bool is_idle() const { return state_ == State::IDLE; }
    bool is_pre_alert() const { return state_ == State::PRE_ALERT; }
    bool is_confirmed_crash() const { return state_ == State::CONFIRMED_CRASH; }
    float get_peak_impact_g() const { return peak_impact_g_; }
    float get_tilt_deg() const { return last_tilt_deg_; }
    float get_roll_deg() const { return roll_deg_; }
    float get_pitch_deg() const { return pitch_deg_; }
    float get_ax() const { return ax_; }
    float get_ay() const { return ay_; }
    float get_az() const { return az_; }
    float get_gx() const { return gx_; }
    float get_gy() const { return gy_; }
    float get_gz() const { return gz_; }
    bool is_anomaly() const { return anomaly_active_; }
    bool is_tilt_warning() const { return tilt_warn_; }
    bool is_rider_upright() const { return last_tilt_deg_ < config_.warn_tilt_deg; }

    const char* state_str() const {
        switch (state_) {
            case State::IDLE: return "IDLE";
            case State::IMPACT_DETECTED: return "IMPACT";
            case State::PRE_ALERT: return "PRE_ALERT";
            case State::CONFIRMED_CRASH: return "CRASH";
            default: return "UNKNOWN";
        }
    }

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
    uint32_t anomaly_until_ms_;
    float peak_impact_g_;
    float last_tilt_deg_;
    float last_a_mag_;
    float ax_, ay_, az_, gx_, gy_, gz_;
    float roll_deg_, pitch_deg_;
    bool anomaly_active_;
    bool tilt_warn_;
};
