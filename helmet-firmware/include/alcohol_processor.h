#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

// ==============================================================================
// MQ-3 ALCOHOL SENSOR PROCESSOR & BREATHALYZER CALIBRATION
// Pure C++ algorithmic class with zero hardware dependencies
// ==============================================================================

struct AlcoholConfig {
    uint32_t warmup_time_ms = 20000;    // 20 seconds pre-heat timer
    float bac_threshold = 0.05f;        // 0.05% BAC threshold (legal limit)
    float v_ref = 3.3f;                 // ESP32 ADC reference voltage
    float divider_factor = 1.5f;        // 10k / 20k divider multiplier (3.0 / 2.0 = 1.5)
    float r_load = 20000.0f;            // 20k load resistor
    float ema_alpha = 0.2f;             // Exponential moving average filter factor
    uint16_t sample_window_ms = 1500;   // Breath blow duration required
};

class AlcoholProcessor {
public:
    using Config = AlcoholConfig;

    AlcoholProcessor() : AlcoholProcessor(AlcoholConfig{}) {}

    explicit AlcoholProcessor(const AlcoholConfig& config)
        : config_(config),
          warmup_start_ms_(0),
          is_warmed_up_(false),
          baseline_r0_(10000.0f),
          baseline_calibrated_(false),
          filtered_adc_(0.0f),
          current_bac_(0.0f),
          blow_start_ms_(0),
          blow_stable_(false) {}

    void start_warmup(uint32_t now_ms) {
        warmup_start_ms_ = now_ms;
        is_warmed_up_ = false;
        filtered_adc_ = 0.0f;
    }

    void update(uint16_t raw_adc, uint32_t now_ms, bool helmet_worn) {
        // 1. Check Warm-up State
        if (!is_warmed_up_) {
            if (now_ms - warmup_start_ms_ >= config_.warmup_time_ms) {
                is_warmed_up_ = true;
                if (!baseline_calibrated_) {
                    calibrate_clean_air(raw_adc);
                }
            }
        }

        // 2. Exponential Moving Average Filter
        if (filtered_adc_ < 1.0f) {
            filtered_adc_ = static_cast<float>(raw_adc);
        } else {
            filtered_adc_ = (config_.ema_alpha * static_cast<float>(raw_adc)) +
                            ((1.0f - config_.ema_alpha) * filtered_adc_);
        }

        // 3. Compute Vout and Sensor Resistance Rs
        // ADC 12-bit (0-4095) mapped to 0-3.3V, then scaled by divider (x 1.5 -> 0-4.95V)
        float v_adc = (filtered_adc_ / 4095.0f) * config_.v_ref;
        float v_sensor = std::min(5.0f, v_adc * config_.divider_factor);

        if (v_sensor <= 0.1f) {
            current_bac_ = 0.0f;
            return;
        }

        // Rs = RL * (Vcc - Vout) / Vout
        float rs = config_.r_load * (5.0f - v_sensor) / v_sensor;
        if (rs < 100.0f) rs = 100.0f;

        // 4. Calculate BAC equivalent
        float ratio = rs / baseline_r0_;
        if (ratio >= 0.95f) {
            current_bac_ = 0.0f;
        } else {
            float bac_est = 0.18f * std::pow(std::max(0.01f, ratio), -1.45f) - 0.18f;
            current_bac_ = std::max(0.0f, std::min(0.50f, bac_est));
        }

        // 5. Blow sample verification
        if (helmet_worn && filtered_adc_ > 250.0f) {
            if (blow_start_ms_ == 0) {
                blow_start_ms_ = now_ms;
            } else if (now_ms - blow_start_ms_ >= config_.sample_window_ms) {
                blow_stable_ = true;
            }
        } else {
            blow_start_ms_ = 0;
            blow_stable_ = false;
        }
    }

    void calibrate_clean_air(uint16_t clean_adc) {
        float v_adc = (static_cast<float>(clean_adc) / 4095.0f) * config_.v_ref;
        float v_sensor = std::max(0.1f, std::min(4.9f, v_adc * config_.divider_factor));
        baseline_r0_ = config_.r_load * (5.0f - v_sensor) / v_sensor;
        if (baseline_r0_ < 1000.0f) baseline_r0_ = 10000.0f;
        baseline_calibrated_ = true;
    }

    void set_baseline_r0(float r0) {
        baseline_r0_ = r0;
        baseline_calibrated_ = true;
    }

    void set_bac_threshold(float threshold) {
        config_.bac_threshold = threshold;
    }

    // --- State Queries ---
    bool is_warming_up() const { return !is_warmed_up_; }
    
    float warmup_progress_pct(uint32_t now_ms) const {
        if (is_warmed_up_) return 100.0f;
        if (config_.warmup_time_ms == 0) return 100.0f;
        uint32_t elapsed = now_ms - warmup_start_ms_;
        if (elapsed >= config_.warmup_time_ms) return 100.0f;
        return (static_cast<float>(elapsed) / static_cast<float>(config_.warmup_time_ms)) * 100.0f;
    }

    float get_bac() const { return current_bac_; }
    float get_filtered_adc() const { return filtered_adc_; }
    float get_baseline_r0() const { return baseline_r0_; }
    bool is_sample_valid() const { return blow_stable_; }

    bool is_over_threshold() const {
        return is_warmed_up_ && (current_bac_ >= config_.bac_threshold);
    }

    void reset_sample() {
        blow_start_ms_ = 0;
        blow_stable_ = false;
    }

private:
    AlcoholConfig config_;
    uint32_t warmup_start_ms_;
    bool is_warmed_up_;
    float baseline_r0_;
    bool baseline_calibrated_;
    float filtered_adc_;
    float current_bac_;
    uint32_t blow_start_ms_;
    bool blow_stable_;
};
