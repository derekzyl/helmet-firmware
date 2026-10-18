#include <unity.h>
#include <string>
#include <iostream>

#include "crypto_utils.h"
#include "alcohol_processor.h"
#include "collision_detector.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

// ==============================================================================
// 1. CRYPTO UTILS TESTS
// ==============================================================================

void test_sha256_empty_string(void) {
    crypto::SHA256 sha;
    sha.update("");
    std::string digest = sha.digest_hex();
    TEST_ASSERT_EQUAL_STRING("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", digest.c_str());
}

void test_hmac_sha256_rfc_vector(void) {
    std::string key = "key";
    std::string data = "The quick brown fox jumps over the lazy dog";
    std::string hmac = crypto::hmac_sha256_hex(key, data);
    TEST_ASSERT_EQUAL_STRING("f7bc83f430538424b13298e6aa6fb143ef4d59a14946175997479dbc2d1a3cd8", hmac.c_str());
}

void test_hmac_verification_and_tampering(void) {
    std::string psk = "d4a8f9c1e2b3a4c5d6e7f8091a2b3c4d";
    std::string frame = "1:HEARTBEAT:HLM-001:100:1776500000:worn=1,bac=0.00";
    std::string sig = crypto::hmac_sha256_hex(psk, frame);

    // Valid check
    TEST_ASSERT_TRUE(crypto::verify_hmac(psk, frame, sig));

    // Tampered payload
    std::string tampered_frame = "1:HEARTBEAT:HLM-001:100:1776500000:worn=0,bac=0.00";
    TEST_ASSERT_FALSE(crypto::verify_hmac(psk, tampered_frame, sig));

    // Tampered key
    std::string wrong_key = "wrong_psk_key_123456789012345678";
    TEST_ASSERT_FALSE(crypto::verify_hmac(wrong_key, frame, sig));
}

// ==============================================================================
// 2. ALCOHOL PROCESSOR TESTS
// ==============================================================================

void test_alcohol_warmup_and_calibration(void) {
    AlcoholProcessor::Config cfg;
    cfg.warmup_time_ms = 5000; // 5s for fast test
    cfg.bac_threshold = 0.05f;

    AlcoholProcessor proc(cfg);
    proc.start_warmup(1000);

    TEST_ASSERT_TRUE(proc.is_warming_up());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, proc.warmup_progress_pct(1000));

    // Halfway through warmup
    proc.update(400, 3500, true);
    TEST_ASSERT_TRUE(proc.is_warming_up());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 50.0f, proc.warmup_progress_pct(3500));

    // Warmup completed
    proc.update(400, 6001, true);
    TEST_ASSERT_FALSE(proc.is_warming_up());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, proc.warmup_progress_pct(6001));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, proc.get_bac());
    TEST_ASSERT_FALSE(proc.is_over_threshold());
}

void test_alcohol_over_threshold_detection(void) {
    AlcoholProcessor::Config cfg;
    cfg.warmup_time_ms = 1000;
    cfg.bac_threshold = 0.05f;

    AlcoholProcessor proc(cfg);
    proc.start_warmup(0);
    proc.update(400, 1001, true); // warm up & calibrate baseline

    // Clean air baseline
    TEST_ASSERT_FALSE(proc.is_over_threshold());

    // High alcohol concentration (raw ADC shoots up to 2800)
    for (int i = 0; i < 10; ++i) {
        proc.update(2800, 2000 + (i * 100), true);
    }

    TEST_ASSERT_TRUE(proc.get_bac() > 0.05f);
    TEST_ASSERT_TRUE(proc.is_over_threshold());
}

// ==============================================================================
// 3. COLLISION DETECTOR TESTS
// ==============================================================================

void test_collision_normal_riding_no_false_alarm(void) {
    CollisionDetector det;

    // Normal 1G riding with upright orientation (az = 1.0G, no gyro motion)
    for (uint32_t t = 0; t < 5000; t += 100) {
        det.update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, t);
        TEST_ASSERT_TRUE(det.is_idle());
    }

    // Pothole shock (3.0G instant shock, but below 4.0G threshold)
    det.update(0.0f, 0.0f, 3.0f, 5.0f, 5.0f, 0.0f, 5100);
    TEST_ASSERT_TRUE(det.is_idle());
}

void test_collision_cancel_window_and_confirmation(void) {
    CollisionDetector::Config cfg;
    cfg.impact_threshold_g = 4.0f;
    cfg.tilt_threshold_deg = 60.0f;
    cfg.stillness_window_ms = 2000;
    cfg.cancel_window_ms = 10000; // 10s cancel window

    CollisionDetector det(cfg);

    // 1. Initial High G Impact: 5.5G
    det.update(2.0f, 3.0f, 4.0f, 150.0f, 200.0f, 80.0f, 1000);
    TEST_ASSERT_EQUAL(static_cast<int>(CollisionDetector::State::IMPACT_DETECTED), static_cast<int>(det.get_state()));

    // 2. Fallen tilt orientation (ax=0.9, ay=0.3, az=0.1 -> tilt > 80 deg) + stillness
    det.update(0.95f, 0.2f, 0.1f, 2.0f, 1.0f, 0.5f, 1500);
    det.update(0.95f, 0.2f, 0.1f, 1.0f, 0.5f, 0.5f, 3200);

    // Stillness window elapsed (1500 to 3600 >= 2000ms)
    det.update(0.95f, 0.2f, 0.1f, 0.5f, 0.5f, 0.2f, 3600);
    TEST_ASSERT_EQUAL(static_cast<int>(CollisionDetector::State::PRE_ALERT), static_cast<int>(det.get_state()));
    TEST_ASSERT_TRUE(det.is_pre_alert());

    // 3. Test Button A Cancel
    bool cancelled = det.cancel_alarm();
    TEST_ASSERT_TRUE(cancelled);
    TEST_ASSERT_TRUE(det.is_idle());

    // 4. Test Uncancelled Crash Confirmation
    det.update(2.0f, 3.0f, 4.0f, 150.0f, 200.0f, 80.0f, 10000); // New impact at t=10000
    det.update(0.95f, 0.2f, 0.1f, 0.5f, 0.5f, 0.2f, 10500);     // Stillness start
    det.update(0.95f, 0.2f, 0.1f, 0.5f, 0.5f, 0.2f, 12600);     // Pre-alert entered at t=12600
    TEST_ASSERT_TRUE(det.is_pre_alert());

    // Wait until cancel window expires (10s later -> t=22700)
    det.update(0.95f, 0.2f, 0.1f, 0.5f, 0.5f, 0.2f, 22700);
    TEST_ASSERT_TRUE(det.is_confirmed_crash());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_sha256_empty_string);
    RUN_TEST(test_hmac_sha256_rfc_vector);
    RUN_TEST(test_hmac_verification_and_tampering);

    RUN_TEST(test_alcohol_warmup_and_calibration);
    RUN_TEST(test_alcohol_over_threshold_detection);

    RUN_TEST(test_collision_normal_riding_no_false_alarm);
    RUN_TEST(test_collision_cancel_window_and_confirmation);

    return UNITY_END();
}
