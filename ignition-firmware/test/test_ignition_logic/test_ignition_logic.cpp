#include <unity.h>
#include <string>
#include <iostream>

#include "crypto_utils.h"
#include "safety_fsm.h"

void setUp(void) {}
void tearDown(void) {}

void test_initial_failsafe_locked(void) {
    SafetyFSM fsm;
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::LOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_EQUAL(static_cast<int>(LockReason::NO_LINK), static_cast<int>(fsm.get_lock_reason()));
    TEST_ASSERT_FALSE(fsm.is_ssr_active());
}

void test_preride_conditions_evaluation(void) {
    SafetyFSM fsm;
    HeartbeatInput hb;
    uint32_t now = 1000;

    // 1. Link alive, but helmet not worn
    fsm.register_heartbeat(now);
    hb.worn = false;
    hb.worn_ms = 0;
    fsm.update(hb, now, 12.6f);
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::LOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_EQUAL(static_cast<int>(LockReason::NO_HELMET), static_cast<int>(fsm.get_lock_reason()));
    TEST_ASSERT_FALSE(fsm.is_ssr_active());

    // 2. Helmet worn, but still warming up MQ-3
    hb.worn = true;
    hb.worn_ms = 1500;
    hb.mq3_warm = false;
    fsm.update(hb, now, 12.6f);
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::LOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_EQUAL(static_cast<int>(LockReason::WARMING_UP), static_cast<int>(fsm.get_lock_reason()));

    // 3. Warmed up, but alcohol detected (BAC 0.08 > 0.05)
    hb.mq3_warm = true;
    hb.bac = 0.08f;
    fsm.update(hb, now, 12.6f);
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::LOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_EQUAL(static_cast<int>(LockReason::ALCOHOL_DETECTED), static_cast<int>(fsm.get_lock_reason()));

    // 4. Sober (BAC 0.00), but battery critical (10%)
    hb.bac = 0.00f;
    hb.batt_pct = 10;
    fsm.update(hb, now, 12.6f);
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::LOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_EQUAL(static_cast<int>(LockReason::LOW_BATTERY), static_cast<int>(fsm.get_lock_reason()));

    // 5. ALL 5 PRE-RIDE CONDITIONS SATISFIED -> UNLOCKED!
    hb.batt_pct = 85;
    fsm.update(hb, now, 12.6f);
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::UNLOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_EQUAL(static_cast<int>(LockReason::NONE), static_cast<int>(fsm.get_lock_reason()));
    TEST_ASSERT_TRUE(fsm.is_ssr_active());
}

void test_preride_helmet_removal_relocks(void) {
    SafetyFSM fsm;
    HeartbeatInput hb;
    uint32_t now = 1000;

    fsm.register_heartbeat(now);
    hb.worn = true;
    hb.worn_ms = 2000;
    hb.mq3_warm = true;
    hb.bac = 0.00f;
    hb.batt_pct = 90;

    fsm.update(hb, now, 12.6f);
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::UNLOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_TRUE(fsm.is_ssr_active());

    // Helmet removed before ride starts -> must re-lock
    hb.worn = false;
    hb.worn_ms = 0;
    now += 500;
    fsm.register_heartbeat(now);
    fsm.update(hb, now, 12.6f);

    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::LOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_EQUAL(static_cast<int>(LockReason::NO_HELMET), static_cast<int>(fsm.get_lock_reason()));
    TEST_ASSERT_FALSE(fsm.is_ssr_active());
}

void test_anti_stall_never_cuts_engine_mid_ride(void) {
    SafetyFSM fsm;
    HeartbeatInput hb;
    uint32_t now = 1000;

    // 1. Pre-ride unlock
    fsm.register_heartbeat(now);
    hb.worn = true;
    hb.worn_ms = 2000;
    hb.mq3_warm = true;
    hb.bac = 0.00f;
    hb.batt_pct = 90;
    fsm.update(hb, now, 12.6f);
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::UNLOCKED), static_cast<int>(fsm.get_state()));

    // 2. Rider rides off: speed > 3 km/h -> transitions to RUNNING_LATCHED
    hb.speed_kmh = 25.0f;
    now += 1000;
    fsm.register_heartbeat(now);
    fsm.update(hb, now, 14.1f); // Alternator charging at 14.1V
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::RUNNING_LATCHED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_TRUE(fsm.is_ssr_active());

    // 3. CRITICAL TEST: Link drops mid-ride (no heartbeats for 3 seconds)
    now += 3000;
    // Note: NOT calling fsm.register_heartbeat(now)
    fsm.update(hb, now, 14.1f);

    // ENGINE MUST REMAIN RUNNING (SSR ON)!
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::RUNNING_LATCHED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_TRUE(fsm.is_ssr_active());
    TEST_ASSERT_TRUE(fsm.is_mid_ride_warning());

    // 4. CRITICAL TEST: Rider unfastens helmet mid-ride
    hb.worn = false;
    fsm.register_heartbeat(now);
    fsm.update(hb, now, 14.1f);

    // ENGINE STILL NEVER CUT!
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::RUNNING_LATCHED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_TRUE(fsm.is_ssr_active());
    TEST_ASSERT_TRUE(fsm.is_mid_ride_warning());

    // 5. Confirmed accident crash -> Safety Cutoff permitted
    hb.impact = true;
    fsm.update(hb, now, 14.1f);
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::FAULT), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_FALSE(fsm.is_ssr_active());
}

void test_maintenance_override(void) {
    SafetyFSM fsm;
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::LOCKED), static_cast<int>(fsm.get_state()));

    fsm.apply_maintenance_override(1000, 300); // 300s temporary unlock
    TEST_ASSERT_TRUE(fsm.is_maintenance_active());
    TEST_ASSERT_EQUAL(static_cast<int>(IgnitionState::UNLOCKED), static_cast<int>(fsm.get_state()));
    TEST_ASSERT_TRUE(fsm.is_ssr_active());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_initial_failsafe_locked);
    RUN_TEST(test_preride_conditions_evaluation);
    RUN_TEST(test_preride_helmet_removal_relocks);
    RUN_TEST(test_anti_stall_never_cuts_engine_mid_ride);
    RUN_TEST(test_maintenance_override);

    return UNITY_END();
}
