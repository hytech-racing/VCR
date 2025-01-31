#define TEST_WATCHDOG
#include "gtest/gtest.h"
#include "WatchdogSystem.h"

WatchdogSystem &watchdog = WatchdogSystem::getInstance();
int time_var = 0; // starting time
int time_arb = 2500; // arbitrary number greater than 2000

//Test case where time is 0 millisecs
TEST (WatchdogSystemTesting, initial_state) {
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}

//Test consecutive steps
TEST (WatchdogSystemTesting, next_state1) {
    time_var += 1; // time_var = 1
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state2) {
    time_var += 1; // time_var = 2
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state3) {
    time_var += 1; // time_var = 3
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state4) {
    time_var += 1; // time_var = 4
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state5) {
    time_var += 1; // time_var = 5
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state6) {
    time_var += 1; // time_var = 6
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state7) {
    time_var += 1; // time_var = 7
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state8) {
    time_var += 1; // time_var = 8
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state9) {
    time_var += 1; // time_var = 9
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state10) {
    time_var += 1; // time_var = 10
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), false);
}
TEST (WatchdogSystemTesting, next_state11) {
    time_var += 1; // time_var = 11
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), true);
}
TEST (WatchdogSystemTesting, next_state12) {
    time_var += 1; // time_var = 12
    ASSERT_EQ(watchdog.get_watchdog_state(time_var), true);
}

//Test an arbituary time above 10 millisecs from previous time
TEST (WatchdogSystemTesting, instance_state1) {
    ASSERT_EQ(watchdog.get_watchdog_state(time_arb), false);
}

//Test second arbituary time above 10 millisecs from previous time
TEST (WatchdogSystemTesting, instance_state2) {
    ASSERT_EQ(watchdog.get_watchdog_state(time_arb + 1000), true);
}