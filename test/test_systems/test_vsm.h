#include "VehicleStateMachine.h"
#include <gtest/gtest.h>

bool hv_over_threshold;
bool start_btn;
bool brake_pressed;
bool drivetrain_error;
bool drivetrain_ready;
bool drivetrain_commanded;
bool buzzer_active;
bool pedals_timeout;

etl::delegate<bool()> mock_hv_over_threshold = etl::delegate<bool()>::create([]() -> bool {
    return hv_over_threshold;
});

etl::delegate<bool()> mock_start_btn = etl::delegate<bool()>::create([]() -> bool {
    return brake_pressed;
});

etl::delegate<bool()> mock_brake_pressed = etl::delegate<bool()>::create([]() -> bool {
    return brake_pressed;
});

etl::delegate<bool()> mock_drivetrain_error = etl::delegate<bool()>::create([]() -> bool {
    return drivetrain_error;
});

etl::delegate<bool()> mock_drivetrain_ready = etl::delegate<bool()>::create([]() -> bool {
    return drivetrain_ready;
});

etl::delegate<void()> mock_start_buzzer = etl::delegate<void()>::create([]() -> void {
    buzzer_active = true;
});

etl::delegate<bool()> mock_buzzer_done = etl::delegate<bool()>::create([]() -> bool {
    return !buzzer_active;
});

etl::delegate<void()> mock_end_buzzer = etl::delegate<void()>::create([]() -> void {
    buzzer_active = false;
});

etl::delegate<void()> mock_handle_drivetrain_init = etl::delegate<void()>::create([]() -> void {
    return;
});

etl::delegate<void()> mock_command_drivetrain = etl::delegate<void()>::create([]() -> void {
    drivetrain_commanded = true;
    return;
});

etl::delegate<bool()> mock_pedals_timeout = etl::delegate<bool()>::create([]() -> bool {
    return pedals_timeout;
});

etl::delegate<void()> mock_pedals_reset = etl::delegate<void()>::create([]() -> void {
    pedals_timeout = false;
    return;
});

VehicleStateMachine state_machine = VehicleStateMachine(
    mock_hv_over_threshold,
    mock_start_btn,
    mock_brake_pressed,
    mock_drivetrain_error,
    mock_drivetrain_ready,
    mock_start_buzzer,
    mock_buzzer_done,
    mock_end_buzzer,
    mock_handle_drivetrain_init,
    mock_command_drivetrain,
    mock_pedals_timeout,
    mock_pedals_reset
);

TEST (VehicleStateMachine, TractiveSystemNotActive) {
    hv_over_threshold = false;
    start_btn = false;
    brake_pressed = false;
    drivetrain_error = false;
    drivetrain_ready = false;
    drivetrain_commanded = false;
    buzzer_active = false;

    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    state_machine.tick_state_machine(0);
}

TEST (VehicleStateMachine, TractiveSystemActive) {
    hv_over_threshold = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    state_machine.tick_state_machine(0);

    hv_over_threshold = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    state_machine.tick_state_machine(0);

    hv_over_threshold = true;
}

TEST (VehicleStateMachine, WantingReadyToDrive) {
    start_btn = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    state_machine.tick_state_machine(0);
    brake_pressed = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::READY_TO_DRIVE);
    state_machine.tick_state_machine(0);

    hv_over_threshold = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
    state_machine.tick_state_machine(0);

    hv_over_threshold = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    state_machine.tick_state_machine(0);
    
    brake_pressed = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::READY_TO_DRIVE);
    ASSERT_EQ(buzzer_active, true);
    state_machine.tick_state_machine(0);
}

TEST (VehicleStateMachine, ReadyToDrive) {
    buzzer_active = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::READY_TO_DRIVE);
    state_machine.tick_state_machine(0);
    ASSERT_EQ(drivetrain_commanded, true);
    state_machine.tick_state_machine(0);

    drivetrain_error = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_ACTIVE);
    state_machine.tick_state_machine(0);
    buzzer_active = false;
    drivetrain_ready = true;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::READY_TO_DRIVE);

    hv_over_threshold = false;
    state_machine.tick_state_machine(0);
    ASSERT_EQ(state_machine.get_state(), VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE);
}