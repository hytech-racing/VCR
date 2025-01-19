#include "VehicleStateMachine.h"
#include "gtest/gtest.h"
#include "DrivetrainSystem.h"

static VehicleStateMachine vsm = VehicleStateMachine::getInstance();
static DrivetrainSystem<uint32_t> drivetrain;
VCRSystemData_s system_data;

TEST (VehicleStateMachineTesting, initial_state)
{
    ASSERT_EQ(vsm.get_state(), CAR_STATE::STARTUP);
}

TEST (VehicleStateMachineTesting, state_after_tick)
{
    vsm.tick_state_machine(0, system_data);
    ASSERT_EQ(vsm.get_state(), CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
}

/*
TEST (VehicleStateMachineTesting, tractive_system_activation)
{
    vsm.tick_state_machine(0, system_data);
    while (!drivetrain.hv_over_threshold_on_drivetrain()) {
        ASSERT_EQ(vsm.get_state(), CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
        vsm.tick_state_machine(0, system_data);
    }   // should this time out eventually?
    
    ASSERT_EQ(vsm.get_state(), CAR_STATE::TRACTIVE_SYSTEM_ACTIVE);
}

TEST (VehicleStateMachineTesting, brake_wo_start)
{
    system_data.pedals_system_data.brake_is_pressed = 1;
    system_data.dash_input_state.start_btn_is_pressed = 0;
    vsm.tick_state_machine(0, system_data);

    ASSERT_EQ(vsm.get_state(), CAR_STATE::TRACTIVE_SYSTEM_ACTIVE);
}

TEST (VehicleStateMachineTesting, start_wo_brake)
{
    system_data.pedals_system_data.brake_is_pressed = 0;
    system_data.dash_input_state.start_btn_is_pressed = 1;
    vsm.tick_state_machine(0, system_data);

    ASSERT_EQ(vsm.get_state(), CAR_STATE::TRACTIVE_SYSTEM_ACTIVE);
}

TEST (VehicleStateMachineTesting, car_startup)
{
    system_data.pedals_system_data.brake_is_pressed = 1;
    system_data.dash_input_state.start_btn_is_pressed = 1;
    vsm.tick_state_machine(0, system_data);

    ASSERT_EQ(vsm.get_state(), CAR_STATE::ENABLING_INVERTERS);
}

TEST (VehicleStateMachineTesting, enable_inverters)
{
    vsm.tick_state_machine(0, system_data);
    ASSERT_EQ(vsm.get_state(), CAR_STATE::WAITING_READY_TO_DRIVE_SOUND);
}

*/