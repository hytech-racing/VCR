#ifndef __TEST_DRIVEBRAIN_CONTROLLER_H__
#define __TEST_DRIVEBRAIN_CONTROLLER_H__

#include "SharedFirmwareTypes.h"
#include "controllers/DrivebrainController.h"
#include "controllers/SimpleController.h"
#include <gtest/gtest.h>

auto runTick(DrivebrainController *controller, float last_speed_recv_millis,
             float last_torque_receive_time_millis, ControllerMode_e current_control_mode,
             unsigned long curr_millis, float brakePercent, float accelPercent, bool reset_button_pressed = false) {
    StampedDrivetrainTorqueCommand_s data;
    data.torque_setpoints.last_recv_millis = last_torque_receive_time_millis;
    data.torque_setpoints.recvd = true;
    data.torque_setpoints.veh_vec_data = {1, 1, 1, 1};

    TorqueControllerMuxStatus_s status = {};
    status.active_controller_mode = current_control_mode;
    PedalsSystemData_s pedals_data = {};
    pedals_data.brake_percent = brakePercent;
    pedals_data.accel_percent = accelPercent;

    VCRData_s state;
    state.interface_data.dash_input_state.data_btn_is_pressed = reset_button_pressed;
    state.interface_data.latest_drivebrain_command = data;
    state.interface_data.recvd_pedals_data.pedals_data = pedals_data;

    return controller->evaluate(state, curr_millis);
}

TEST(DrivebrainControllerTesting, signals_sent_within_range) {
    DrivebrainController controller(10);
    auto torque_controller_output_s =
        runTick(&controller, 998, 1001, ControllerMode_e::MODE_4, 1002, 0.0f, 0.0f);
    EXPECT_FALSE(controller.get_timing_failure_status());
    EXPECT_FLOAT_EQ(torque_controller_output_s.torque_limits.FL, 1);
}

TEST(DrivebrainControllerTesting, setpoint_too_latent_still_in_control) {
    DrivebrainController controller(5);
    auto torque_controller_output_s =
        runTick(&controller, 800, 1006, ControllerMode_e::MODE_4, 1012, 1.0f, 0.0f);
    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, 0);

    TorqueControllerSimpleParams_s params_def;
    EXPECT_FLOAT_EQ(torque_controller_output_s.torque_limits.FL, params_def.amk_max_regen_torque);
}

TEST(DrivebrainControllerTesting, failing_stay_failing) {
    DrivebrainController controller(10);
    auto torque_controller_output_s =
        runTick(&controller, 200, 1011, ControllerMode_e::MODE_4, 1032, 0.01f, 0.0f);
    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, 0);

    torque_controller_output_s =
        runTick(&controller, 200, 1033, ControllerMode_e::MODE_4, 1033, 0.01f, 0.0f);
    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, 0);

    torque_controller_output_s =
        runTick(&controller, 400, 1034, ControllerMode_e::MODE_4, 1034, 0.01f, 0.0f);
    EXPECT_TRUE(controller.get_timing_failure_status());
    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, 0);
}

TEST(DrivebrainControllerTesting, failing_in_control) {
    
    TorqueControllerSimpleParams_s params_def; // has default vals for members of the struct
    DrivebrainController controller(10);
    auto torque_controller_output_s =
        runTick(&controller, 200, 1011, ControllerMode_e::MODE_4, 1032, 0.01f, 0.0f);
    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, 0);

    torque_controller_output_s =
        runTick(&controller, 200, 1033, ControllerMode_e::MODE_4, 1033, 0.01f, 0.0f);
    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, 0);

    torque_controller_output_s =
        runTick(&controller, 400, 1034, ControllerMode_e::MODE_4, 1034, 0.01f, 0.0f);
    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, 0);

    torque_controller_output_s =
        runTick(&controller, 400, 1034, ControllerMode_e::MODE_4, 1034, 0.0f, 1.0f);

    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, params_def.amk_max_rpm);

    EXPECT_TRUE(controller.get_timing_failure_status());

    EXPECT_FLOAT_EQ(torque_controller_output_s.torque_limits.FL, params_def.amk_max_torque);
}

TEST(DrivebrainControllerTesting, failing_reset_success) {
    DrivebrainController controller(10);
    auto torque_controller_output_s =
        runTick(&controller, 300, 1011, ControllerMode_e::MODE_4, 1022, 0.01f, 0.0f);
    EXPECT_FLOAT_EQ(torque_controller_output_s.desired_speeds.FL, 0);

    EXPECT_TRUE(controller.get_timing_failure_status());
    torque_controller_output_s =
        runTick(&controller, 1020, 1021, ControllerMode_e::MODE_4, 1023, 0.01f, 0, true);
    EXPECT_FALSE(controller.get_timing_failure_status());
    EXPECT_FLOAT_EQ(torque_controller_output_s.torque_limits.FL, 1);
}

#endif // __TEST_DRIVEBRAIN_CONTROLLER_H__