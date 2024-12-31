#ifndef __TEST_TCMUX__
#define __TEST_TCMUX__
#include "TorqueControllerMux.hpp"
#include "gtest/gtest.h"

// TODO
// - [x] test to ensure that the size checking for desired modes works and failes properly
template <typename quad_array_type>
void set_four_outputs(quad_array_type &out, float val)
{
    out[0] = val;
    out[1] = val;
    out[2] = val;
    out[3] = val;
}

template <typename cmd_type>
void set_outputs(cmd_type &cmd, float mps, float torque)
{
    set_four_outputs(cmd.speeds_rpm, METERS_PER_SECOND_TO_RPM * mps);
    set_four_outputs(cmd.inverter_torque_limit, torque);
}


TEST(TorqueControllerMuxTesting, test_construction)
{
    auto test_func = [](const VCRSystemData_s& state) -> TorqueControllerOutput_s {
        return {};
    };
    TorqueControllerMux<2> test({test_func, test_func}, {false, false});
}

TEST(TorqueControllerMuxTesting, test_invalid_controller_request_error)
{
    
    auto test_func = [](const VCRSystemData_s& state) -> TorqueControllerOutput_s {
        return {};
    };
    TorqueControllerMux<2> test({test_func, nullptr}, {false, false});
    auto res = test.get_drivetrain_command(ControllerMode_e::MODE_2, TorqueLimit_e::TCMUX_FULL_TORQUE, {});
    
    ASSERT_EQ(test.get_tc_mux_status().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS);
    for (int i =0; i< 4; i++)
    {

        ASSERT_EQ(res.speeds_rpm[i], 0.0);
        ASSERT_EQ(res.inverter_torque_limit[i], 0.0);
    }

    res = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, {});

    ASSERT_EQ(test.get_tc_mux_status().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER);
    for (int i =0; i< 4; i++)
    {

        ASSERT_EQ(res.speeds_rpm[i], 0.0);
        ASSERT_EQ(res.inverter_torque_limit[i], 0.0);
    }
}


// ensure that swapping to a controller that has a higher desired output speed than previously
// commanded that we dont switch
TEST(TorqueControllerMuxTesting, test_controller_output_swap_logic)
{
    TorqueControllerOutput_s out1;
    set_outputs(out1.command, 0, 1);
    TorqueControllerOutput_s out2;
    set_outputs(out2.command, 6, 1);
    auto test_func_1 = [&out1](const VCRSystemData_s& state) -> TorqueControllerOutput_s {
        return out1;
    };
    auto test_func_2 = [&out2](const VCRSystemData_s& state) -> TorqueControllerOutput_s {
        return out2;
    };

    TorqueControllerMux<2> test({test_func_1, test_func_2}, {false, false});
    VCRSystemData_s state;
    set_four_outputs(state.drivetrain_data.measuredSpeeds, 10000.0);

    auto res1 = test.get_drivetrain_command(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    res1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    ASSERT_EQ(test.get_tc_mux_status().active_controller_mode, ControllerMode_e::MODE_0);
    ASSERT_EQ(test.get_tc_mux_status().active_error, TorqueControllerMuxError_e::ERROR_SPEED_DIFF_TOO_HIGH);

    set_outputs(out1.command, 0, 1);
    set_outputs(out2.command, 0, 1);
    set_four_outputs(state.drivetrain_data.measuredSpeeds, 0);

    res1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    ASSERT_EQ(test.get_tc_mux_status().active_controller_mode, ControllerMode_e::MODE_1);
    ASSERT_EQ(test.get_tc_mux_status().active_error, TorqueControllerMuxError_e::NO_ERROR);
}

TEST(TorqueControllerMuxTesting, test_torque_diff_swap_limit)
{
    TorqueControllerOutput_s inst1, inst2;
    auto test_func_1 = [&inst1](const VCRSystemData_s& state) -> TorqueControllerOutput_s {
        return inst1;
    };
    auto test_func_2 = [&inst2](const VCRSystemData_s& state) -> TorqueControllerOutput_s {
        return inst2;
    };
    set_outputs(inst1.command, 0.1, 1);
    set_outputs(inst2.command, 3, 10);
    TorqueControllerMux<2> test({test_func_1, test_func_2}, {false, false});
    VCRSystemData_s state;

    state.drivetrain_data = {};

    auto out1 = test.get_drivetrain_command(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    ASSERT_EQ(test.get_tc_mux_status().active_controller_mode, ControllerMode_e::MODE_0);
    ASSERT_EQ(test.get_tc_mux_status().active_error, TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH);

    // tick it a bunch of times
    out1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);
    out1 = test.get_drivetrain_command(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, state);

    ASSERT_EQ(test.get_tc_mux_status().active_error, TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH);

    ASSERT_EQ(test.get_tc_mux_status().active_controller_mode, ControllerMode_e::MODE_0);

    ASSERT_EQ(out1.inverter_torque_limit[0], 1);
    ASSERT_EQ(out1.inverter_torque_limit[1], 1);
    ASSERT_EQ(out1.inverter_torque_limit[2], 1);
    ASSERT_EQ(out1.inverter_torque_limit[3], 1);
}

// TEST(TorqueControllerMuxTesting, test_construction_with_new_controller_orgs)
// {
//     // mode 0
//     TorqueControllerSimple tc_simple(1.0f, 1.0f);
//     // mode 1
//     TorqueControllerLoadCellVectoring tc_vec;
//     // mode 2
//     DummyQueue_s q;
//     CASESystem<DummyQueue_s> case_sys(&q, 100, 70, 550, {});
//     TorqueControllerCASEWrapper<DummyQueue_s> case_wrapper(&case_sys);

//     // mode 3
//     TorqueControllerSimpleLaunch simple_launch;
//     // mode 4
//     TorqueControllerSlipLaunch slip_launch;

//     TorqueControllerMux<5>
//         torque_controller_mux({static_cast<Controller *>(&tc_simple),
//                                static_cast<Controller *>(&tc_vec),
//                                static_cast<Controller *>(&case_wrapper),
//                                static_cast<Controller *>(&simple_launch),
//                                static_cast<Controller *>(&slip_launch)},
//                               {false, false, true, false, false});
// }

// TEST(TorqueControllerMuxTesting, test_mode0_evaluation)
// {

//     float max_torque = 21.42;
//     // mode 0
//     TorqueControllerSimple tc_simple(1.0f, 1.0f);
//     // mode 1
//     TorqueControllerLoadCellVectoring tc_vec;
//     // mode 2
//     DummyQueue_s q;
//     CASESystem<DummyQueue_s> case_sys(&q, 100, 70, 550, {});
//     TorqueControllerCASEWrapper<DummyQueue_s> case_wrapper(&case_sys);

//     // mode 3
//     TorqueControllerSimpleLaunch simple_launch;
//     // mode 4
//     TorqueControllerSlipLaunch slip_launch;
//     TorqueControllerMux<5> torque_controller_mux({static_cast<Controller *>(&tc_simple),
//                                                   static_cast<Controller *>(&tc_vec),
//                                                   static_cast<Controller *>(&case_wrapper),
//                                                   static_cast<Controller *>(&simple_launch),
//                                                   static_cast<Controller *>(&slip_launch)},
//                                                  {false, true, false, false, false});
//     VCRSystemData_s mode_0_input_state({}, {}, {}, {}, {.accelPercent = 0.5f, .brakePercent = 0.0f, .regenPercent = 0.0}, {}, {}, {});

//     DrivetrainCommand_s out = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
//     ASSERT_NEAR(out.inverter_torque_limit[0], (max_torque / 2), 0.01);
//     ASSERT_NEAR(out.inverter_torque_limit[1], (max_torque / 2), 0.01);
//     ASSERT_NEAR(out.inverter_torque_limit[2], (max_torque / 2), 0.01);
//     ASSERT_NEAR(out.inverter_torque_limit[3], (max_torque / 2), 0.01);

//     mode_0_input_state = {{}, {}, {}, {}, {.accelPercent = 0.0f, .brakePercent = 0.0f, .regenPercent = 0.0}, {}, {}, {}};
//     out = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);
//     ASSERT_EQ(out.inverter_torque_limit[0], 0);
//     ASSERT_EQ(out.inverter_torque_limit[1], 0);
//     ASSERT_EQ(out.inverter_torque_limit[2], 0);
//     ASSERT_EQ(out.inverter_torque_limit[3], 0);

//     // out = torque_controller_mux.getDrivetrainCommand(ControllerMode_e::MODE_1, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_1_input_state);
// }

// TEST(TorqueControllerMuxTesting, test_power_limit)
// {
//     TestControllerType inst1;
//     set_output_rpm(inst1, 20000, 10.0);
//     TorqueControllerMux<1> test({static_cast<Controller *>(&inst1)}, {false});

//     DrivetrainDynamicReport_s drivetrain_data = {};
//     for (int i = 0; i < 4; i++)
//     {
//         drivetrain_data.measuredSpeeds[i] = 500.0f;
//     }
//     VCRSystemData_s mode_0_input_state({}, {}, drivetrain_data, {}, {.accelPercent = 0.5f, .brakePercent = 0.0f, .regenPercent = 0.0}, {}, {} , {});

//     DrivetrainCommand_s res = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state);

//     for (int i = 0; i < 4; i++)
//     {
//         ASSERT_EQ(res.inverter_torque_limit[i], 10.0f);
//     }
//     for (int i = 0; i < 4; i++)
//     {
//         drivetrain_data.measuredSpeeds[i] = 20000.0f;
//     }
//     set_output_rpm(inst1, 20000, 21.0);

//     VCRSystemData_s mode_0_input_state_high_power({}, {}, drivetrain_data, {}, {.accelPercent = 1.0f, .brakePercent = 0.0f, .regenPercent = 0.0}, {}, {}, {});
//     res = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_FULL_TORQUE, mode_0_input_state_high_power);

//     for (int i = 0; i < 4; i++)
//     {
//         ASSERT_LT(res.inverter_torque_limit[i], 7.6); // hardcoded value based on online calculator
//     }
// }

// TEST(TorqueControllerMuxTesting, test_torque_limit)
// {

//     TestControllerType inst1;

//     set_output_rpm(inst1, 500, 10.0);
//     inst1.output.command.inverter_torque_limit[0] = 5;
//     TorqueControllerMux<1> test({static_cast<Controller *>(&inst1)}, {false});

//     DrivetrainDynamicReport_s drivetrain_data = {};
//     for (int i = 0; i < 4; i++)
//     {
//         drivetrain_data.measuredSpeeds[i] = 500.0f;
//     }

//     VCRSystemData_s mode_0_input_state({}, {}, drivetrain_data, {}, {.accelPercent = 0.5f, .brakePercent = 0.0f, .regenPercent = 0.0}, {}, {}, {});

//     auto drive_command = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_LOW_TORQUE, mode_0_input_state);

//     ASSERT_EQ(drive_command.inverter_torque_limit[0], 5.0f);
//     ASSERT_EQ(drive_command.inverter_torque_limit[1], 10.0f);
//     ASSERT_EQ(drive_command.inverter_torque_limit[2], 10.0f);
//     ASSERT_EQ(drive_command.inverter_torque_limit[3], 10.0f);

//     set_output_rpm(inst1, 500, 20.0);
//     inst1.output.command.inverter_torque_limit[0] = 5;

//     drive_command = test.getDrivetrainCommand(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_LOW_TORQUE, mode_0_input_state);

//     ASSERT_LT(drive_command.inverter_torque_limit[0], 3.5f);
//     ASSERT_LT(drive_command.inverter_torque_limit[1], 12.5f);
//     ASSERT_LT(drive_command.inverter_torque_limit[2], 12.5f);
//     ASSERT_LT(drive_command.inverter_torque_limit[3], 12.5f);
    
//     printf("torque 1: %.2f\n", drive_command.inverter_torque_limit[0]);
//     printf("torque 2: %.2f\n", drive_command.inverter_torque_limit[1]);
//     printf("torque 3: %.2f\n", drive_command.inverter_torque_limit[2]);
//     printf("torque 4: %.2f\n", drive_command.inverter_torque_limit[3]);
// }

TEST(TorqueControllerMuxTesting, test_null_pointer_error_state)
{
    TorqueControllerMux<1> test({nullptr}, {true});
    VCRSystemData_s state;
    auto res = test.get_drivetrain_command(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_LOW_TORQUE, state);
    for (int i = 0; i < 4; i++)
    {
        ASSERT_EQ(res.inverter_torque_limit[i], 0.0f);
        ASSERT_EQ(res.speeds_rpm[i], 0.0f);
    }
    ASSERT_EQ(test.get_tc_mux_status().active_error, TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER);
}
#endif // __TEST_TCMUX__