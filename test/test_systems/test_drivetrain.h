#ifndef __TEST_DRIVETRAIN__
#define __TEST_DRIVETRAIN__
#include "DrivetrainSystem.h"
#include "gtest/gtest.h"

InverterStatus_s FL_status = {};
InverterStatus_s FR_status = {};
InverterStatus_s RL_status = {};
InverterStatus_s RR_status = {};

class MockInverterInterface {

public:

    DrivetrainSystem::InverterFuncts inverter_functs;

    MockInverterInterface(InverterStatus_s &status) : inverter_status(status) {

        using namespace std::placeholders;

        inverter_functs.set_speed =
            std::bind(&MockInverterInterface::set_speed, this, _1, _2);
        inverter_functs.set_torque =
            std::bind(&MockInverterInterface::set_torque, this, _1);
        inverter_functs.set_idle =
            std::bind(&MockInverterInterface::set_idle, this);
        inverter_functs.set_inverter_control_word =
            std::bind(&MockInverterInterface::set_inverter_control_word, this, _1);
        inverter_functs.get_status =
            std::bind(&MockInverterInterface::get_status, this);
    }

private:

    InverterStatus_s &inverter_status;

    void set_speed(float desired_rpm, float torque_limit_nm)
    {
        inverter_status.speed_rpm = desired_rpm;
        return;
    }
    void set_torque(float torque_nm)
    {
        inverter_status.torque_nm = torque_nm;
        return;
    }
    void set_idle()
    {
        inverter_status.speed_rpm = 0;
        inverter_status.torque_nm = 0;
        return;
    }
    void set_inverter_control_word(InverterControlWord_s control_word)
    {
        return;
    }
    InverterStatus_s get_status()
    {
        return inverter_status;
    }

};

MockInverterInterface FL(FL_status);
MockInverterInterface FR(FR_status);
MockInverterInterface RL(RL_status);
MockInverterInterface RR(RR_status);

veh_vec<DrivetrainSystem::InverterFuncts> mock_inverter_functs = {FL.inverter_functs, FR.inverter_functs, RL.inverter_functs, RR.inverter_functs};
DrivetrainSystem drivetrain = DrivetrainSystem(mock_inverter_functs);

DrivetrainInit_s init = {DrivetrainModeRequest_e::INIT_SPEED_MODE};
DrivetrainSystem::CmdVariant cmd = init;

TEST (DrivetrainTest, initial_state) {
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_CONNECTED);
}

TEST (DrivetrainTest, connect_inverters) {
    FL_status.connected = true;
    FR_status.connected = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_CONNECTED);
    RL_status.connected = true;
    RR_status.connected = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
}

TEST (DrivetrainTest, hv_present) {
    FL_status.hv_present = true;
    FR_status.hv_present = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
    RL_status.hv_present = true;
    RR_status.hv_present = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
}

TEST (DrivetrainTest, inverters_ready) {
    FL_status.inverter_ready = true;
    FR_status.inverter_ready = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
    RL_status.inverter_ready = true;
    RR_status.inverter_ready = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_READY);
    FL_status.inverter_ready = false;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
    FL_status.inverter_ready = true;
}

TEST (DrivetrainTest, inverters_hv_enabled) {
    FL_status.quit_dc = true;
    FR_status.quit_dc = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_READY);
    RL_status.quit_dc = true;
    RR_status.quit_dc = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    FL_status.quit_dc = false;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_READY);
    FL_status.quit_dc = true;
}

TEST (DrivetrainTest, inverters_enabled) {
    FL_status.quit_inverter = false;
    FR_status.quit_inverter = false;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    RL_status.quit_inverter = false;
    RR_status.quit_inverter = false;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_ENABLED);
    FL_status.quit_inverter = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    FL_status.quit_inverter = false;
}

TEST (DrivetrainTest, enabling_inverters_speed_mode) {
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_ENABLED);
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::ENABLING_INVERTERS_SPEED_MODE);
}

#endif