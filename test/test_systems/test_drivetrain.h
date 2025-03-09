#ifndef __TEST_DRIVETRAIN__
#define __TEST_DRIVETRAIN__
#include "DrivetrainSystem.h"
#include "gtest/gtest.h"
#include "shared_types.h"

InverterStatus_s FL_status = {};
InverterStatus_s FR_status = {};
InverterStatus_s RL_status = {};
InverterStatus_s RR_status = {};

MotorMechanics_s FL_motor_mechanics = {};
MotorMechanics_s FR_motor_mechanics = {};
MotorMechanics_s RL_motor_mechanics = {};
MotorMechanics_s RR_motor_mechanics = {};

class MockInverterInterface {

public:

    DrivetrainSystem::InverterFuncts inverter_functs;

    MockInverterInterface(InverterStatus_s &status, MotorMechanics_s &m_mech)
    : inverter_status(status), motor_mechanics(m_mech) {

        using namespace std::placeholders;

        inverter_functs.set_speed =
            std::bind(&MockInverterInterface::set_speed, this, _1, _2);
        inverter_functs.set_idle =
            std::bind(&MockInverterInterface::set_idle, this);
        inverter_functs.set_inverter_control_word =
            std::bind(&MockInverterInterface::set_inverter_control_word, this, _1);
        inverter_functs.get_status =
            std::bind(&MockInverterInterface::get_status, this);
        inverter_functs.get_motor_mechanics =
            std::bind(&MockInverterInterface::get_motor_mechanics, this);
    }

private:

    InverterStatus_s &inverter_status;
    MotorMechanics_s &motor_mechanics;

    void set_speed(float desired_rpm, float torque_limit_nm)
    {
        // inverter_status.speed_rpm = desired_rpm;
        return;
    }
    void set_idle()
    {
        // inverter_status.speed_rpm = 0;
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
    MotorMechanics_s get_motor_mechanics()
    {
        return motor_mechanics;
    }

};

MockInverterInterface FL(FL_status, FL_motor_mechanics);
MockInverterInterface FR(FR_status, FR_motor_mechanics);
MockInverterInterface RL(RL_status, RL_motor_mechanics);
MockInverterInterface RR(RR_status, RR_motor_mechanics);

veh_vec<DrivetrainSystem::InverterFuncts> mock_inverter_functs = {FL.inverter_functs, FR.inverter_functs, RL.inverter_functs, RR.inverter_functs};
DrivetrainSystem drivetrain = DrivetrainSystem(mock_inverter_functs);

DrivetrainInit_s init = {DrivetrainModeRequest_e::INIT_DRIVE_MODE};
DrivetrainResetError_s reset = {true};
DrivetrainSystem::CmdVariant cmd = init;

TEST (DrivetrainTest, test) {
    
    FL_status.hv_present = false;
    FR_status.hv_present = false;
    RL_status.hv_present = false;
    RR_status.hv_present = false;

    FL_status.connected = false;
    FR_status.connected = false;
    RL_status.connected = false;
    RR_status.connected = false;

    FL_status.quit_dc_on = false;
    FR_status.quit_dc_on = false;
    RL_status.quit_dc_on = false;
    RR_status.quit_dc_on = false;

    FL_status.quit_inverter_on = false;
    FR_status.quit_inverter_on = false;
    RL_status.quit_inverter_on = false;
    RR_status.quit_inverter_on = false;

    FL_status.error = false;
    FR_status.error = false;
    RL_status.error = false;
    RR_status.error = false;
    
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_CONNECTED);
    
    FL_status.connected = true;
    FR_status.connected = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_CONNECTED);
    RL_status.connected = true;
    RR_status.connected = true;

    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);

    FL_status.hv_present = true;
    FR_status.hv_present = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
    RL_status.hv_present = true;
    RR_status.hv_present = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);

    FL_status.system_ready = true;
    FR_status.system_ready = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
    RL_status.system_ready = true;
    RR_status.system_ready = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_READY);
    FL_status.system_ready = false;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
    FL_status.system_ready = true;

    FL_status.quit_dc_on = true;
    FR_status.quit_dc_on = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_READY);
    RL_status.quit_dc_on = true;
    RR_status.quit_dc_on = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    FL_status.quit_dc_on = false;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_READY);
    FL_status.quit_dc_on = true;
    FR_status.quit_dc_on = true;
    RL_status.quit_dc_on = true;
    RR_status.quit_dc_on = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    drivetrain.evaluate_drivetrain(cmd);
    FL_status.quit_inverter_on = true;
    FR_status.quit_inverter_on = true;
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_HV_ENABLED);
    RL_status.quit_inverter_on = true;
    RR_status.quit_inverter_on = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::INVERTERS_ENABLED);
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::ENABLED_DRIVE_MODE);

    FL_status.error = true;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::ERROR);
    FL_status.error = false;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::ERROR);

    cmd = reset;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::CLEARING_ERRORS);
    cmd = init;
    drivetrain.evaluate_drivetrain(cmd);
    ASSERT_EQ(drivetrain.get_state(), DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
}

#endif