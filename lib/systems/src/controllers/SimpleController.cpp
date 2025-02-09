#include "Controllers/SimpleController.h"

TorqueControllerOutput_s TorqueControllerSimple::evaluate(const VCRSystemData_s &state);
{

    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0

    TorqueControllerOutput_s output;
    float accelRequest = pedalsData.accelPercent - pedalsData.regenPercent;
    float torqueRequest;

    constexpr float balance = 2.0;

    if (accelRequest >= 0.0)
    {
        // Positive torque request
        torqueRequest = accelRequest * PhysicalParameters::AMK_MAX_TORQUE;

        writeout_.command.speeds_rpm[FL] = PhysicalParameters::AMK_MAX_RPM;
        writeout_.command.speeds_rpm[FR] = PhysicalParameters::AMK_MAX_RPM;
        writeout_.command.speeds_rpm[RL] = PhysicalParameters::AMK_MAX_RPM;
        writeout_.command.speeds_rpm[RR] = PhysicalParameters::AMK_MAX_RPM;
        
        writeout_.command.inverter_torque_limit[FL] = torqueRequest * (balance - _params.rear_torque_scale);
        writeout_.command.inverter_torque_limit[FR] = torqueRequest * (balance - _params.rear_torque_scale);
        writeout_.command.inverter_torque_limit[RL] = torqueRequest * _params.rear_torque_scale;
        writeout_.command.inverter_torque_limit[RR] = torqueRequest * _params.rear_torque_scale;
    }
    else
    {
        // Negative torque request
        torqueRequest = PhysicalParameters::MAX_REGEN_TORQUE * accelRequest * -1.0;

        writeout_.command.speeds_rpm[FL] = 0.0;
        writeout_.command.speeds_rpm[FR] = 0.0;
        writeout_.command.speeds_rpm[RL] = 0.0;
        writeout_.command.speeds_rpm[RR] = 0.0;

        writeout_.command.inverter_torque_limit[FL] = torqueRequest * frontRegenTorqueScale_;
        writeout_.command.inverter_torque_limit[FR] = torqueRequest * frontRegenTorqueScale_;
        writeout_.command.inverter_torque_limit[RL] = torqueRequest * rearRegenTorqueScale_;
        writeout_.command.inverter_torque_limit[RR] = torqueRequest * rearRegenTorqueScale_;
    }
}

TorqueControllerOutput_s TorqueControllerSimple::evaluate(const SharedCarState_s &state)
{
    tick(state.pedals_data);
    return writeout_;
}