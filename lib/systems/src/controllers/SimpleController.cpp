#include "controllers/SimpleController.h"

DrivetrainCommand_s TorqueControllerSimple::evaluate(const VCRData_s &state)
{
    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0
    DrivetrainCommand_s out;
    
<<<<<<< HEAD
    float accelRequest = state.interface_data.recvd_pedals_data.pedals_data.accel_percent - state.interface_data.recvd_pedals_data.pedals_data.brake_percent;
=======
    float accelRequest = state.pedals_system_data.accel_percent - state.pedals_system_data.brake_percent;
>>>>>>> 41e4c8a3bbee440d05141335850886d1f5ae3a9d
    float torqueRequest = 0.0f;

    constexpr float balance = 2.0;

    if (accelRequest >= 0.0)
    {
        // Positive torque request
        torqueRequest = accelRequest * _params.amk_max_torque;

        out.desired_speeds = {_params.amk_max_rpm, _params.amk_max_rpm, _params.amk_max_rpm, _params.amk_max_rpm};
        out.torque_limits.FL = torqueRequest * (balance - _params.rear_torque_scale);
        out.torque_limits.FR = torqueRequest * (balance - _params.rear_torque_scale);
        out.torque_limits.RL = torqueRequest * _params.rear_torque_scale;
        out.torque_limits.RR = torqueRequest * _params.rear_torque_scale;
    }
    else
    {
        // regen torque request
        torqueRequest = _params.amk_max_regen_torque * accelRequest * -1.0;

        out.desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f};
        
        out.torque_limits.FL = torqueRequest * (balance - _params.rear_regen_torque_scale);
        out.torque_limits.FR = torqueRequest * (balance - _params.rear_regen_torque_scale);
        out.torque_limits.RL = torqueRequest * _params.rear_regen_torque_scale;
        out.torque_limits.RR = torqueRequest * _params.rear_regen_torque_scale;
    }

    return out;
}