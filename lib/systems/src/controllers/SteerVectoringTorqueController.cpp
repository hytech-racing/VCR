#include "controllers/SteerVectoringTorqueController.h"

DrivetrainCommand_s SteerVectoringTorqueController::evaluate(VCFInterface &vcf_interface, unsigned long curr_millis)
{
    DrivetrainCommand_s out = {
        {0.0f, 0.0f, 0.0f, 0.0f}, // speed rpms
        {0.0f, 0.0f, 0.0f, 0.0f}  // torques
    };

    const PedalsSystemData_s &pedals_data = vcf_interface.get_latest_data().stamped_pedals.pedals_data;
    const SteeringSensorData_s &steering_data = vcf_interface.get_latest_data().steering_data;

    float accel_request = pedals_data.accel_percent - pedals_data.brake_percent;
    float torque_request = 0.0f;

    constexpr float total_balance = 2.0;

    float yaw_request = steering_data.analog_steering_degrees; // positive is right turn, negative is left turn

    if (accel_request >= 0.0)
        {
            // Positive torque request
            torque_request = accel_request * PhysicalParameters::AMK_MAX_TORQUE * 4;
            
            out.desired_speeds.FL = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.FR = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.RL = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.RR = PhysicalParameters::AMK_MAX_RPM;
            
        out.torque_limits.FL = torque_request * (total_balance - _rear_torque_scale);
        out.torque_limits.RL = torque_request * _rear_torque_scale;
        
        out.torque_limits.FR = torque_request * (total_balance -_rear_torque_scale);
        out.torque_limits.RR = torque_request * _rear_torque_scale;
        }

        else
        {
            // Negative torque request

            torque_request = PhysicalParameters::MAX_REGEN_TORQUE * accel_request * -1.0F;

            out.desired_speeds = {0.0F, 0.0F, 0.0F, 0.0F};

            out.torque_limits.FL = torque_request * _front_regen_torque_scale;
            out.torque_limits.FR = torque_request * _front_regen_torque_scale;
            out.torque_limits.RL = torque_request * _rear_regen_torque_scale;
            out.torque_limits.RR = torque_request * _rear_regen_torque_scale;
        }



    



    return out;
};