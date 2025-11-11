#include "controllers/NissanModeController.h"
#include <algorithm>

DrivetrainCommand_s NissanModeController::evaluate(const VCRData_s &vcr_data, unsigned long curr_millis)
{
    DrivetrainCommand_s out = {
        {0.0f, 0.0f, 0.0f, 0.0f}, // speed rpms
        {0.0f, 0.0f, 0.0f, 0.0f}  // torques
    };

    const PedalsSystemData_s &pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data; // get pedals data
    const InverterData_s &fl_inverter_data = vcr_data.interface_data.inverter_data.FL;
    const InverterData_s &fr_inverter_data = vcr_data.interface_data.inverter_data.FR;
    const InverterData_s &rl_inverter_data = vcr_data.interface_data.inverter_data.RL;
    const InverterData_s &rr_inverter_data = vcr_data.interface_data.inverter_data.RR;

    // each motor rpm
    float fl_rpm = static_cast<float>(fl_inverter_data.speed_rpm);
    float fr_rpm = static_cast<float>(fr_inverter_data.speed_rpm);
    float rl_rpm = static_cast<float>(rl_inverter_data.speed_rpm);
    float rr_rpm = static_cast<float>(rr_inverter_data.speed_rpm);
    // avg motor rpm
    float avg_rpm = (fl_rpm + fr_rpm + rl_rpm + rr_rpm) / 4.0f; //NOLINT num motors


    // Both pedals are not pressed and no implausibility has been detected
    // accel_request goes between 1.0 and -1.0
    float accel_request = pedals_data.accel_percent - pedals_data.brake_percent;
    float torque_request = 0;

    if (accel_request >= 0.0) {
        // Positive torque request
        torque_request = accel_request * PhysicalParameters::AMK_MAX_TORQUE;

        out.desired_speeds.FL = _max_amk_rpm;
        out.desired_speeds.FR = _max_amk_rpm;
        out.desired_speeds.RL = _max_amk_rpm;
        out.desired_speeds.RR = _max_amk_rpm;

        // calc torque to fl motor
        if (fl_rpm <= avg_rpm) {
            float fl_delta_rpm = (avg_rpm - fl_rpm) / avg_rpm; 
            float fl_compensator = _damping * _last_fl_torque + _rpm_jump_limit * fl_delta_rpm;
            out.torque_limits.FL = std::clamp((torque_request * _front_torque_scale) + fl_compensator, 0.0f, PhysicalParameters::AMK_MAX_TORQUE);

        }
        else if (fl_rpm > avg_rpm * _slip_threshold) {
            float fl_delta_rpm = (fl_rpm - avg_rpm) / avg_rpm;
            float fl_compensator = _damping * _last_fl_torque - _rpm_jump_limit * fl_delta_rpm;
            out.torque_limits.FL = std::clamp((torque_request * _front_torque_scale) + fl_compensator, 0.0f, PhysicalParameters::AMK_MAX_TORQUE);
        }
        else {
            out.torque_limits.FL = torque_request * _front_torque_scale;
        }
        // calc torque to fr motor
        if (fr_rpm <= avg_rpm) {
            float fr_delta_rpm = (avg_rpm - fr_rpm) / avg_rpm; 
            float fr_compensator = _damping * _last_fr_torque + _rpm_jump_limit * fr_delta_rpm;
            out.torque_limits.FR = std::clamp((torque_request * _front_torque_scale) + fr_compensator, 0.0f, PhysicalParameters::AMK_MAX_TORQUE);
        }
        else if (fr_rpm > avg_rpm * _slip_threshold) {
            float fr_delta_rpm = (fr_rpm - avg_rpm) / avg_rpm;
            float fr_compensator = _damping * _last_fr_torque - _rpm_jump_limit * fr_delta_rpm;
            out.torque_limits.FR = std::clamp((torque_request * _front_torque_scale) + fr_compensator, 0.0f, PhysicalParameters::AMK_MAX_TORQUE);
        }
        else {
            out.torque_limits.FR = torque_request * _front_torque_scale;
        }
        // calc torque to rl motor
        if (rl_rpm <= avg_rpm) {
            float rl_delta_rpm = (avg_rpm - rl_rpm) / avg_rpm; 
            float rl_compensator = _damping * _last_rl_torque + _rpm_jump_limit * rl_delta_rpm;
            out.torque_limits.RL = std::clamp((torque_request * _rear_torque_scale) + rl_compensator, 0.0f, PhysicalParameters::AMK_MAX_TORQUE);
        }
        else if (rl_rpm > avg_rpm * _slip_threshold) {
            float rl_delta_rpm = (rl_rpm - avg_rpm) / avg_rpm;
            float rl_compensator = _damping * _last_rl_torque - _rpm_jump_limit * rl_delta_rpm;
            out.torque_limits.RL = std::clamp((torque_request * _rear_torque_scale) + rl_compensator, 0.0f, PhysicalParameters::AMK_MAX_TORQUE);
        }
        else {
            out.torque_limits.RL = torque_request * _rear_torque_scale;
        }
        // calc torque to rr motor
        if (rr_rpm <= avg_rpm) {
            float rr_delta_rpm = (avg_rpm - rr_rpm) / avg_rpm; 
            float rr_compensator = _damping * _last_rr_torque + _rpm_jump_limit * rr_delta_rpm;
            out.torque_limits.RR = std::clamp((torque_request * _rear_torque_scale) + rr_compensator, 0.0f, PhysicalParameters::AMK_MAX_TORQUE);
        }
        else if (rr_rpm > avg_rpm * _slip_threshold) {
            float rr_delta_rpm = (rr_rpm - avg_rpm) / avg_rpm;
            float rr_compensator = _damping * _last_rr_torque - _rpm_jump_limit * rr_delta_rpm;
            out.torque_limits.RR = std::clamp((torque_request * _rear_torque_scale) + rr_compensator, 0.0f, PhysicalParameters::AMK_MAX_TORQUE);
        }
        else {
            out.torque_limits.RR = torque_request * _rear_torque_scale;
        }

        // store last torques
        _last_fl_torque = out.torque_limits.FL;
        _last_fr_torque = out.torque_limits.FR;
        _last_rl_torque = out.torque_limits.RL;
        _last_rr_torque = out.torque_limits.RR;

        return out;
    } else {
        // Negative torque request
        // RPM_TO_METERS_PER_SECOND

        torque_request = _max_amk_regen * accel_request * -1.0F;

        out.desired_speeds = {0.0F, 0.0F, 0.0F, 0.0F};

        out.torque_limits.FL = torque_request * _front_regen_torque_scale;
        out.torque_limits.FR = torque_request * _front_regen_torque_scale;
        out.torque_limits.RL = torque_request * _rear_regen_torque_scale;
        out.torque_limits.RR = torque_request * _rear_regen_torque_scale;

        //reset last torque to zero to be ready for next accel
        _last_fl_torque = 0;
        _last_fr_torque = 0;
        _last_rl_torque = 0;
        _last_rr_torque = 0;
        return out;
    }
}