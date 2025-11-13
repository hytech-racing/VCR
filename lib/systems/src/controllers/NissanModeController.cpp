#include "controllers/NissanModeController.h"


DrivetrainCommand_s NissanModeController::evaluate(const VCRData_s &vcr_data, unsigned long curr_millis)
{
    DrivetrainCommand_s out = {
        {0.0f, 0.0f, 0.0f, 0.0f}, // speed rpms
        {0.0f, 0.0f, 0.0f, 0.0f}  // torques
    };

    float max_front_power = PhysicalParameters::AMK_MAX_TORQUE  * _front_torque_scale;
    float max_rear_power = PhysicalParameters::AMK_MAX_TORQUE  * (2 - _front_torque_scale); 

    const PedalsSystemData_s &pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data; // get pedals data
    const InverterData_s &fl_inverter_data = vcr_data.interface_data.inverter_data.FL;
    const InverterData_s &fr_inverter_data = vcr_data.interface_data.inverter_data.FR;
    const InverterData_s &rl_inverter_data = vcr_data.interface_data.inverter_data.RL;
    const InverterData_s &rr_inverter_data = vcr_data.interface_data.inverter_data.RR;

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


        // begin tv logic
        float fl_speed_ms = fl_inverter_data.speed_rpm * RPM_TO_METERS_PER_SECOND; //NOLINT
        float fr_speed_ms = fr_inverter_data.speed_rpm * RPM_TO_METERS_PER_SECOND; // NOLINT
        float rl_speed_ms = rl_inverter_data.speed_rpm * RPM_TO_METERS_PER_SECOND; // NOLINT
        float rr_speed_ms = rr_inverter_data.speed_rpm * RPM_TO_METERS_PER_SECOND; // NOLINT

        float fr_slip_unclamped = ((rl_speed_ms + rr_speed_ms + 250.0) / (fl_speed_ms + fr_speed_ms + 250.0)) - 1 * _fr_slip_factor; //NOLINT
        _fr_slip_clamped = std::clamp(fr_slip_unclamped, 0.0f, 1.0f); // NOLINT

        //set front torques
        _f_torque = 2 * ((1 - _gtr_def_split) * (1 - _fr_slip_clamped) + (1 - _gtr_alt_split) * _fr_slip_clamped) * torque_request;

        //add calculated front torques to the out struct
        out.torque_limits.FL = _f_torque / 2;
        out.torque_limits.FR = _f_torque / 2;

        _r_torque = 2 * ((_gtr_def_split)  * (1 - _fr_slip_clamped) + (_gtr_alt_split) * _fr_slip_clamped) * (torque_request);
        if (rl_speed_ms > rr_speed_ms)
        {
            float rear_lr_slip_unclamped = (rl_speed_ms + 250 / rr_speed_ms + 250) - 1 * _rear_slip_factor; //NOLINT 250 is a factor from original Nissan Mode
            _rear_lr_slip_clamped = std::clamp(rear_lr_slip_unclamped, 0.0f, 0.5f); //NOLINT clamping to ensure our delta is less than for scaling
            _rear_torque_right_split = 0.5 + _rear_lr_slip_clamped;  //NOLINT right side torque split actual number
        }
        else
        {
            float rear_lr_slip_unclamped = (rr_speed_ms + 250 / rl_speed_ms + 250) - 1 * _rear_slip_factor; // NOLINT 250 is a factor from original Nissan Mode
            _rear_lr_slip_clamped = std::clamp(rear_lr_slip_unclamped, 0.0f, 0.5f); //NOLINT clamping to ensure our delta is less than for scaling
            _rear_torque_right_split = 0.5 - _rear_lr_slip_clamped; //NOLINT right side torque split actual number
        }

        //add calculated front torques to the out struct
        out.torque_limits.RL = _r_torque * (1 - _rear_torque_right_split);
        out.torque_limits.RR = _r_torque * (_rear_torque_right_split);

        return out;
    } 
    else 
    {
        // Negative torque request
        // RPM_TO_METERS_PER_SECOND

        torque_request = _max_amk_regen * accel_request * -1.0F;

        out.desired_speeds = {0.0F, 0.0F, 0.0F, 0.0F};

        out.torque_limits.FL = torque_request * _front_regen_torque_scale;
        out.torque_limits.FR = torque_request * _front_regen_torque_scale;
        out.torque_limits.RL = torque_request * _rear_regen_torque_scale;
        out.torque_limits.RR = torque_request * _rear_regen_torque_scale;

        return out;
    }
}