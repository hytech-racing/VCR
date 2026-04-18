#include "controllers/LoadCellVectoringTorqueController.h"

#include <algorithm>

DrivetrainCommand_s LoadCellVectoringTorqueController::evaluate(const VCRData_s &vcr_data, unsigned long curr_millis)
{
    DrivetrainCommand_s out = {
        {0.0f, 0.0f, 0.0f, 0.0f}, // speed rpms
        {0.0f, 0.0f, 0.0f, 0.0f}  // torques
    };
    
    const PedalsSystemData_s &pedals_data = vcr_data.interface_data.recvd_pedals_data.pedals_data;
    const FrontLoadCellData_s &front_lc_data = vcr_data.interface_data.front_loadcell_data;
    const RearLoadCellData_s &rear_lc_data = vcr_data.interface_data.rear_loadcell_data;

    float _fz_fl = static_cast<float>(front_lc_data.FL_loadcell_analog) * _fl_load_cell_scale + _fl_load_cell_offset;
    float _fz_fr = static_cast<float>(front_lc_data.FR_loadcell_analog) * _fr_load_cell_scale + _fr_load_cell_offset;
    float _fz_rl = static_cast<float>(rear_lc_data.RL_loadcell_analog) * _rl_load_cell_scale + _rl_load_cell_offset;
    float _fz_rr = static_cast<float>(rear_lc_data.RR_loadcell_analog) * _rr_load_cell_scale + _rr_load_cell_offset;

    veh_vec<float> load_cell_data(static_cast<float>(_fz_fl), 
                                  static_cast<float>(_fz_fr),
                                  static_cast<float>(_fz_rl),  
                                  static_cast<float>(_fz_rr)); 
    
    // Do sanity checks on raw data - FIX
    _load_cell_error_counts.FL = front_lc_data.valid_FL_sample ? 0 : _load_cell_error_counts.FL + 1;
    _load_cell_error_counts.FR = front_lc_data.valid_FR_sample ? 0 : _load_cell_error_counts.FR + 1;
    _load_cell_error_counts.RL = rear_lc_data.valid_RL_sample ? 0 : _load_cell_error_counts.RL + 1;
    _load_cell_error_counts.RR = rear_lc_data.valid_RR_sample ? 0 : _load_cell_error_counts.RR + 1;

    if (_load_cell_error_counts.FL < _max_error_count && _load_cell_error_counts.FR < _max_error_count && _load_cell_error_counts.RL < _max_error_count && _load_cell_error_counts.RR < _max_error_count)
    {
        // Calculate total normal force
        float sum_normal_force = load_cell_data.FL + load_cell_data.FR + load_cell_data.RL + load_cell_data.RR;

        // Both pedals are not pressed and no implausibility has been detected
        // accel_request goes between 1.0 and -1.0
        float accel_request = pedals_data.accel_percent - std::min(1.0f, (_brake_percent_scale * pedals_data.brake_percent));
        float torque_request = 0;

        if (accel_request >= 0.0)
        {
            // Positive torque request  
            torque_request = accel_request * PhysicalParameters::AMK_MAX_TORQUE * 4;
            
            out.desired_speeds.FL = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.FR = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.RL = PhysicalParameters::AMK_MAX_RPM;
            out.desired_speeds.RR = PhysicalParameters::AMK_MAX_RPM;
            
            out.torque_limits.FL = torque_request * _front_torque_bias * load_cell_data.FL / sum_normal_force;
            out.torque_limits.FR = torque_request * _front_torque_bias * load_cell_data.FR / sum_normal_force;
            out.torque_limits.RL = torque_request * _rear_torque_bias * load_cell_data.RL / sum_normal_force;
            out.torque_limits.RR = torque_request * _rear_torque_bias * load_cell_data.RR / sum_normal_force;
        }
        else
        {
            // Negative torque request
            // No load cell vectoring on regen

            // RPM_TO_METERS_PER_SECOND

            torque_request = PhysicalParameters::REGEN_TORQUE_DEFAULT_SCALAR * accel_request * -1.0F;

            out.desired_speeds = {0.0F, 0.0F, 0.0F, 0.0F};

            out.torque_limits.FR = std::min(PhysicalParameters::FRONT_MAX_REGEN_TORQUE, std::max(0.0f, torque_request * _front_regen_torque_bias));
            out.torque_limits.RL = std::min(PhysicalParameters::REAR_MAX_REGEN_TORQUE, std::max(0.0f, torque_request * _rear_regen_torque_bias));
            out.torque_limits.RR = std::min(PhysicalParameters::REAR_MAX_REGEN_TORQUE, std::max(0.0f, torque_request * _rear_regen_torque_bias));
            out.torque_limits.FL = std::min(PhysicalParameters::FRONT_MAX_REGEN_TORQUE, std::max(0.0f, torque_request * _front_regen_torque_bias));
        }
    }

    return out;
}