#include "SharedFirmwareTypes.h"
#include "TorqueControllerMux.hpp"
#include "PhysicalParameters.h"
#include <cmath>


#include "SystemTimeInterface.h"


template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::get_drivetrain_command(ControllerMode_e requested_controller_type,
                                                                               TorqueLimit_e requested_torque_limit,
                                                                               const VCRData_s &input_state)
{

    DrivetrainCommand_s empty_command = {.desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f}, .torque_limits = {0.0f, 0.0f, 0.0f, 0.0f}};

    DrivetrainCommand_s current_output = empty_command;

    int req_controller_mode_index = static_cast<int>(requested_controller_type);
    int active_controller_mode_index = static_cast<int>(_active_status.active_controller_mode);

    if ((std::size_t)req_controller_mode_index > ( _controller_evals.size() - 1 ))
    {
        _active_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS;
        return empty_command;
    }

    if( (!_controller_evals[active_controller_mode_index]) || (!_controller_evals[req_controller_mode_index]))
    {
        _active_status.active_error = TorqueControllerMuxError_e::ERROR_CONTROLLER_NULL_POINTER;
        return empty_command;
    }

    current_output = _controller_evals[active_controller_mode_index](input_state, sys_time::hal_millis());

    // std::cout << "output torques " << current_output.inverter_torque_limit[0] << " " << current_output.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;
    bool requesting_controller_change = requested_controller_type != _active_status.active_controller_mode;

    if (requesting_controller_change)
    {
        DrivetrainCommand_s proposed_output = _controller_evals[req_controller_mode_index](input_state, sys_time::hal_millis());
        TorqueControllerMuxError_e error_state = can_switch_controller(input_state.system_data.drivetrain_data, current_output, proposed_output);
        if (error_state == TorqueControllerMuxError_e::NO_ERROR)
        {
            _active_status.active_controller_mode = requested_controller_type;
            active_controller_mode_index = req_controller_mode_index;
            current_output = proposed_output;
        }
        _active_status.active_error = error_state;
    }
    if (!_mux_bypass_limits[active_controller_mode_index])
    {
        _active_status.active_torque_limit_enum = requested_torque_limit;

        if (current_output.desired_speeds.FL == 0.0f && current_output.desired_speeds.FR == 0.0f && current_output.desired_speeds.RL == 0.0f && current_output.desired_speeds.RR == 0.0f)
        {
            current_output = apply_regen_limit(current_output, input_state.system_data.drivetrain_data, input_state.interface_data.inverter_data.FL.dc_bus_voltage);
        }

        current_output = apply_torque_limit(current_output, _torque_limit_map[requested_torque_limit]);
        // std::cout << "output torques after " << current_output.inverter_torque_limit[0] << " " <<current_output.inverter_torque_limit[1] << " " <<current_output.command.inverter_torque_limit[2] << " " <<current_output.command.inverter_torque_limit[3] << std::endl;
        _active_status.active_torque_limit_value = _torque_limit_map[requested_torque_limit];
        // std::cout << "output torques before pw " << current_output.inverter_torque_limit[0] << " " << current_output.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;

        // Do not apply power limit when regen braking
        if (current_output.desired_speeds.FL != 0.0f || current_output.desired_speeds.FR != 0.0f || current_output.desired_speeds.RL != 0.0f || current_output.desired_speeds.RR != 0.0f)
        {
            current_output = apply_power_limit(current_output, input_state.system_data.drivetrain_data, _max_power_limit, _torque_limit_map[requested_torque_limit]);
        }
        // std::cout << "output torques after pw " << current_output.inverter_torque_limit[0] << " " << current_output.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;
        current_output = apply_positive_speed_limit(current_output);
        _active_status.output_is_bypassing_limits = false;
    }
    else{
        _active_status.active_torque_limit_enum = TorqueLimit_e::TCMUX_FULL_TORQUE;
        _active_status.active_torque_limit_value= PhysicalParameters::AMK_MAX_TORQUE;
        _active_status.output_is_bypassing_limits = true;
    }

    // std::cout << "output torques before return " << current_output.inverter_torque_limit[0] << " " << current_output.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;
    return current_output;
}

template <std::size_t num_controllers>
TorqueControllerMuxError_e TorqueControllerMux<num_controllers>::can_switch_controller(DrivetrainDynamicReport_s active_drivetrain_data,
                                                                                      DrivetrainCommand_s previous_controller_command,
                                                                                      DrivetrainCommand_s desired_controller_out)
{
    bool speedPreventsModeChange = false;
    // Check if torque delta permits mode change
    bool torqueDeltaPreventsModeChange = false;

    auto speeds = active_drivetrain_data.measuredSpeeds.as_array();
    auto desired_torq_lims = desired_controller_out.torque_limits.as_array();
    auto prev_torq_lims = previous_controller_command.torque_limits.as_array();
    for (size_t i = 0; i < _num_motors; i++)
    {
        speedPreventsModeChange = (fabs(speeds[i] * RPM_TO_METERS_PER_SECOND) >= _max_change_speed);
        // only if the torque delta is positive do we not want to switch to the new one
        torqueDeltaPreventsModeChange = (desired_torq_lims[i] - prev_torq_lims[i]) > _max_torque_pos_change_delta;
        if (speedPreventsModeChange)
        {
            return TorqueControllerMuxError_e::ERROR_SPEED_DIFF_TOO_HIGH;
        }
        if (torqueDeltaPreventsModeChange)
        {
            return TorqueControllerMuxError_e::ERROR_TORQUE_DIFF_TOO_HIGH;
        }
    }
    return TorqueControllerMuxError_e::NO_ERROR;
}

/* Apply limit such that wheelspeed never goes negative */
template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::apply_positive_speed_limit(const DrivetrainCommand_s &command)
{
    DrivetrainCommand_s out;
    out = command;

    // I just hope HyTech never has to go back to single motor. that would be very sadge :(
    out.desired_speeds.FL = std::max(0.0f,command.desired_speeds.FL);
    out.desired_speeds.FR = std::max(0.0f,command.desired_speeds.FR);
    out.desired_speeds.RL = std::max(0.0f,command.desired_speeds.RL);
    out.desired_speeds.RR = std::max(0.0f,command.desired_speeds.RR);
    return out;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::apply_torque_limit(const DrivetrainCommand_s &command, float max_torque)
{
    DrivetrainCommand_s out = command;
    float avg_torque = 0;
    // get the average torque accross all 4 wheels
    auto torq_lims = out.torque_limits.as_array();
    for (size_t i = 0; i < torq_lims.size(); i++)
    {
        avg_torque += abs(torq_lims[i]);
    }

    avg_torque /= _num_motors;

    // if this is greather than the torque limit, scale down
    if (avg_torque > max_torque)
    {
        // get the scale of avg torque above max torque
        float scale = avg_torque / max_torque;
        // divide by scale to lower avg below max torque
        out.torque_limits.FL = out.torque_limits.FL / scale; 
        out.torque_limits.FR = out.torque_limits.FR / scale; 
        out.torque_limits.RL = out.torque_limits.RL / scale; 
        out.torque_limits.RR = out.torque_limits.RR / scale; 
    }

    return out;
}

/*
    Apply power limit such that the mechanical power of all wheels never
    exceeds the preset mechanical power limit. Scales all wheels down to
    preserve functionality of torque controllers
*/
template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::apply_power_limit(const DrivetrainCommand_s &command, const DrivetrainDynamicReport_s &drivetrain, float power_limit, float max_torque)
{
    DrivetrainCommand_s out = command;
    float net_torque_mag = 0;
    float net_power = 0;


    net_torque_mag += out.torque_limits.FL;
    net_torque_mag += out.torque_limits.FR;
    net_torque_mag += out.torque_limits.RL;
    net_torque_mag += out.torque_limits.RR;

    net_power += (out.torque_limits.FL * (drivetrain.measuredSpeeds.FL * RPM_TO_RAD_PER_SECOND));
    net_power += (out.torque_limits.FR * (drivetrain.measuredSpeeds.FR * RPM_TO_RAD_PER_SECOND));
    net_power += (out.torque_limits.RL * (drivetrain.measuredSpeeds.RL * RPM_TO_RAD_PER_SECOND));
    net_power += (out.torque_limits.RR * (drivetrain.measuredSpeeds.RR * RPM_TO_RAD_PER_SECOND));
    // only evaluate power limit if current power exceeds it
    auto scale_torque_limit = [](float desired_wheel_torque, float current_wheel_rpm, float net_torque_mag, float power_limit, float max_torque) -> float
    {
        float res = desired_wheel_torque;
        
        float desired_wheel_torque_percentage = fabs(desired_wheel_torque / net_torque_mag);
        float corner_power = (desired_wheel_torque_percentage * power_limit);
        
        // power / omega (motor rad/s) to get torque per wheel
        res = fabs(corner_power / (current_wheel_rpm * RPM_TO_RAD_PER_SECOND));
        res = std::max(0.0f, std::min(res, max_torque)); // ensure torque limit is above zero and below max torque(?)
        // std::cout <<"final torque setpoint " << res <<std::endl;
        return res;
    };

    if (net_power > power_limit)
    {

        // std::cout <<"net power too big" <<std::endl;
        out.torque_limits.FL  = scale_torque_limit(out.torque_limits.FL , drivetrain.measuredSpeeds.FL , net_torque_mag, power_limit, max_torque);
        out.torque_limits.FR  = scale_torque_limit(out.torque_limits.FR , drivetrain.measuredSpeeds.FR , net_torque_mag, power_limit, max_torque);
        out.torque_limits.RL  = scale_torque_limit(out.torque_limits.RL , drivetrain.measuredSpeeds.RL , net_torque_mag, power_limit, max_torque);
        out.torque_limits.RR  = scale_torque_limit(out.torque_limits.RR , drivetrain.measuredSpeeds.RR , net_torque_mag, power_limit, max_torque);
    }
    return out;
}



template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::apply_regen_limit(const DrivetrainCommand_s &command, const DrivetrainDynamicReport_s &drivetrain_data, float pack_voltage)
{
    DrivetrainCommand_s out = command;
    const float noRegenLimitKPH = 10.0;
    const float fullRegenLimitKPH = 5.0;
    float maxWheelSpeed = 0.0;
    float torqueScaleDown = 0.0;
    bool allWheelsRegen = true; // true when all wheels are targeting speeds below the current wheel speed

    DrivetrainDynamicReport_s dt_data = drivetrain_data;
    auto speeds = dt_data.measuredSpeeds.as_array();
    auto command_speeds = out.desired_speeds.as_array();
    for (size_t i = 0; i < _num_motors; i++)
    {
        maxWheelSpeed = std::max(maxWheelSpeed, static_cast<float>(fabs(speeds[i]) * RPM_TO_KILOMETERS_PER_HOUR));
        allWheelsRegen &= (command_speeds[i] < static_cast<float>(fabs(speeds[i])) || command_speeds[i] == 0);
    }

    // begin limiting regen at noRegenLimitKPH and completely limit regen at fullRegenLimitKPH
    // linearly interpolate the scale factor between noRegenLimitKPH and fullRegenLimitKPH
    torqueScaleDown = std::min(1.0f, std::max(0.0f, (maxWheelSpeed - fullRegenLimitKPH) / (noRegenLimitKPH - fullRegenLimitKPH)));

    if (allWheelsRegen)
    {
        out.torque_limits.FL *= torqueScaleDown; 
        out.torque_limits.FR *= torqueScaleDown; 
        out.torque_limits.RL *= torqueScaleDown; 
        out.torque_limits.RR *= torqueScaleDown; 
    }

    if(pack_voltage > _regen_limited_pack_voltage) {
        out.torque_limits.FL = std::min(out.torque_limits.FL, _max_high_pack_voltage_regen_nm);
        out.torque_limits.FR = std::min(out.torque_limits.FR, _max_high_pack_voltage_regen_nm);
        out.torque_limits.RL = std::min(out.torque_limits.RL, _max_high_pack_voltage_regen_nm);
        out.torque_limits.RR = std::min(out.torque_limits.RR, _max_high_pack_voltage_regen_nm);
    }
    return out;
}