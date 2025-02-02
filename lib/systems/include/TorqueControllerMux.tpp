#include "TorqueControllerMux.hpp"
#include "PhysicalParameters.h"
#include "BaseController.h"

// #include <iostream>
template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::get_drivetrain_command(ControllerMode_e requested_controller_type,
                                                                               TorqueLimit_e requested_torque_limit,
                                                                               const VCRSystemData_s &input_state)
{

    DrivetrainCommand_s empty_command = BaseControllerParams::TC_COMMAND_NO_TORQUE;

    TorqueControllerOutput_s current_output = {
        .command = empty_command,
        .ready = true};

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

    current_output = _controller_evals[active_controller_mode_index](input_state);

    // std::cout << "output torques " << current_output.command.inverter_torque_limit[0] << " " << current_output.command.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;
    bool requesting_controller_change = requested_controller_type != _active_status.active_controller_mode;

    if (requesting_controller_change)
    {
        TorqueControllerOutput_s proposed_output = _controller_evals[req_controller_mode_index](input_state);
        TorqueControllerMuxError_e error_state = can_switch_controller(input_state.drivetrain_data, current_output.command, proposed_output.command);
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
        current_output.command = apply_regen_limit(current_output.command, input_state.drivetrain_data);
        // std::cout << "output torques before " << current_output.command.inverter_torque_limit[0] << " " << current_output.command.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;

        current_output.command = apply_torque_limit(current_output.command, _torque_limit_map[requested_torque_limit]);
        // std::cout << "output torques after " << current_output.command.inverter_torque_limit[0] << " " <<current_output.command.inverter_torque_limit[1] << " " <<current_output.command.inverter_torque_limit[2] << " " <<current_output.command.inverter_torque_limit[3] << std::endl;
        _active_status.active_torque_limit_value = _torque_limit_map[requested_torque_limit];
        // std::cout << "output torques before pw " << current_output.command.inverter_torque_limit[0] << " " << current_output.command.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;

        current_output.command = apply_power_limit(current_output.command, input_state.drivetrain_data, _max_power_limit, _torque_limit_map[requested_torque_limit]);
        // std::cout << "output torques after pw " << current_output.command.inverter_torque_limit[0] << " " << current_output.command.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;
        current_output.command = apply_positive_speed_limit(current_output.command);
        _active_status.output_is_bypassing_limits = false;
    }
    else{
        _active_status.active_torque_limit_enum = TorqueLimit_e::TCMUX_FULL_TORQUE;
        _active_status.active_torque_limit_value= PhysicalParameters::AMK_MAX_TORQUE;
        _active_status.output_is_bypassing_limits = true;
    }

    // std::cout << "output torques before return " << current_output.command.inverter_torque_limit[0] << " " << current_output.command.inverter_torque_limit[1] << " " << current_output.command.inverter_torque_limit[2] << " " << current_output.command.inverter_torque_limit[3] << std::endl;
    return current_output.command;
}

template <std::size_t num_controllers>
TorqueControllerMuxError_e TorqueControllerMux<num_controllers>::can_switch_controller(DrivetrainDynamicReport_s active_drivetrain_data,
                                                                                      DrivetrainCommand_s previous_controller_command,
                                                                                      DrivetrainCommand_s desired_controller_out)
{
    bool speedPreventsModeChange = false;
    // Check if torque delta permits mode change
    bool torqueDeltaPreventsModeChange = false;
    for (int i = 0; i < _num_motors; i++)
    {
        speedPreventsModeChange = (abs(active_drivetrain_data.measuredSpeeds[i] * RPM_TO_METERS_PER_SECOND) >= _max_change_speed);
        // only if the torque delta is positive do we not want to switch to the new one
        torqueDeltaPreventsModeChange = (desired_controller_out.inverter_torque_limit[i] - previous_controller_command.inverter_torque_limit[i]) > _max_torque_pos_change_delta;
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
    for (int i = 0; i < _num_motors; i++)
    {
        out.speeds_rpm[i] = std::max(0.0f, out.speeds_rpm[i]);
    }
    return out;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::apply_torque_limit(const DrivetrainCommand_s &command, float max_torque)
{
    DrivetrainCommand_s out = command;
    float avg_torque = 0;
    // get the average torque accross all 4 wheels
    for (int i = 0; i < _num_motors; i++)
    {

        avg_torque += abs(out.inverter_torque_limit[i]);
    }
    avg_torque /= _num_motors;

    // if this is greather than the torque limit, scale down
    if (avg_torque > max_torque)
    {
        // get the scale of avg torque above max torque
        float scale = avg_torque / max_torque;
        // divide by scale to lower avg below max torque
        for (int i = 0; i < _num_motors; i++)
        {
            out.inverter_torque_limit[i] = out.inverter_torque_limit[i] / scale;
        }
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

    // calculate current mechanical power
    for (int i = 0; i < _num_motors; i++)
    {
        // get the total magnitude of torque from all 4 wheels
        // sum up net torque
        net_torque_mag += abs(out.inverter_torque_limit[i]);
        // calculate P = T*w for each wheel and sum together
        net_power += abs(out.inverter_torque_limit[i] * (drivetrain.measuredSpeeds[i] * RPM_TO_RAD_PER_SECOND));
    }

    // only evaluate power limit if current power exceeds it
    if (net_power > power_limit)
    {
        // std::cout <<"net power too big" <<std::endl;
        for (int i = 0; i < _num_motors; i++)
        {

            // calculate the percent of total torque requested per wheel
            float torque_percent = abs(out.inverter_torque_limit[i] / net_torque_mag);
            // based on the torque percent and max power limit, get the max power each wheel can use
            float power_per_corner = (torque_percent * power_limit);

            // std::cout <<"corner power " << power_per_corner <<std::endl;
            // power / omega (motor rad/s) to get torque per wheel
            out.inverter_torque_limit[i] = abs(power_per_corner / (drivetrain.measuredSpeeds[i] * RPM_TO_RAD_PER_SECOND));
            // std::cout <<"adjusted torque setpoint " << out.inverter_torque_limit[i] <<std::endl;
            out.inverter_torque_limit[i] = std::max(0.0f, std::min(out.inverter_torque_limit[i], max_torque));

            // std::cout <<"final torque setpoint " << out.inverter_torque_limit[i] <<std::endl;
        }
    }
    return out;
}

template <std::size_t num_controllers>
DrivetrainCommand_s TorqueControllerMux<num_controllers>::apply_regen_limit(const DrivetrainCommand_s &command, const DrivetrainDynamicReport_s &drivetrain_data)
{
    DrivetrainCommand_s out = command;
    const float noRegenLimitKPH = 10.0;
    const float fullRegenLimitKPH = 5.0;
    float maxWheelSpeed = 0.0;
    float torqueScaleDown = 0.0;
    bool allWheelsRegen = true; // true when all wheels are targeting speeds below the current wheel speed

    for (int i = 0; i < _num_motors; i++)
    {
        maxWheelSpeed = std::max(maxWheelSpeed, abs(drivetrain_data.measuredSpeeds[i]) * RPM_TO_KILOMETERS_PER_HOUR);

        allWheelsRegen &= (out.speeds_rpm[i] < abs(drivetrain_data.measuredSpeeds[i]) || out.speeds_rpm[i] == 0);
    }

    // begin limiting regen at noRegenLimitKPH and completely limit regen at fullRegenLimitKPH
    // linearly interpolate the scale factor between noRegenLimitKPH and fullRegenLimitKPH
    torqueScaleDown = std::min(1.0f, std::max(0.0f, (maxWheelSpeed - fullRegenLimitKPH) / (noRegenLimitKPH - fullRegenLimitKPH)));

    if (allWheelsRegen)
    {
        for (int i = 0; i < _num_motors; i++)
        {
            out.inverter_torque_limit[i] *= torqueScaleDown;
        }
    }
    return out;
}