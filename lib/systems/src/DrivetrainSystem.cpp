#include <DrivetrainSystem.h>
#include <Arduino.h>

//- [x] TODO handle inverter keepalives with correct settings of inverter flags for their associated states

DrivetrainSystem::DrivetrainSystem(
    veh_vec<DrivetrainSystem::InverterFuncts> inverter_interfaces)
    : _inverter_interfaces(inverter_interfaces), _state(DrivetrainState_e::NOT_CONNECTED),
    _check_inverter_ready_flag([](const InverterStatus_s & status) -> bool {return status.system_ready;}),
    _check_inverter_connected_flag([](const InverterStatus_s & status) -> bool {return status.connected;}),
    _check_inverter_quit_dc_flag([](const InverterStatus_s & status) -> bool {return status.quit_dc_on;}),
    _check_inverter_no_errors_present([](const InverterStatus_s & status) -> bool {return !status.error;}),
    _check_inverter_hv_present_flag([](const InverterStatus_s & status) -> bool {return status.hv_present;}),
    _check_inverter_hv_not_present_flag([](const InverterStatus_s & status) -> bool {return !status.hv_present;}), 
    _check_inverter_enabled([](const InverterStatus_s & status) -> bool {return status.quit_inverter_on;}) { };


DrivetrainState_e DrivetrainSystem::get_state()
{
    return _state;
}

DrivetrainStatus_s DrivetrainSystem::get_status()
{
    return _status; 
}

bool DrivetrainSystem::hv_over_threshold()
{
   return _check_inverter_flags(_check_inverter_hv_present_flag); 
}

bool DrivetrainSystem::drivetrain_error_present()
{
    return !_check_inverter_flags(_check_inverter_no_errors_present);
}

bool DrivetrainSystem::drivetrain_ready()
{
    DrivetrainInit_s init_cmd = {.init_drivetrain = DrivetrainModeRequest_e::INIT_DRIVE_MODE};
    DrivetrainSystem::CmdVariant var = init_cmd;
    auto state = evaluate_drivetrain(init_cmd).state;

    return (state == DrivetrainState_e::ENABLED_DRIVE_MODE);
}

DrivetrainStatus_s DrivetrainSystem::evaluate_drivetrain(DrivetrainSystem::CmdVariant cmd) 
{

    auto state = _evaluate_state_machine(cmd);

    DrivetrainStatus_s status;
    status.all_inverters_connected = (_inverter_interfaces.FL.get_status().connected && _inverter_interfaces.FR.get_status().connected && _inverter_interfaces.RL.get_status().connected && _inverter_interfaces.RR.get_status().connected);
    // while not all inverters are connected, some may still be here so we can return the entire status for partially connected drivetrain.
    status.inverter_statuses = {_inverter_interfaces.FL.get_status(), 
                                _inverter_interfaces.FR.get_status(), 
                                _inverter_interfaces.RL.get_status(), 
                                _inverter_interfaces.RR.get_status() };

    bool attempting_init_while_not_connected = ((state == DrivetrainState_e::NOT_CONNECTED) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain != DrivetrainModeRequest_e::UNINITIALIZED));
    
    if (attempting_init_while_not_connected)
    {
        status.cmd_resp = DrivetrainCmdResponse_e::CANNOT_INIT_NOT_CONNECTED;
    } else 
    {
        status.cmd_resp = DrivetrainCmdResponse_e::COMMAND_OK;
    }
    _status = status;
    status.state = state;
    return status; 
}

DrivetrainState_e DrivetrainSystem::_evaluate_state_machine(DrivetrainSystem::CmdVariant cmd)
{
    switch(get_state())
    {
        // TODO need to ensure that the inverter outputs CAN messages on idle even not when being sent msgs
        case DrivetrainState_e::NOT_CONNECTED:
        {
            bool connected_no_hv_present = false;
            connected_no_hv_present = (_check_inverter_flags(_check_inverter_connected_flag) && _check_inverter_flags(_check_inverter_hv_not_present_flag) );
            bool connected_hv_present = false;
            connected_hv_present = (_check_inverter_flags(_check_inverter_connected_flag) && _check_inverter_flags(_check_inverter_hv_present_flag)); 
            
            if(connected_no_hv_present)
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
            } else if(connected_hv_present)
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }
            _set_drivetrain_disabled();
            break;
        }

        case DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT:
        {
            if (_check_inverter_flags(_check_inverter_hv_present_flag))
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }
            _set_drivetrain_disabled();
            break;            
        }

        case DrivetrainState_e::NOT_ENABLED_HV_PRESENT:
        {
            bool inverter_error_present = false;
            inverter_error_present = !_check_inverter_flags(_check_inverter_no_errors_present);
            if(inverter_error_present)
            {
                _set_state(DrivetrainState_e::ERROR);
            }

            else if(_check_inverter_flags(_check_inverter_ready_flag))
            {
                _set_state(DrivetrainState_e::INVERTERS_READY);
            }
            _set_drivetrain_disabled();
            break;
        }

        case DrivetrainState_e::INVERTERS_READY:
        {
            bool inverter_error_present = false;
            inverter_error_present = !_check_inverter_flags(_check_inverter_no_errors_present);
            
            // in this state, we are trying to make quit dc on true
            bool requesting_init = false;
            requesting_init = etl::holds_alternative<DrivetrainInit_s>(cmd) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain != DrivetrainModeRequest_e::UNINITIALIZED);

            bool inverters_ready = false;
            inverters_ready = _check_inverter_flags(_check_inverter_ready_flag);

            bool quit_dc_on = false;
            quit_dc_on = _check_inverter_flags(_check_inverter_quit_dc_flag);

            bool hv_present = false;
            hv_present = _check_inverter_flags(_check_inverter_hv_present_flag);

            if (inverter_error_present) {
                _set_state(DrivetrainState_e::ERROR);
            } else if (!hv_present) {
                _set_state(DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
            } else if(requesting_init && inverters_ready && quit_dc_on) {
                _set_state(DrivetrainState_e::INVERTERS_HV_ENABLED);
            } else if (!inverters_ready) {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            } else {
                // This sends a control word with hv_enable = true
                _set_enable_drivetrain_hv();
            }
            break;
        }

        case DrivetrainState_e::INVERTERS_HV_ENABLED:
        {

            bool inverter_error_present = false;
            inverter_error_present = !_check_inverter_flags(_check_inverter_no_errors_present);

            bool hv_enabled = false;
            hv_enabled = _check_inverter_flags(_check_inverter_quit_dc_flag);

            bool inverters_ready = false;
            inverters_ready = _check_inverter_flags(_check_inverter_ready_flag);

            bool inverters_enabled = false;
            inverters_enabled = _check_inverter_flags(_check_inverter_enabled);

            if(inverter_error_present) {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
                break;
            } else if(hv_enabled && inverters_ready && inverters_enabled) {
                _set_enable_drivetrain();
                _set_state(DrivetrainState_e::INVERTERS_ENABLED);
            } else if(!hv_enabled && inverters_ready) {        
                _set_drivetrain_disabled();  
                _set_state(DrivetrainState_e::INVERTERS_READY);
            } else {
                // This sets inverter enable and driver enable to true in the control word
                _set_enable_drivetrain();
                break;
            } 

            break;
        }

        case DrivetrainState_e::INVERTERS_ENABLED:
        {
            bool inverters_enabled = false;
            inverters_enabled = _check_inverter_flags(_check_inverter_enabled);

            bool inverter_error_present = false; 
            inverter_error_present = !_check_inverter_flags(_check_inverter_no_errors_present);
            
            bool requesting_init = false;
            requesting_init = etl::holds_alternative<DrivetrainInit_s>(cmd) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain != DrivetrainModeRequest_e::UNINITIALIZED);

            if(inverter_error_present)
            {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
            }
            else if (!inverters_enabled)
            {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::INVERTERS_HV_ENABLED);
            }
            else if(requesting_init)
            {
                _set_enable_drivetrain(); // this is just being verbose here, underlying on the inverter's interfaces all maintain 
                _set_state(DrivetrainState_e::ENABLED_DRIVE_MODE);
            } 
            break;
        }

        case DrivetrainState_e::ENABLED_DRIVE_MODE:
        {
            if(!(_check_inverter_flags(_check_inverter_hv_present_flag)))
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
                break;
            }

            bool inverter_error_present = false;
            inverter_error_present = !_check_inverter_flags(_check_inverter_no_errors_present);  

            bool valid_drivetrain_command = etl::holds_alternative<DrivetrainCommand_s>(cmd);
            
            if (inverter_error_present) {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
                break;
            } else if (valid_drivetrain_command) {
                DrivetrainCommand_s drivetrain_command = etl::get<DrivetrainCommand_s>(cmd);
                
                _set_drivetrain_command(drivetrain_command);
            }
            break;
        }

        case DrivetrainState_e::ERROR:
        {
            _set_drivetrain_disabled();
            bool user_requesting_error_reset = etl::holds_alternative<DrivetrainResetError_s>(cmd) && (etl::get<DrivetrainResetError_s>(cmd).reset_errors); 
            bool inverter_error_present = false;
            
            inverter_error_present = !_check_inverter_flags(_check_inverter_no_errors_present);
            
            if(user_requesting_error_reset)
            {
                _set_drivetrain_error_reset();
                _set_state(DrivetrainState_e::CLEARING_ERRORS);
            } else if(user_requesting_error_reset && (!inverter_error_present))
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }
            break;
        }
        case DrivetrainState_e::CLEARING_ERRORS: 
        {
            _set_drivetrain_disabled();
            bool inverter_error_present = false;
            inverter_error_present = !_check_inverter_flags(_check_inverter_no_errors_present);

            if (!inverter_error_present) 
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
                break;
            }
            break;

        }
        default:
            break;
    }

    return get_state();
}

void DrivetrainSystem::_set_state(DrivetrainState_e new_state)
{
    _state = new_state;
}

// returns false if any of the inverters fail the flag check.
bool DrivetrainSystem::_check_inverter_flags(std::function<bool(const InverterStatus_s&)> flag_check_func)
{
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func : funcs_arr)
    {
        auto inverter_status = func.get_status();
        if(!flag_check_func(inverter_status))
        {
            return false;
        }
    }
    return true;
}

void DrivetrainSystem::_set_drivetrain_disabled()
{
    InverterControlWord_s disabled_control_word = {.inverter_enable = false, 
                                                    .hv_enable = false,
                                                    .driver_enable = false,
                                                    .remove_error = false };
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func: funcs_arr)
    {
        func.set_inverter_control_word(disabled_control_word);
        func.set_idle(); // set speed / torque to zero
    }
}

void DrivetrainSystem::_set_enable_drivetrain_hv()
{
    InverterControlWord_s control_word = {.inverter_enable = false, 
                                          .hv_enable = true,
                                          .driver_enable = false,
                                          .remove_error = false };
    
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func : funcs_arr)
    {
        func.set_inverter_control_word(control_word);
        func.set_idle();
    }
}

void DrivetrainSystem::_set_enable_drivetrain()
{
    _set_drivetrain_keepalive_idle(); // since they are the same
}

void DrivetrainSystem::_set_drivetrain_keepalive_idle()
{
    InverterControlWord_s control_word = {.inverter_enable = true, 
                                          .hv_enable = true,
                                          .driver_enable = true,
                                          .remove_error = false };
    
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func : funcs_arr)
    {
        func.set_inverter_control_word(control_word);
        func.set_idle();
    }
}

void DrivetrainSystem::_set_drivetrain_error_reset()
{
    InverterControlWord_s control_word = {.inverter_enable = false, 
                                          .hv_enable = true,
                                          .driver_enable = true,
                                          .remove_error = true };
    
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func : funcs_arr)
    {
        func.set_inverter_control_word(control_word);
        func.set_idle();
    }
}
void DrivetrainSystem::_set_drivetrain_command(DrivetrainCommand_s cmd)
{
    _inverter_interfaces.FL.set_speed(cmd.desired_speeds.FL, cmd.torque_limits.FL);
    _inverter_interfaces.FR.set_speed(cmd.desired_speeds.FR, cmd.torque_limits.FR);
    _inverter_interfaces.RL.set_speed(cmd.desired_speeds.RL, cmd.torque_limits.RL);
    _inverter_interfaces.RR.set_speed(cmd.desired_speeds.RR, cmd.torque_limits.RR);
}

bool DrivetrainSystem::_drivetrain_active(float min_active_rpm)
{
    
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func : funcs_arr)
    {
        auto motor_mechanics = func.get_motor_mechanics();
        if(motor_mechanics.actual_speed >= min_active_rpm)
        {
            return true;
        }
    }
    return false;
}