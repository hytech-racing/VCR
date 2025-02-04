#include <DrivetrainSystem.h>

//- [x] TODO handle inverter keepalives with correct settings of inverter flags for their associated states

DrivetrainSystem::DrivetrainSystem(
    veh_vec<DrivetrainSystem::InverterFuncts> inverter_interfaces)
    : _inverter_interfaces(inverter_interfaces), _state(DrivetrainState_e::NOT_CONNECTED), 
    _check_inverter_ready_flag([](const InverterStatus_s & status) -> bool {return status.inverter_ready;}),
    _check_inverter_connected_flag([](const InverterStatus_s & status) -> bool {return status.connected;}),
    _check_inverter_quit_dc_flag([](const InverterStatus_s & status) -> bool {return status.quit_dc;}),
    _check_inverter_error_flag([](const InverterStatus_s & status) -> bool {return status.error_present;}),
    _check_inverter_hv_present_flag([](const InverterStatus_s & status) -> bool {return status.hv_present;}),
    _check_inverter_hv_not_present_flag([](const InverterStatus_s & status) -> bool {return !status.hv_present;}),
    _check_inverter_enabled([](const InverterStatus_s & status) -> bool {return !status.quit_inverter;}) { };


DrivetrainState_e DrivetrainSystem::get_state()
{
    return _state;
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
            
            _set_drivetrain_disabled(); // TODO dont know if this should be sent here, but it shouldn't hurt
            break;
        }

        case DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT:
        {
            if(_check_inverter_flags(_check_inverter_hv_present_flag))
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }
            _set_drivetrain_disabled();
            break;            
        }

        case DrivetrainState_e::NOT_ENABLED_HV_PRESENT:
        {   
            bool inverter_error_present = false;
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
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
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
            
            // in this state, we are requesting inverters to enable HV
            bool requesting_init = false;
            requesting_init = etl::holds_alternative<DrivetrainInit_s>(cmd) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain != DrivetrainModeRequest_e::UNINITIALIZED);
            bool inverters_ready = false;
            inverters_ready = _check_inverter_flags(_check_inverter_ready_flag);
            bool hv_enabled = false;
            hv_enabled = _check_inverter_flags(_check_inverter_quit_dc_flag);
            
            if(inverter_error_present)
            {
                _set_state(DrivetrainState_e::ERROR);
            } else if(requesting_init && inverters_ready && !hv_enabled)
            {
                _set_enable_drivetrain_hv();
            } else if(requesting_init && inverters_ready && hv_enabled)
            {
                _set_enable_drivetrain_hv();
                _set_state(DrivetrainState_e::INVERTERS_HV_ENABLED);
            } else if(!inverters_ready)
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }
            break;
        }

        case DrivetrainState_e::INVERTERS_HV_ENABLED:
        {
            // in this state, we are requesting the inverter to actually be enabled

            // cases that need to be handled:
            // - [x] if the user stops requesting init -> go back to inverters ready state
            // - [x] error present on an inverter -> go to error state

            bool inverter_error_present = false;
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
            
            bool requesting_init = false;
            requesting_init = etl::holds_alternative<DrivetrainInit_s>(cmd) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain != DrivetrainModeRequest_e::UNINITIALIZED);

            bool hv_enabled = false;
            hv_enabled = _check_inverter_flags(_check_inverter_quit_dc_flag);
            bool inverters_ready = false;
            inverters_ready = _check_inverter_flags(_check_inverter_ready_flag);
            bool inverters_enabled = false;
            inverters_enabled = _check_inverter_flags(_check_inverter_enabled);

            if(inverter_error_present)
            {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
            }
            else if(requesting_init && hv_enabled && inverters_ready && hv_enabled && !inverters_enabled)
            {
                _set_enable_drivetrain(); // should be done on entry of this state
            } else if(hv_enabled && inverters_ready && hv_enabled && inverters_enabled)
            {
                _set_enable_drivetrain();
                _set_state(DrivetrainState_e::INVERTERS_ENABLED);
            } else if((!requesting_init) && inverters_ready)
            {
                // NOTE if the user only calls the evaluate_drivetrain function once with the initialization request, 
                // this statement will kick in and lead to the inverter staying disabled. the user must keep calling with the initialization 
                // request flag / struct type in order to keep this state machine evaluating properly          
                _set_drivetrain_disabled();  
                _set_state(DrivetrainState_e::INVERTERS_READY);
            }

            break;
        }

        case DrivetrainState_e::INVERTERS_ENABLED:
        {
            bool inverter_error_present = false; 
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
            
            bool requesting_speed_mode = false;
            requesting_speed_mode = etl::holds_alternative<DrivetrainInit_s>(cmd) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain == DrivetrainModeRequest_e::INIT_SPEED_MODE);
            bool requesting_torque_mode = false;
            requesting_torque_mode = etl::holds_alternative<DrivetrainInit_s>(cmd) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain == DrivetrainModeRequest_e::INIT_TORQUE_MODE);
            
            // for now i wont worry about checking whether or not the inverters are still enabled.
            // bool inverters_enabled = _check_inverter_flags(_check_inverter_enabled);

            if(inverter_error_present)
            {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
            }
            else if(requesting_speed_mode)
            {
                // on transition the pin will be written to the correct state (for speed mode it should be off)
                
                _set_enable_drivetrain(); // this is just being verbose here, underlying on the inverter's interfaces all maintain 
                _set_state(DrivetrainState_e::ENABLING_INVERTERS_SPEED_MODE);
            } else if(requesting_torque_mode)
            {
                // on transition the pin will be written to the correct state (for torq mode it should be on)
                _set_enable_drivetrain();
                _set_state(DrivetrainState_e::ENABLING_INVERTERS_TORQUE_MODE);
            }
            
            break;
        }

        case DrivetrainState_e::ENABLING_INVERTERS_SPEED_MODE:
        {    
            bool inverter_error_present = false;
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
            bool inverters_in_speed_mode = false;
            inverters_in_speed_mode = (!_get_gpio_state().torque_mode_enabled_pin_state);
            
            if(inverter_error_present)
            {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
            } else if(inverters_in_speed_mode)
            {
                _set_enable_drivetrain();
                _set_state(DrivetrainState_e::ENABLED_SPEED_MODE);
            }
            break;
        }

        case DrivetrainState_e::ENABLING_INVERTERS_TORQUE_MODE:
        {

            bool inverter_error_present = false;
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
            bool inverters_in_torque_mode = false;
            inverters_in_torque_mode = _get_gpio_state().torque_mode_enabled_pin_state;
            
            if(inverter_error_present)
            {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
            } else if(inverters_in_torque_mode)
            {
                _set_enable_drivetrain();
                _set_state(DrivetrainState_e::ENABLED_TORQUE_MODE);
            }

            break;
        }

        case DrivetrainState_e::ENABLED_SPEED_MODE:
        {
            if(!(_check_inverter_flags(_check_inverter_hv_present_flag)))
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
                break;
            }

            // TODO may need to verify that the gpio state is correct while in this state (GPIO torque mode low)
            bool inverter_error_present = false;
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
            // now finally in this mode and the ENABLED_TORQUE_MODE can we command the drivetrain
            bool user_requesting_speed_command = false;
            user_requesting_speed_command = etl::holds_alternative<DrivetrainSpeedCommand_s>(cmd);
            bool user_requesting_torque_mode = false;
            user_requesting_torque_mode = etl::holds_alternative<DrivetrainInit_s>(cmd) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain == DrivetrainModeRequest_e::INIT_TORQUE_MODE);            
            
            if(inverter_error_present)
            {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
                break;
            }
            else if(user_requesting_speed_command)
            {
                _set_drivetrain_speed_command(etl::get<DrivetrainSpeedCommand_s>(cmd));
                
            } else if(user_requesting_torque_mode && !_drivetrain_active(_active_rpm_level))
            {
                _set_drivetrain_keepalive_idle();
                _set_state(DrivetrainState_e::ENABLING_INVERTERS_TORQUE_MODE);
            }
            break;
        }
        case DrivetrainState_e::ENABLED_TORQUE_MODE:
        {
            // TODO may need to verify that the gpio state is correct while in this state (GPIO torque mode high)
            bool inverter_error_present = false;
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
            
            bool user_requesting_torque_command = etl::holds_alternative<DrivetrainTorqueCommand_s>(cmd);
            bool user_requesting_speed_mode = etl::holds_alternative<DrivetrainInit_s>(cmd) && (etl::get<DrivetrainInit_s>(cmd).init_drivetrain == DrivetrainModeRequest_e::INIT_SPEED_MODE);
            
            if(inverter_error_present)
            {
                _set_drivetrain_disabled();
                _set_state(DrivetrainState_e::ERROR);
            }
            else if(user_requesting_torque_command)
            {
                _set_drivetrain_torque_command(etl::get<DrivetrainTorqueCommand_s>(cmd));
            } else if(user_requesting_speed_mode && !_drivetrain_active(_active_rpm_level))
            {
                _set_drivetrain_keepalive_idle();
                _set_state(DrivetrainState_e::ENABLING_INVERTERS_TORQUE_MODE);
            }
            break;
        }
        case DrivetrainState_e::ERROR:
        {
            bool user_requesting_error_reset = etl::holds_alternative<DrivetrainResetError_s>(cmd) && (etl::get<DrivetrainResetError_s>(cmd).reset_errors); 
            bool inverter_error_present = false;
            inverter_error_present = _check_inverter_flags(_check_inverter_error_flag);
            
            if(user_requesting_error_reset && inverter_error_present)
            {
                _set_drivetrain_error_reset();
            } else if(user_requesting_error_reset && (!inverter_error_present))
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
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
    _handle_exit_logic(_state);
    _state = new_state;
    _handle_entry_logic(new_state);
}

void DrivetrainSystem::_handle_exit_logic(DrivetrainState_e prev_state)
{
    switch (prev_state)
    {
    case DrivetrainState_e::NOT_CONNECTED:
        break;
    case DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT:
        break;
    case DrivetrainState_e::NOT_ENABLED_HV_PRESENT:
        break;
    case DrivetrainState_e::INVERTERS_READY:
        break;
    case DrivetrainState_e::INVERTERS_HV_ENABLED:
        break;
    case DrivetrainState_e::INVERTERS_ENABLED:
        break;
    case DrivetrainState_e::ENABLING_INVERTERS_SPEED_MODE:
        break;
    case DrivetrainState_e::ENABLING_INVERTERS_TORQUE_MODE:
        break;
    case DrivetrainState_e::ENABLED_SPEED_MODE:
        break;
    case DrivetrainState_e::ENABLED_TORQUE_MODE:
        break;
    case DrivetrainState_e::ERROR:
        break;
    default:
        break;
    }
}

void DrivetrainSystem::_handle_entry_logic(DrivetrainState_e new_state)
{
    switch (new_state)
    {
    case DrivetrainState_e::NOT_CONNECTED:
        break;
    case DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT:
        break;
    case DrivetrainState_e::NOT_ENABLED_HV_PRESENT:
        break;
    case DrivetrainState_e::INVERTERS_READY:
        break;
    case DrivetrainState_e::INVERTERS_HV_ENABLED:
        break;
    case DrivetrainState_e::INVERTERS_ENABLED:
        break;
    
    case DrivetrainState_e::ENABLING_INVERTERS_SPEED_MODE:
    {
        // set the torque mode pin state to false (aka: keep the inverter in the main control mode which is speed mode)
        _set_gpio_state({false}); 
        break;
    }
        
    case DrivetrainState_e::ENABLING_INVERTERS_TORQUE_MODE:
    {
        _set_gpio_state({true});
        break;
    }

    case DrivetrainState_e::ENABLED_SPEED_MODE:
        break;
    case DrivetrainState_e::ENABLED_TORQUE_MODE:
        break;
    case DrivetrainState_e::ERROR:
        break;
    default:
        break;
    }
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
void DrivetrainSystem::_set_drivetrain_speed_command(DrivetrainSpeedCommand_s cmd)
{
    _inverter_interfaces.FL.set_speed(cmd.desired_speed_rpm.FL, cmd.torque_limit_nm.FL);
    _inverter_interfaces.FR.set_speed(cmd.desired_speed_rpm.FR, cmd.torque_limit_nm.FR);
    _inverter_interfaces.RL.set_speed(cmd.desired_speed_rpm.RL, cmd.torque_limit_nm.RL);
    _inverter_interfaces.RR.set_speed(cmd.desired_speed_rpm.RR, cmd.torque_limit_nm.RR);
}

void DrivetrainSystem::_set_drivetrain_torque_command(DrivetrainTorqueCommand_s cmd)
{
    _inverter_interfaces.FL.set_torque(cmd.desired_torque_nm.FL);
    _inverter_interfaces.FR.set_torque(cmd.desired_torque_nm.FR);
    _inverter_interfaces.RL.set_torque(cmd.desired_torque_nm.RL);
    _inverter_interfaces.RR.set_torque(cmd.desired_torque_nm.RR);
}

bool DrivetrainSystem::_drivetrain_active(float max_active_rpm)
{
    
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func : funcs_arr)
    {
        auto inv_status = func.get_status();
        if(inv_status.speed_rpm >= max_active_rpm)
        {
            return true;
        }
    }
    return false;
}