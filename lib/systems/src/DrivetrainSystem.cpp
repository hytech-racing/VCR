/******************************************************************************
 * @file    DrivetrainSystem.cpp
 * @brief   Source for the drivetrain system statemachine
 ******************************************************************************/

 /******************************************************************************
 * Includes
 ******************************************************************************/
#include <DrivetrainSystem.h>

/******************************************************************************
 * Public Method Definitions
 ******************************************************************************/
DrivetrainSystem::DrivetrainSystem(
    veh_vec<InverterInterface> inverter_interfaces, etl::delegate<void(bool)> set_ef_active_pin, unsigned long ef_pin_enable_delay_ms)
    : _inverter_interfaces(inverter_interfaces), _state(DrivetrainState_e::NOT_CONNECTED),
    _set_ef_active_pin(set_ef_active_pin), 
    _ef_pin_enable_delay_ms(ef_pin_enable_delay_ms) { };


DrivetrainState_e DrivetrainSystem::getState() {
    return _state;
}

DrivetrainStatus_s DrivetrainSystem::getStatus() {
    return _status; 
}

void DrivetrainSystem::userResetDrivetrainError() {
    DrivetrainResetError_s reset_cmd = {true};
    DrivetrainSystem::CmdVariant var = reset_cmd;
    auto state = evaluate_drivetrain(reset_cmd).state;
}

void DrivetrainSystem::userInitDrivetrain() {

}

void 

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
    
    if (attempting_init_while_not_connected) {
        status.cmd_resp = DrivetrainCmdResponse_e::CANNOT_INIT_NOT_CONNECTED;
    } else {
        status.cmd_resp = DrivetrainCmdResponse_e::COMMAND_OK;
    }
    _status = status;
    status.state = state;
    return status; 
}

DrivetrainState_e DrivetrainSystem::_evaluate_state_machine(DrivetrainSystem::CmdVariant cmd) {
    switch(get_state()) {
        case DrivetrainState_e::NOT_CONNECTED: {

            // State Outputs
            _setDrivetrainDisabled(); 
            _setEFActive(false);
            
            // State Transitions
            bool connected_no_hv_present = _checkInvertersConnected() && !_checkHvPresent();
            bool connected_hv_present = _checkInvertersConnected() && _checkHvPresent();
            
            if (connected_no_hv_present) {
                _setState(DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
            } else if (connected_hv_present) {
                _setState(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }

            break;
        }

        case DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT: {

            // State Outputs
            _setDrivetrainDisabled(); 
            _setEFActive(false);

            // State Transitions
            if (_checkHvPresent()) {
                _setState(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }

            break;            
        }

        case DrivetrainState_e::NOT_ENABLED_HV_PRESENT:
        {
            // State Outputs
            _setDrivetrainDisabled(); 
            _setEFActive(false);

            // State Transitions
            if (_checkErrorPresent()) {
                _setState(DrivetrainState_e::ERROR);
            } else if (_checkInvertersReady()) {
                _setState(DrivetrainState_e::INVERTERS_READY);
            }
            break;
        }

        case DrivetrainState_e::INVERTERS_READY: {
            // State Outputs (Note: the reason that this state exists is to attempt to make quit_dc_on true)
            _setEnableDrivetrainHv();
            _setEFActive(false);

            // State Transitions
            bool inverter_error_present = !_checkErrorPresent();
            bool inverters_ready = _checkInvertersReady();
            bool quit_dc_on = _checkQuitDcOn();
            bool hv_present = _checkHvPresent();

            if (inverter_error_present) {
                _setState(DrivetrainState_e::ERROR);
            } else if (hv_present) {
                _setState(DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
            } else if (_init_drivetrain_flag && inverters_ready && quit_dc_on) {
                _setEFActive(true);
                _setState(DrivetrainState_e::INVERTERS_HV_ENABLED);
            } else if (!inverters_ready) {
                _setState(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }
            break;
        }

        case DrivetrainState_e::INVERTERS_HV_ENABLED:
        {

            // State Outputs (Note: trying to make inverters ready and inverters enabled true in this state)
            _setEFActive(true);
            if (sys_time::hal_millis() - _last_toggled_ef_active > _ef_pin_enable_delay_ms) {
                _setEnableDrivetrain();
            } else {
                _setEnableDrivetrainHv(); 
            }


            // State Transitions
            bool inverter_error_present = _checkErrorPresent();
            bool quit_dc_on = _checkQuitDcOn(); 
            bool inverters_ready = _checkInvertersReady();
            bool inverters_enabled = _checkInvertersEnabled(); 

            if (inverter_error_present) {
                _setState(DrivetrainState_e::ERROR);
            } else if (quit_dc_on && inverters_ready && inverters_enabled) {
                _setState(DrivetrainState_e::ENABLED_DRIVE_MODE);
            } else if (!quit_dc_on && inverters_ready) {        
                _setState(DrivetrainState_e::INVERTERS_READY);
            }
            break;
        }

        case DrivetrainState_e::ENABLED_DRIVE_MODE: {

            // State Outputs
            _setDrivetrainCommand(drivetrain_command);
            _set_ef_active_pin(true);

            // State Transitions
            bool inverter_error_present = _checkErrorPresent();  

            if (inverter_error_present) {
                _setState(DrivetrainState_e::ERROR);
            } else if (!_checkHvPresent()) {
                _setState(DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
            }
            break;
        }

        case DrivetrainState_e::ERROR: {
            // State Outputs
            _setDrivetrainDisabled();
            _setEFActive(false);

            DrivetrainCommand_s drivetrain_command = {{0, 0, 0, 0}, {0.0f, 0.0f, 0.0f, 0.0f}};
            _setDrivetrainCommand(drivetrain_command); // Explicitly set RPMs and torques to 0 while in ERROR state
            
            // State Transitions
            bool user_requesting_error_reset = _reset_errors_flag;             
            bool inverter_error_present = _checkErrorPresent();
            
            if (user_requesting_error_reset) {
                _setState(DrivetrainState_e::CLEARING_ERRORS);
            } else if(_reset_errors_flag && !inverter_error_present) {
                _setState(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            } 
            break;
        }

        case DrivetrainState_e::CLEARING_ERRORS: {
            // State Outputs
            _setDrivetrainErrorReset();
            _setEFActive(false);

            DrivetrainCommand_s drivetrain_command = {{0, 0, 0, 0}, {0.0f, 0.0f, 0.0f, 0.0f}};
            _setDrivetrainCommand(drivetrain_command); // Explicitly set RPMs and torques to 0 while in ERROR state
 
            // State Transitions
            bool inverter_error_present = _checkErrorPresent();

            if (!inverter_error_present) {
                _setState(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }
            break;
        }

        default:
            break;
    }

    return get_state();
}

/******************************************************************************
 * Private Method Definitions
 ******************************************************************************/

void DrivetrainSystem::_setState(DrivetrainState_e new_state) {
    _state = new_state;
}

void DrivetrainSystem::_setDrivetrainDisabled() {
    InverterControlWord_s disabled_control_word = {.inverter_enable = false, 
                                                    .hv_enable = false,
                                                    .driver_enable = false,
                                                    .remove_error = false };

    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        inverter.setInverterControlWord(disabled_control_word);
        inverter.setIdle();
    }

}

void DrivetrainSystem::_setEnableDrivetrainHv() {
    InverterControlWord_s enable_hv_control_word = {.inverter_enable = false, 
                                          .hv_enable = true,
                                          .driver_enable = false,
                                          .remove_error = false };
    
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        inverter.setInverterControlWord(enable_hv_control_word);
        inverter.setIdle();
    }
}

void DrivetrainSystem::_setEnableDrivetrain() {
    _setDrivetrainKeepaliveIdle(); // since they are the same
}

void DrivetrainSystem::_setDrivetrainKeepaliveIdle() {
    InverterControlWord_s control_word = {.inverter_enable = true, 
                                          .hv_enable = true,
                                          .driver_enable = true,
                                          .remove_error = false };
    
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        inverter.setInverterControlWord(control_word);
        inverter.setIdle();
    }
}

void DrivetrainSystem::_setDrivetrainErrorReset() {
    InverterControlWord_s control_word = {.inverter_enable = false, 
                                          .hv_enable = true,
                                          .driver_enable = true,
                                          .remove_error = true };
    
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        inverter.setInverterControlWord(control_word);
        inverter.setIdle();
    }
}

void DrivetrainSystem::_setDrivetrainCommand(DrivetrainCommand_s cmd) {
    _inverter_interfaces.FL.setSpeed(cmd.desired_speeds.FL, cmd.torque_limits.FL);
    _inverter_interfaces.FR.setSpeed(cmd.desired_speeds.FR, cmd.torque_limits.FR);
    _inverter_interfaces.RL.setSpeed(cmd.desired_speeds.RL, cmd.torque_limits.RL);
    _inverter_interfaces.RR.setSpeed(cmd.desired_speeds.RR, cmd.torque_limits.RR);
}

void DrivetrainSystem::_setEFActive(bool set_active) {
    digitalWrite(INVERTER_ENABLE_PIN, static_cast<int>(set_active));
    _last_toggled_ef_active = sys_time::hal_millis();
}

bool DrivetrainSystem::_checkHvPresent() {
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        if (!inverter.getStatus().hv_present) {
            return false;
        }
    }
    return true;
}

bool DrivetrainSystem::_checkInvertersReady() {
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        if (!inverter.getStatus().system_ready) {
            return false;
        }
    }
    return true;
}

bool DrivetrainSystem::_checkQuitDcOn() {
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        if (!inverter.getStatus().quit_dc_on) {
            return false;
        }
    }
    return true;
}

bool DrivetrainSystem::_checkErrorPresent() {
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        if (!inverter.getStatus().error) {
            return false;
        }
    }
    return true;
}

bool DrivetrainSystem::_checkDrivetrainActive() {
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        InverterMotorMechanics_s motor_mechanics = inverter.getMotorMechanics();
        if (motor_mechanics.actual_speed_rpm >= MIN_ACTIVE_RPM)
        {
            return true;
        }
    }
    return false;   
}

bool DrivetrainSystem::_checkInvertersEnabled() {
    for (InverterInterface inverter : _inverter_interfaces.as_array()) {
        if (!inverter.getStatus().quit_inverter_on) {
            return false;
        }
    }
    return true;
}