/* From shared-systems-lib */
#include "Logger.h"

/* Local includes */
#include "VehicleStateMachine.h"

VehicleState_e VehicleStateMachine::tick_state_machine(unsigned long current_millis)
{
    switch (_current_state)
    {
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        {
            if (_check_hv_over_threshold()) 
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }
            break;
        }

        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE: 
        {
            if (!_check_hv_over_threshold()) 
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }

            if (_is_start_button_pressed() && _is_brake_pressed())
            {
                _set_state(VehicleState_e::WANTING_READY_TO_DRIVE, current_millis);
                break;
            }
            break;
        }
        case VehicleState_e::WANTING_READY_TO_DRIVE: 
        {
            if (!_check_hv_over_threshold())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis); 
                break;
            }

            if (_check_drivetrain_ready())
            {
                _set_state(VehicleState_e::READY_TO_DRIVE, current_millis);
                break;
            }
            break;
        }

        case VehicleState_e::READY_TO_DRIVE: 
        {
            if (!_check_hv_over_threshold()) 
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }

            // TODO this shouldnt de-latch us. 
            if (_check_drivetrain_error_ocurred())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }

            if(_check_pedals_timeout())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }

            _command_drivetrain();
            break;
        }

        default: 
        {
            break;
        }
    }
    return _current_state;
}

void VehicleStateMachine::_set_state(VehicleState_e new_state, unsigned long curr_millis)
{
    _handle_exit_logic(_current_state, curr_millis);
    _current_state = new_state;
    _handle_entry_logic(_current_state, curr_millis);
}

void VehicleStateMachine::_handle_exit_logic(VehicleState_e prev_state, unsigned long curr_millis)
{
    switch (prev_state)
    {
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
            break;
        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE:
            break;
        case VehicleState_e::WANTING_READY_TO_DRIVE:
            break;
        case VehicleState_e::READY_TO_DRIVE:
            break;
        default:
            break;
    }
}

void VehicleStateMachine::_handle_entry_logic(VehicleState_e new_state, unsigned long curr_millis)
{
    switch (new_state)
    {
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
            break;
        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE:
            break;
        case VehicleState_e::WANTING_READY_TO_DRIVE:
        {
            _start_buzzer();
            _reset_pedals_timeout();
            break;
        }
        case VehicleState_e::READY_TO_DRIVE:
            break;
        default:
            break;
    }
}