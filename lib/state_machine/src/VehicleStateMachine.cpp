/* From shared-systems-lib */
#include "Logger.h"

/* Local includes */
#include "VehicleStateMachine.h"

void VehicleStateMachine::tick_state_machine(unsigned long current_millis)
{

    switch (_current_state)
    {
        case VEHICLE_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
        {
            if (_hv_over_threshold()) 
            {
                _set_state(VEHICLE_STATE::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }
            break;
        }

        case VEHICLE_STATE::TRACTIVE_SYSTEM_ACTIVE: 
        {
            if (!_hv_over_threshold()) 
            {
                _set_state(VEHICLE_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }

            if (_start_button_pressed() && _brake_pressed())
            {
                _set_state(VEHICLE_STATE::WANTING_READY_TO_DRIVE, current_millis);
                break;
            }
            break;
        }

        case VEHICLE_STATE::WANTING_READY_TO_DRIVE: 
        {
            if (!_hv_over_threshold)
            {
                _set_state(VEHICLE_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis); 
                break;
            }

            if (_drivetrain_ready())
            {
                _set_state(VEHICLE_STATE::READY_TO_DRIVE, current_millis);
                break;
            }
            break;
        }

        case VEHICLE_STATE::READY_TO_DRIVE: 
        {
            if (!_hv_over_threshold) 
            {
                _set_state(VEHICLE_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }

            if (_drivetrain_error_ocurred)
            {
                _set_state(VEHICLE_STATE::TRACTIVE_SYSTEM_ACTIVE, current_millis);
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
}

void VehicleStateMachine::_set_state(VEHICLE_STATE new_state, unsigned long curr_millis)
{
    _handle_entry_logic(_current_state, curr_millis);
    _current_state = new_state;
    _handle_exit_logic(new_state, curr_millis);
}

void VehicleStateMachine::_handle_exit_logic(VEHICLE_STATE prev_state, unsigned long curr_millis)
{
    switch (prev_state)
    {
        case VEHICLE_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
            break;
        case VEHICLE_STATE::TRACTIVE_SYSTEM_ACTIVE:
            break;
        case VEHICLE_STATE::WANTING_READY_TO_DRIVE:
            break;
        case VEHICLE_STATE::READY_TO_DRIVE:
            break;
        default:
            break;
        }
}

void VehicleStateMachine::_handle_entry_logic(VEHICLE_STATE new_state, unsigned long curr_millis)
{
    switch (new_state)
        {
        case VEHICLE_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
            break;
        case VEHICLE_STATE::TRACTIVE_SYSTEM_ACTIVE:
            break;
        case VEHICLE_STATE::WANTING_READY_TO_DRIVE:
            break;
        case VEHICLE_STATE::READY_TO_DRIVE:
            break;
        default:
            break;
        }
}
