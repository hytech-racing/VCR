/* From shared-systems-lib */
#include "Logger.h"

/* Local includes */
#include "VehicleStateMachine.h"

void VehicleStateMachine::tick_state_machine(unsigned long current_millis, const VCRData_s &shared_data)
{

    switch (_current_state)
    {

    case CarState_e::STARTUP:
    {
        hal_println("In startup state");
        set_state_(CarState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
        break;
    }

    case CarState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
    {
        // if TS is above HV threshold, move to Tractive System Active
        // _drivetrain.disable_no_pins();
        // if (_drivetrain.hv_over_threshold_on_drivetrain())
        // {
        //     set_state_(CarState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
        // }
        break;
    }

    case CarState_e::TRACTIVE_SYSTEM_ACTIVE:
    {
        if (_buzzer.buzzer_is_active(current_millis))
        {
            _buzzer.deactivate();
        }

        // _drivetrain.disable_no_pins();

        // if (!_drivetrain.hv_over_threshold_on_drivetrain())
        // {
        //     set_state_(CarState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
        //     break;
        // }

        if (shared_data.interface_data.dash_input_state.start_btn_is_pressed && shared_data.interface_data.recvd_pedals_data.pedals_data.brake_is_pressed)
        {
            set_state_(CarState_e::ENABLING_INVERTERS, current_millis);
            break;
        }
        break;
    }

    case CarState_e::ENABLING_INVERTERS:
    {

        // TODO: replace old drivetrain function handling with new interaction paradigm
        // If HV is not active, go to TRACTIVE_SYSTEM_NOT_ACTIVE
        // if (_drivetrain.hv_over_threshold_on_drivetrain())
        // {
        //     set_state_(CarState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
        //     break;
        // }

        // If motor controllers have error, but HV still active
        // if (_drivetrain.drivetrain_error_occured())
        // {
        //     set_state_(CarState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
        // }

        // if (_drivetrain.handle_inverter_startup(current_millis))
        // {
        //     set_state_(CarState_e::WAITING_READY_TO_DRIVE_SOUND, current_millis);
        //     break;
        // }
        break;
    }

    case CarState_e::WAITING_READY_TO_DRIVE_SOUND:
    {

        // If HV is no longer active, return to TRACTIVE_SYSTEM_NOT_ACTIVE
        // if (_drivetrain.hv_over_threshold_on_drivetrain())
        // {
        //     set_state_(CarState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
        //     break;
        // }

        // _drivetrain.command_drivetrain_no_torque(); // While waiting for RTD sound to complete, always command 0 torque

        // If the ready-to-drive sound is done playing, move to ready to drive mode
        if (!_buzzer.buzzer_is_active(current_millis))
        {
            set_state_(CarState_e::READY_TO_DRIVE, current_millis);
        }
        break;

    }

    case CarState_e::READY_TO_DRIVE:
    {
        
        // Commented this state out bc it'll be fixed in the vehicle statemachine PR (and it's referencing drivetrain system methods that don't exist)

        break;
    }

    }
}

void VehicleStateMachine::set_state_(CarState_e new_state, unsigned long curr_millis)
{
    handle_exit_logic_(_current_state, curr_millis);
    _current_state = new_state;
    handle_entry_logic_(new_state, curr_millis);
}

void VehicleStateMachine::handle_exit_logic_(CarState_e prev_state, unsigned long curr_millis)
{
    switch (get_state())
    {
    case CarState_e::STARTUP:
        break;
    case CarState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        break;
    case CarState_e::TRACTIVE_SYSTEM_ACTIVE:
        break;
    case CarState_e::ENABLING_INVERTERS:
        break;
    case CarState_e::WAITING_READY_TO_DRIVE_SOUND:
        break;
    case CarState_e::READY_TO_DRIVE:
        break;
    default:
        break;
    }
}

void VehicleStateMachine::handle_entry_logic_(CarState_e new_state, unsigned long curr_millis)
{
    switch (new_state)
    {
    case CarState_e::STARTUP:
        break;
    case CarState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        break;
    case CarState_e::TRACTIVE_SYSTEM_ACTIVE:
        break;
    case CarState_e::ENABLING_INVERTERS:
        break;
    case CarState_e::WAITING_READY_TO_DRIVE_SOUND:
    {
        _buzzer.activate(curr_millis);
        break;
    }
    case CarState_e::READY_TO_DRIVE:
        break;
    default:
        break;
    }
}
