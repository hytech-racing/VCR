/* From shared-systems-lib */
#include "Logger.h"

/* Local includes */
#include "VehicleStateMachine.h"
#include "VCR_Globals.h"

void VehicleStateMachine::tick_state_machine(unsigned long current_millis)
{

    switch (_current_state)
    {

    case CAR_STATE::STARTUP:
    {
        hal_println("In startup state");
        set_state_(CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
        break;
    }

    case CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
    {
        // if TS is above HV threshold, move to Tractive System Active
        _drivetrain.disable_no_pins();
        if (_drivetrain.hv_over_threshold_on_drivetrain())
        {
            set_state_(CAR_STATE::TRACTIVE_SYSTEM_ACTIVE, current_millis);
        }
        break;
    }

    case CAR_STATE::TRACTIVE_SYSTEM_ACTIVE:
    {
        if (_buzzer.buzzer_is_active(current_millis))
        {
            _buzzer.deactivate();
        }

        _drivetrain.disable_no_pins();

        if (!_drivetrain.hv_over_threshold_on_drivetrain())
        {
            set_state_(CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            break;
        }

        if (system_data.dash_input_state.start_btn_is_pressed && system_data.pedals_system_data.brake_is_pressed)
        {
            set_state_(CAR_STATE::ENABLING_INVERTERS, current_millis);
            break;
        }
        break;
    }

    case CAR_STATE::ENABLING_INVERTERS:
    {

        // If HV is not active, go to TRACTIVE_SYSTEM_NOT_ACTIVE
        if (_drivetrain.hv_over_threshold_on_drivetrain())
        {
            set_state_(CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            break;
        }

        // If motor controllers have error, but HV still active
        if (_drivetrain.drivetrain_error_occured())
        {
            set_state_(CAR_STATE::TRACTIVE_SYSTEM_ACTIVE, current_millis);
        }

        if (_drivetrain.handle_inverter_startup(current_millis))
        {
            set_state_(CAR_STATE::WAITING_READY_TO_DRIVE_SOUND, current_millis);
            break;
        }
        break;
    }

    case CAR_STATE::WAITING_READY_TO_DRIVE_SOUND:
    {

        // If HV is no longer active, return to TRACTIVE_SYSTEM_NOT_ACTIVE
        if (_drivetrain.hv_over_threshold_on_drivetrain())
        {
            set_state_(CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            break;
        }

        _drivetrain.command_drivetrain_no_torque(); // While waiting for RTD sound to complete, always command 0 torque

        // If the ready-to-drive sound is done playing, move to ready to drive mode
        if (!_buzzer.buzzer_is_active(current_millis))
        {
            set_state_(CAR_STATE::READY_TO_DRIVE, current_millis);
        }
        break;

    }

    case CAR_STATE::READY_TO_DRIVE:
    {

        // If HV is no longer active, return to TRACTIVE_SYSTEM_NOT_ACTIVE
        if (!_drivetrain.hv_over_threshold_on_drivetrain())
        {
            hal_println("Drivetrain not over threshold while in READY_TO_DRIVE");
            set_state_(CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            break;
        }

        if (_drivetrain.drivetrain_error_occured())
        {
            hal_println("Drivetrain error occurred while in READY_TO_DRIVE");
            set_state_(CAR_STATE::TRACTIVE_SYSTEM_ACTIVE, current_millis);
            break;
        }

        if (_safetysystem.get_software_is_ok() && !system_data.pedals_system_data.implausibility_has_exceeded_max_duration)
        {
            // TODO: Fix with all references to singleton classes
            // _drivetrain.command_drivetrain(controller_mux_->getDrivetrainCommand(dashboard_->getDialMode(), dashboard_->getTorqueLimitMode(), current_car_state));
        }
        else
        {
            // If software is not OK or some implausibility has exceeded max duration, command 0 torque (but stay in RTD mode)
            _drivetrain.command_drivetrain_no_torque();
        }

        break;
    }

    }
}

void VehicleStateMachine::set_state_(CAR_STATE new_state, unsigned long curr_millis)
{
    handle_exit_logic_(_current_state, curr_millis);
    _current_state = new_state;
    handle_entry_logic_(new_state, curr_millis);
}

void VehicleStateMachine::handle_exit_logic_(CAR_STATE prev_state, unsigned long curr_millis)
{
    switch (get_state())
    {
    case CAR_STATE::STARTUP:
        break;
    case CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
        break;
    case CAR_STATE::TRACTIVE_SYSTEM_ACTIVE:
        break;
    case CAR_STATE::ENABLING_INVERTERS:
        break;
    case CAR_STATE::WAITING_READY_TO_DRIVE_SOUND:
        break;
    case CAR_STATE::READY_TO_DRIVE:
        break;
    default:
        break;
    }
}

void VehicleStateMachine::handle_entry_logic_(CAR_STATE new_state, unsigned long curr_millis)
{
    switch (new_state)
    {
    case CAR_STATE::STARTUP:
        break;
    case CAR_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
        break;
    case CAR_STATE::TRACTIVE_SYSTEM_ACTIVE:
        break;
    case CAR_STATE::ENABLING_INVERTERS:
        break;
    case CAR_STATE::WAITING_READY_TO_DRIVE_SOUND:
    {
        _buzzer.activate(curr_millis);
        break;
    }
    case CAR_STATE::READY_TO_DRIVE:
        break;
    default:
        break;
    }
}
