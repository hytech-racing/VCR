#ifndef __VehicleState_e_MACHINE__
#define __VehicleState_e_MACHINE__

#include <etl/delegate.h>

enum class VehicleState_e {
    TRACTIVE_SYSTEM_NOT_ACTIVE = 1, 
    TRACTIVE_SYSTEM_ACTIVE = 2,
    WANTING_READY_TO_DRIVE = 3,
    READY_TO_DRIVE = 4
};

class VehicleStateMachine
{
    public: 
        VehicleStateMachine(
            etl::delegate<bool()> check_hv_over_threshold, 
            etl::delegate<bool()> is_start_button_pressed, 
            etl::delegate<bool()> is_brake_pressed, 
            etl::delegate<bool()> check_drivetrain_error_ocurred, 
            etl::delegate<bool()> check_drivetrain_ready, // when calling this function the initialization of the drivetrain is occuring, returning false during and true when finished
            etl::delegate<void()> start_buzzer,
            etl::delegate<void()> command_drivetrain,
            etl::delegate<bool()> check_pedals_timeout,
            etl::delegate<void()> reset_pedals_timeout
        ) :  
        _check_hv_over_threshold(check_hv_over_threshold),
        _is_start_button_pressed(is_start_button_pressed), 
        _is_brake_pressed(is_brake_pressed),
        _check_drivetrain_error_ocurred(check_drivetrain_error_ocurred),
        _check_drivetrain_ready(check_drivetrain_ready),
        _start_buzzer(start_buzzer),
        _command_drivetrain(command_drivetrain),
        _check_pedals_timeout(check_pedals_timeout),
        _reset_pedals_timeout(reset_pedals_timeout)
        {   
            _current_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
        }

        VehicleState_e tick_state_machine(unsigned long curr_time_millis);

        VehicleState_e get_state() { return _current_state; }
    
    private: 

        void _set_state(VehicleState_e new_state, unsigned long current_time_millis); 

        void _handle_entry_logic(VehicleState_e prev_state, unsigned long current_time_millis);

        void _handle_exit_logic(VehicleState_e new_state, unsigned long current_time_millis);

        VehicleState_e _current_state;

        // Lambdas neccesary for state machine to work
        etl::delegate<bool()> _check_hv_over_threshold; 
        etl::delegate<bool()> _is_start_button_pressed; 
        etl::delegate<bool()> _is_brake_pressed; 
        etl::delegate<bool()> _check_drivetrain_error_ocurred; 
        etl::delegate<bool()> _check_drivetrain_ready; 
        etl::delegate<void()> _start_buzzer;
        etl::delegate<void()> _command_drivetrain; // Shouldn't need to pass anything; logic will be handled in the lambda
        etl::delegate<bool()> _check_pedals_timeout;
        etl::delegate<void()> _reset_pedals_timeout;
};

#endif