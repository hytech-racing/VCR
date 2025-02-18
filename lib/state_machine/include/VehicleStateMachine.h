#ifndef __VEHICLE_STATE_MACHINE__
#define __VEHICLE_STATE_MACHINE__

#include <etl/delegate.h>

enum class VEHICLE_STATE {
    TRACTIVE_SYSTEM_NOT_ACTIVE = 1, 
    TRACTIVE_SYSTEM_ACTIVE = 2, 
    WANTING_READY_TO_DRIVE = 3,
    READY_TO_DRIVE = 4
};

class VehicleStateMachine
{
    public: 
        VehicleStateMachine(
            etl::delegate<bool()> hv_over_threshold, 
            etl::delegate<bool()> start_button_pressed, 
            etl::delegate<bool()> brake_pressed, 
            etl::delegate<bool()> drivetrain_error_ocurred, 
            etl::delegate<bool()> drivetrain_ready,
            etl::delegate<void()> handle_drivetrain_init, 
            etl::delegate<void()> command_drivetrain
        ) :  
        _hv_over_threshold(hv_over_threshold),
        _start_button_pressed(start_button_pressed), 
        _brake_pressed(brake_pressed),
        _drivetrain_error_ocurred(drivetrain_error_ocurred),
        _drivetrain_ready(drivetrain_ready),
        _handle_drivetrain_init(handle_drivetrain_init),
        _command_drivetrain(command_drivetrain)
        {   
            _current_state = VEHICLE_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE;

        }

        void tick_state_machine(unsigned long curr_time_millis);

        VEHICLE_STATE get_state() { return _current_state; }
    
    private: 

        void _set_state(VEHICLE_STATE new_state, unsigned long current_time_millis); 

        void _handle_entry_logic(VEHICLE_STATE prev_state, unsigned long current_time_millis);

        void _handle_exit_logic(VEHICLE_STATE new_state, unsigned long current_time_millis);

        VEHICLE_STATE _current_state;

        // Lambdas neccesary for state machine to work
        etl::delegate<bool()> _hv_over_threshold; 
        etl::delegate<bool()> _start_button_pressed; 
        etl::delegate<bool()> _brake_pressed; 
        etl::delegate<bool()> _drivetrain_error_ocurred; 
        etl::delegate<bool()> _drivetrain_ready;
        etl::delegate<void()> _handle_drivetrain_init;
        etl::delegate<void()> _command_drivetrain; // Shouldn't need to pass anything; logic will be handled in the lambda

};

#endif