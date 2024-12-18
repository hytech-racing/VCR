#ifndef VEHICLE_STATE_MACHINE
#define VEHICLE_STATE_MACHINE

#include "Logger.h"

/**
 * Enum representing possible states for the vehicle's state machine.
 * 
 * STARTUP                      - Initial state. Vehicle never stays in STARTUP for more than 1 tick.
 * TRACTIVE_SYSTEM_NOT_ACTIVE   - Car stays here until the Drivetrain reports that the inverters' voltage
 *                                is above the HV threshold.
 * TRACTICE_SYSTEM_ACTIVE       - HV is on, car is waiting for brake + start button (to start car)
 * ENABLING_INVERTERS           - While in this state, the car calls the drivetrain's enable command. If
 *                                successful, (usually nearly-immediate), then car goes into WRTDS.
 * WAITING_READY_TO_DRIVE_SOUND - When entering state, buzzer is activated. After BuzzerController says
 *                                enough time has passed, we enter the next state.
 * READY_TO_DRIVE               - While in this state, pedal inputs command the drivetrain.
 */
enum class CAR_STATE
{
    STARTUP = 0,
    TRACTIVE_SYSTEM_NOT_ACTIVE = 1,
    TRACTIVE_SYSTEM_ACTIVE = 2,
    ENABLING_INVERTERS = 3,
    WAITING_READY_TO_DRIVE_SOUND = 4,
    READY_TO_DRIVE = 5
};

/**
 * This singleton class represents the vehicle's state machine as we enable HV, enable the inverters,
 * wait for the start button to be pressed, and enter ready-to-drive mode. Aside from getters, this
 * class only has one public function, tick_state_machine(). In order to run its update logic, the
 * VehicleStateMachine uses instance data directly from other singleton classes (Buzzer, Pedals, etc)
 * and from the global structs (VCRInterfaceData_s, VCRSystemData_s, etc.).
 */
class VehicleStateMachine
{
public:
    static VehicleStateMachine& getInstance()
    {
        static VehicleStateMachine instance;
        return instance;
    }

    /**
     * This tick() function handles all the update logic for traversing states, and calls the functions
     * of other classes as necessary.
     * @pre Other systems are updated properly
     * @pre All relevant data exists in the data structs (VCRInterfaceData, VCRSystemData, etc.)
     */
    void tick_state_machine(unsigned long current_millis);

    CAR_STATE get_state() { return _current_state; }

    bool car_in_ready_to_drive() { return _current_state == CAR_STATE::READY_TO_DRIVE; };

private:
    VehicleStateMachine()
    {
        _current_state = CAR_STATE::STARTUP;
    }

    void set_state_(CAR_STATE new_state, unsigned long curr_time);

    /**
     * The function run upon the entry of the car into a new state.
     * @param new_state The state in which we are entering.
     */
    void handle_entry_logic_(CAR_STATE new_state, unsigned long curr_time);

    /**
     * The function run upon the exit of a state.
     * @param prev_state the state in which we are leaving.
     */
    void handle_exit_logic_(CAR_STATE prev_state, unsigned long curr_time);

    CAR_STATE _current_state;

};

#endif /* VEHICLE_STATE_MACHINE */
