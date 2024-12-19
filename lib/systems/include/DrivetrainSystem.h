#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

#include "SharedFirmwareTypes.h"
#include <array>
#include "stdint.h"
#include "SysClock.h"

/**
 * As of now, only the minimum functions for VehicleStateMachine to compile have been implemented.
 * TODO: Re-add the rest of the necessary functions
 * TODO: Add DrivetrainSystem.tpp to implement all functions
 * 
 */
template <typename InverterType>
class DrivetrainSystem
{
public:

    static DrivetrainSystem& getInstance()
    {
        static DrivetrainSystem instance;
        return instance;
    }

    void disable_no_pins();
    bool hv_over_threshold_on_drivetrain();
    bool drivetrain_error_occured();
    bool handle_inverter_startup(unsigned long curr_millis);
    void command_drivetrain_no_torque();

private:
    DrivetrainSystem();
};

// #include "DrivetrainSystem.tpp"
#endif /* DRIVETRAINSYSTEM */
