#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

#include "SharedFirmwareTypes.h"
#include <array>
#include "stdint.h"
#include "SysClock.h"


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
