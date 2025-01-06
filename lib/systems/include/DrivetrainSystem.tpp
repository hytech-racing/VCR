#include "DrivetrainSystem.h"

template <typename InverterType>
void DrivetrainSystem<InverterType>::tick(const SysTick_s &tick)
{

}

template <typename InverterType>
bool DrivetrainSystem<InverterType>::inverter_init_timeout(unsigned long curr_time)
{
    return false;
}

/*----------------------------------------------------------------------------------------*/
// async command functions
/*----------------------------------------------------------------------------------------*/
template <typename InverterType>
bool DrivetrainSystem<InverterType>::handle_inverter_startup(unsigned long curr_time)
{
    return false;
}

template <typename InverterType>
void DrivetrainSystem<InverterType>::enable_drivetrain_hv_(unsigned long curr_time)
{

}

template <typename InverterType>
void DrivetrainSystem<InverterType>::request_enable_()
{

}
/*----------------------------------------------------------------------------------------*/
// rate limited commands. we will only be commanding one of these at a time.
/*----------------------------------------------------------------------------------------*/
template <typename InverterType>
void DrivetrainSystem<InverterType>::command_drivetrain_debug()
{

}

template <typename InverterType>
void DrivetrainSystem<InverterType>::command_drivetrain_no_torque()
{

}



template <typename InverterType>
void DrivetrainSystem<InverterType>::check_reset_condition()
{

}

template <typename InverterType>
void DrivetrainSystem<InverterType>::reset_drivetrain()
{

}

template <typename InverterType>
void DrivetrainSystem<InverterType>::disable_no_pins()
{    

}

template <typename InverterType>
void DrivetrainSystem<InverterType>::disable()
{    

}

/*----------------------------------------------------------------------------------------*/
// feedback functions
/*----------------------------------------------------------------------------------------*/
template <typename InverterType>
bool DrivetrainSystem<InverterType>::drivetrain_error_occured()
{
    return false;
}

template <typename InverterType>
bool DrivetrainSystem<InverterType>::hv_over_threshold_on_drivetrain()
{
    return false;
}
template <typename InverterType>
bool DrivetrainSystem<InverterType>::drivetrain_ready_()
{
    return false;
}

template <typename InverterType>
bool DrivetrainSystem<InverterType>::check_drivetrain_quit_dc_on_()
{
    return false;
}

template <typename InverterType>
bool DrivetrainSystem<InverterType>::drivetrain_enabled_()
{
    return false;
}