#ifndef __DRIVEBRAINCONTROLLER_H__
#define __DRIVEBRAINCONTROLLER_H__

#include "controllers/SimpleController.h"
#include "SharedFirmwareTypes.h"
#include <cmath>

// TODO - [ ] need to validate that the times that are apparent in the drivebrain data
//            and ensure that they are within tolerence to current sys-tick

// TODO - [ ] if the drivebrain controller is currently the active controller,
//            the latency fault should be latching

//              meaning that if we fail any of the latency checks while active we cannot clear the timing failure fault
//              unless we switch to another controller and back again. in reality, the CAN line or ethernet conditions
//              probably wont improve randomly during runtime so it will keep faulting. primarily this is just to make sure
//              we dont keep going from latent to not latent while driving and thus the driver gets jittered and the
//              drivetrain will keep "powering up" and "powering down"

// TODO - [ ] correctly measure latency between drivebrain and VCR:
//

/*! \brief Drivebrain Controller for fail-safe pass-through control of the car
           RESPONSIBILITIES AND DOCUMENTATION
  
    this class is the "controller" that allows for pass-through control as commanded by the drivebrain. It also calculates the 
    latency of the most recent input and checks to see if the most recent input is still valid and has not expired. If the 
    input has expired then it switches over to the fail-safe control mode (MODE0) to allow for safe failing even while the car
    is driving so that we dont lose hard-braking capabilities.

    This class can clear it's own fault by switching off of this operating mode and then swapping back to this operating mode. 
    The fault clears the first time this controller gets evaluated while switch from the swapped-to mode back to this pass 
    through mode. 

    latency measurement:
        - the latency is measured by the difference in times in received messages from drivebrain over CAN. if the difference 
          is too great, we swap to MODE0 control. it is assumed that the transmission delay is negligible

    config: 
        - maximum allowed latency 
        - assigned controller mode of this controller (currently defaults to MODE4)
    inputs:
        - the current car state from which it gets the latest drivebrain input, current controller mode and the stamped drivebrain 
          message data
 */
class DrivebrainController
{
public:

    /// @brief constructor for the drivebrain controller class
    /// @param allowed_latency the allowed latency in milliseconds for which if the most recent packet has a timestamp older than this measure of time we fail safe
    /// @param assigned_controller_mode the controller mode that the drivebrain controller is assigned to. is required for evaluating whether or not we are active or not
    explicit DrivebrainController(unsigned long allowed_latency,
                         ControllerMode_e assigned_controller_mode = ControllerMode_e::MODE_4)
        : _emergency_control()
    {
        _last_worst_latency_timestamp = 0;
        _worst_message_latencies = {-1, -1};
        _params = {allowed_latency, assigned_controller_mode};
    }

    /// @brief evaluate function for running the business logic
    /// @param state the current state of the car
    /// @param curr_millis
    /// @return torque controller output that gets passed through the TC MUX
    DrivetrainCommand_s evaluate(const VCRData_s &state, unsigned long curr_millis);

    /// @brief getter for the current status of whether or not the controller has had a timing failure during operation
    /// @return bool of status
    bool get_timing_failure_status() { return _timing_failure; }

private:
    struct
    {
        unsigned long allowed_latency = {};
        ControllerMode_e assigned_controller_mode = {};
    } _params;

    unsigned long _last_worst_latency_timestamp;
    struct {
        int64_t worst_speed_setpoint_latency_so_far;
        int64_t worst_torque_lim_latency_so_far;
    } _worst_message_latencies;
    bool _timing_failure = false;
    TorqueControllerSimple _emergency_control;
};

#endif // __DRIVEBRAINCONTROLLER_H__