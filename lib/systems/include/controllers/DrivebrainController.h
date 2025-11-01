#ifndef DRIVEBRAINCONTROLLER_H
#define DRIVEBRAINCONTROLLER_H

#include "controllers/SimpleController.h"
#include "SharedFirmwareTypes.h"
#include <cmath>
#include <hytech.h>

#include <VCRCANInterfaceImpl.h>
#include "CANInterface.h"

class DrivebrainController {
    public:
        /// @brief constructor for the drivebrain controller class
        /// @param allowed_latency the allowed latency in milliseconds for which if the most recent packet has a timestamp older than this measure of time we fail safe
        /// @param assigned_controller_mode the controller mode that the drivebrain controller is assigned to. is required for evaluating whether or not we are active or not
        explicit DrivebrainController(unsigned long allowed_latency,
                            ControllerMode_e assigned_controller_mode = ControllerMode_e::MODE_4)
            : _last_worst_latency_timestamp(0), _emergency_control()
        {
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
        bool get_timing_failure_status() { return !_should_run_controller; }

        void handle_enqueue_timing_status(); 

        void handle_enqueue_latencies(); 

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
        bool _aux_timing_failure = false;
        bool _telem_timing_failure = false;
        bool _should_run_controller = true;

        bool _last_reset_worse_latency_clock = 0; 
        unsigned long _worst_latency_aux = 0;
        unsigned long _worst_latency_telem = 0;

        TorqueControllerSimple _emergency_control = {{1.0f, 1.0f, 20000.0f, 10.0f, -15.0f}}; // NOLINT

        bool _check_drivebrain_command_timing_failure(StampedDrivetrainCommand_s command, unsigned long curr_millis);
};

#endif // DRIVEBRAINCONTROLLER_H