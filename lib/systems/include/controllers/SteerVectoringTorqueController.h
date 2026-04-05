#ifndef STEERING_VECTORING_TORQUE_CONTROLLER
#define STEERING_VECTORING_TORQUE_CONTROLLER

#include "PhysicalParameters.h"
#include "SharedFirmwareTypes.h"
#include "VCFInterface.h"


class SteerVectoringTorqueController
{
public:
    SteerVectoringTorqueController(float rear_torque_scale, float regen_torque_scale)
        : _front_torque_scale(2.0 - rear_torque_scale),
          _rear_torque_scale(rear_torque_scale),
          _front_regen_torque_scale(2.0 - regen_torque_scale),
          _rear_regen_torque_scale(regen_torque_scale)
    { }
    /// @brief default contructor with balanced default values: rear_torque_scale = 1.0, regen_torque_scale = 1.0
    SteerVectoringTorqueController() : SteerVectoringTorqueController(1.0, 1.0) {}

    DrivetrainCommand_s evaluate(const VCRData_s &vcr_data, unsigned long curr_millis);

private:
    const float _rear_torque_scale = 1.0;
    const float _rear_regen_torque_scale = 1.0;
};

#endif
