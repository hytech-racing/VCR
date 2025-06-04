#ifndef LOADCELLVECTORINGCONTROLLER
#define LOADCELLVECTORINGCONTROLLER

#include "PhysicalParameters.h"
#include "SharedFirmwareTypes.h"

class LoadCellVectoringTorqueController
{
public:
    /// @brief load cell TC with tunable F/R torque balance. Accel torque balance can be tuned independently of regen torque balance
    /// @param writeout the reference to the torque controller output being sent that contains the drivetrain command
    /// @param rear_torque_scale 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = Balanced, 2 = RWD
    /// @param regen_torque_scale same as rear_torque_scale but applies to regen torque split. 0 = All regen torque on the front, 1 = 50/50, 2 = all regen torque on the rear
    LoadCellVectoringTorqueController(float rear_torque_scale, float regen_torque_scale)
        : _front_torque_scale(2.0 - rear_torque_scale),
          _rear_torque_scale(rear_torque_scale),
          _front_regen_torque_scale(2.0 - regen_torque_scale),
          _rear_regen_torque_scale(regen_torque_scale)
    { }
    /// @brief default contructor with balanced default values: rear_torque_scale = 1.0, regen_torque_scale = 1.0
    LoadCellVectoringTorqueController() : LoadCellVectoringTorqueController(1.0, 1.0) {}

    DrivetrainCommand_s evaluate(const VCRData_s &vcr_data, unsigned long curr_millis);

private:
    const float _front_torque_scale = 1.0;
    const float _rear_torque_scale = 1.0;
    const float _front_regen_torque_scale = 1.0;
    const float _rear_regen_torque_scale = 1.0;

    const size_t _max_error_count = 25;
    veh_vec<size_t> _load_cell_error_counts = {};
};

#endif