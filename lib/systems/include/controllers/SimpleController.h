#ifndef SIMPLECONTROLLER
#define SIMPLECONTROLLER

#include "SharedFirmwareTypes.h"



/// @param rear_torque_scale 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = Balanced, 2 = RWD
/// @param rear_regen_torque_scale same as rear_torque_scale but applies to regen torque split. 0 = All regen torque on the front, 1 = 50/50, 2 = all regen torque on the rear

struct TorqueControllerSimpleParams_s
{
private:
    static constexpr speed_rpm _amk_max_rpm_default = 20000.0f;
    static constexpr torque_nm _amk_max_torque = 21.0f;
    static constexpr torque_nm _amk_max_regen_torque = 10.0f;
public:
    float rear_torque_scale = {};
    float rear_regen_torque_scale = {};
    speed_rpm amk_max_rpm = {};
    torque_nm amk_max_torque = {};
    torque_nm amk_max_regen_torque = {};
    TorqueControllerSimpleParams_s()
    : rear_torque_scale(1.0f),
    rear_regen_torque_scale(1.0f),
    amk_max_rpm(_amk_max_rpm_default),
    amk_max_torque(_amk_max_torque),
    amk_max_regen_torque(_amk_max_regen_torque) {}
};

class TorqueControllerSimple
{
public:
    /// @brief simple TC with tunable F/R torque balance. Accel torque balance can be tuned independently of regen torque balance
    TorqueControllerSimple(TorqueControllerSimpleParams_s params)
        : _params(params)
    { }
    /// @brief calculates torque output based off max torque and simple torque scaling
    DrivetrainCommand_s evaluate(const VCRData_s &state);

private:
    TorqueControllerSimpleParams_s _params;

};

#endif