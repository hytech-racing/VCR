#ifndef SIMPLECONTROLLER
#define SIMPLECONTROLLER

#include "SharedFirmwareTypes.h"


/// @param rear_torque_scale 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = Balanced, 2 = RWD
/// @param regen_torque_scale same as rear_torque_scale but applies to regen torque split. 0 = All regen torque on the front, 1 = 50/50, 2 = all regen torque on the rear
struct TorqueControllerSimpleParams_s
{
    float rear_torque_scale = 1.0;
    float regen_torque_scale = 1.0;
};

class TorqueControllerSimple
{
public:
    /// @brief simple TC with tunable F/R torque balance. Accel torque balance can be tuned independently of regen torque balance
    TorqueControllerSimple(TorqueControllerSimpleParams_s params)
        : _params(params)
    { }
    /// @brief calculates torque output based off max torque and simple torque scaling
    TorqueControllerOutput_s evaluate(const VCRSystemData_s &state);

private:
    float _front_torque_scale = 1.0;
    float _rear_torque_scale = 1.0;
    float _front_regen_torque_scale = 1.0;
    float _rear_regen_torque_scale = 1.0;

    TorqueControllerSimpleParams_s _params;

    // static constexpr float _scale = 2.0;
};

#endif