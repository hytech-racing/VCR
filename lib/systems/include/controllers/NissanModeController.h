#ifndef NISSANMODECONTROLLER_H
#define NISSANMODECONTROLLER_H

#include "PhysicalParameters.h"
#include "SharedFirmwareTypes.h"

class NissanModeController
{
  public:
    /// @brief nissan mode torque controller calculating torque based on commanded torque and
    /// current motor speed
    /// @param rear_torque_scale 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = 50/50,
    /// Balanced, 2 = RWD
    /// @param regen_torque_scale same as rear_torque_scale but applies to regen torque split. 0 =
    /// All regen torque on the front, 1 = 50/50, 2 = all regen torque on the rear
    NissanModeController(float rear_torque_scale, float regen_torque_scale, float slip_threshold)
        : _front_torque_scale(2.0 - rear_torque_scale), _rear_torque_scale(rear_torque_scale),
          _front_regen_torque_scale(2.0 - regen_torque_scale),
          _rear_regen_torque_scale(regen_torque_scale), _slip_threshold(slip_threshold) {}
    /// @brief default contructor with balanced default values: rear_torque_scale = 1.0, regen_torque_scale = 1.0
    NissanModeController() : NissanModeController(1.8, 0.6, 1.2) {}

    DrivetrainCommand_s evaluate(const VCRData_s &vcr_data, unsigned long curr_millis);

  private:
    const float _front_torque_scale = 1.0;
    const float _rear_torque_scale = 1.0;
    const float _front_regen_torque_scale = 1.0;
    const float _rear_regen_torque_scale = 1.0;
    const float _max_amk_rpm = 20000.0f; // TODO do we want to make this a vcr const?
    const float _max_amk_regen = 15.0f;  // TODO do we want to make this a vcr const?
    const float _slip_threshold = 1.2; // slip threshold multiplier for torque reduction
    const float _damping = 0.2f; // damping factor for torque compensation
    const float _rpm_jump_limit = 0.05f; // rpm jump limit factor for torque compensation

    float _last_fl_torque = 0.0f;
    float _last_fr_torque = 0.0f;
    float _last_rl_torque = 0.0f;
    float _last_rr_torque = 0.0f;
};

#endif