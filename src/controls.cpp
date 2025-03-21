#include "controls.h"
#include "SharedFirmwareTypes.h"
#include "VCR_Globals.h"
#include <Arduino.h>

VCRControls::VCRControls(DrivetrainSystem *dt_system) :
    _tc_mux({[this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); }}, {true}),
    _dt_system(dt_system)
    {}
void VCRControls::handle_drivetrain_command()
{
    if(_dt_system != nullptr)
    {
        auto dt_command = _tc_mux.get_drivetrain_command(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_MID_TORQUE, vcr_data);
        _dt_system->evaluate_drivetrain(dt_command);
        // Serial.println("layer 2");
        // Serial.println(dt_command.torque_limits.FL);
        _debug_dt_command = dt_command;
    }
}
