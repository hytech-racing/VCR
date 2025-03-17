#include "controls.h"
#include "SharedFirmwareTypes.h"
#include "VCR_Globals.h"

VCRControls::VCRControls(DrivetrainSystem *dt_system) :
    _tc_mux({[this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); }}, {false}),
    _dt_system(dt_system)
    {}
void VCRControls::handle_drivetrain_command()
{
    if(_dt_system != nullptr)
    {
        _dt_system->evaluate_drivetrain(_tc_mux.get_drivetrain_command(ControllerMode_e::MODE_0, TorqueLimit_e::TCMUX_MID_TORQUE, vcr_data));
    }
}
