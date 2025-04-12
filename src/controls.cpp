#include "controls.h"
#include "SharedFirmwareTypes.h"
#include "VCR_Globals.h"
#include <Arduino.h>

VCRControls::VCRControls(DrivetrainSystem *dt_system, uint32_t max_allowed_db_latency_ms) :
    _mode4(max_allowed_db_latency_ms),
    _tc_mux({[this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
    [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
    [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
    [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
    [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode4.evaluate(state, curr_millis); }},
    {true, true, true, true, true}),
    _dt_system(dt_system)
{

}

void VCRControls::handle_drivetrain_command()
{
    if(_dt_system != nullptr)
    {
        // Testing code (mode 0 and mode 4)

        auto dt_command = _tc_mux.get_drivetrain_command(mode, TorqueLimit_e::TCMUX_MID_TORQUE, vcr_data);
        _debug_dt_command = dt_command;
        _dt_system->evaluate_drivetrain(dt_command);
        
        // TODO if the user is requesting mc error reset, the dt command needs to be the error reset command
        
    }
}

        
