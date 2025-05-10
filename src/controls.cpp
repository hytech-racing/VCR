#include "controls.h"
#include "SharedFirmwareTypes.h"
#include "VCR_Globals.h"
#include <Arduino.h>

VCRControls::VCRControls(DrivetrainSystem *dt_system, uint32_t max_allowed_db_latency_ms) :
    _mode4(max_allowed_db_latency_ms),
    _tc_mux({
        [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
        [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
        [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode0.evaluate(state, curr_millis); },
        [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode3.evaluate(state, curr_millis); },
        [this](const VCRData_s &state, unsigned long curr_millis) -> DrivetrainCommand_s { return _mode4.evaluate(state, curr_millis); }
    },
    {false, false, false, false, false}),
    _dt_system(dt_system)
{

}

void VCRControls::handle_drivetrain_command(bool wanting_ready_to_drive, bool ready_to_drive)
{
    if(_dt_system != nullptr)
    {
        ControllerMode_e mode = vcr_data.interface_data.dash_input_state.dial_state;

        if(vcr_data.interface_data.dash_input_state.mc_reset_btn_is_pressed)
        {
            DrivetrainResetError_s cmd = {true};
            _dt_system->evaluate_drivetrain(cmd);
        } else if (ready_to_drive) {
            auto dt_command = _tc_mux.get_drivetrain_command(mode, _torque_limit, vcr_data);
            _debug_dt_command = dt_command;
            _dt_system->evaluate_drivetrain(dt_command);
        } else if (wanting_ready_to_drive) {
            DrivetrainInit_s dt_command = {
                .init_drivetrain = INIT_DRIVE_MODE
            };
            _dt_system->evaluate_drivetrain(dt_command);
        } else {
            DrivetrainInit_s dt_command = {
                .init_drivetrain = UNINITIALIZED
            };
            _dt_system->evaluate_drivetrain(dt_command);
        }

        
        // TODO if the user is requesting mc error reset, the dt command needs to be the error reset command

    }
}

        
