#ifndef CONTROLS_IMPL
#define CONTROLS_IMPL

#include "DrivetrainSystem.h"
#include "SharedFirmwareTypes.h"
#include "TorqueControllerMux.hpp"
#include "controllers/SimpleController.h"
#include "controllers/DrivebrainController.h"

/**
 * Class that is responsible for actually commanding the drivetrain. The VehicleStateMachine
 * must be able to call a no-args, no-return "command drivetrain" function, which is this
 * class's "handle_drivetrain_command()" function. Then, internally, this class will find
 * the DrivetrainCommand from the TC MUX and will command the drivetrain with it.
 */
class VCRControls
{

    public:
        explicit VCRControls(DrivetrainSystem *dt_system, uint32_t max_allowed_db_latency_ms);
        void handle_drivetrain_command();
        DrivetrainCommand_s _debug_dt_command = {};
    private:
        TorqueControllerSimple _mode0; // this needs to be first for tc_mux to have a valid capture
        DrivebrainController _mode4;
        TCMuxType _tc_mux;
        DrivetrainSystem *_dt_system = nullptr;

};

#endif