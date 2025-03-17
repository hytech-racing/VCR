#ifndef CONTROLS_IMPL
#define CONTROLS_IMPL

#include "DrivetrainSystem.h"
#include "TorqueControllerMux.hpp"
#include "controllers/SimpleController.h"
class VCRControls
{

    public:
        explicit VCRControls(DrivetrainSystem *dt_system);
        void handle_drivetrain_command();
    private:
        TorqueControllerSimple _mode0; // this needs to be first for tc_mux to have a valid capture
        TCMuxTypeMinViable _tc_mux;
        DrivetrainSystem *_dt_system;
        

};

#endif