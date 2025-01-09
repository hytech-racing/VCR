#include <DrivetrainSystemv2.h>

// DrivetrainSystem::DrivetrainSystem()
// {

// };
DrivetrainSystem::DrivetrainSystem(
    veh_vec<DrivetrainSystem::InverterFuncts> inverter_interfaces)
    : _inverter_interfaces(inverter_interfaces) {};


DrivetrainStatus_s DrivetrainSystem::evaluate_drivetrain(etl::variant<DrivetrainSpeedCommand_s, DrivetrainTorqueCommand_s, DrivetrainInit_s> cmd) {}