#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

#include "etl/variant.h"

#include <array>
#include <functional>
#include "stdint.h"

#include "SharedFirmwareTypes.h"
#include "SysClock.h"
#include <shared_types.h>

// requirements:
// - [ ] must support ability to initialize the drivetrain 
// - [ ] ability to command inverters individually and be able to return a failure status when attempting to send invalid command for a certain state
// - [ ] contain a state machine for managing the state of the drivetrain as a whole (aka: all inverters have the same state)
//  - [ ] initialization states included
//  - [ ] different control mode states
// - [ ] single point of interaction / control of the drivetrain that can receive "commands"
//      (at least for now, need to see how this works out once we start using it)
// - [ ] be decoupled from the inverter class
//      std::function / etl::delegate registered functions for the inverter interface. mostly for ease of testing.
// - [ ] be able to reset drivetrain
    // - [ ] 

// TODO move these into the shared types after finishing the system 
enum class DrivetrainState_e
{
    NOT_CONNECTED = 0,
    NOT_ENABLED_NO_HV_PRESENT = 1,
    NOT_ENABLED_HV_PRESENT = 2,
    INVERTERS_READY = 3,
    INVERTERS_HV_ENABLED = 4,
    INVERTERS_ENABLED = 5,
    ENABLED_DRIVE_MODE = 6,
    ERROR = 7, 
    CLEARING_ERRORS = 8
};

enum class DrivetrainCmdResponse_e
{
    COMMAND_OK = 0,
    CANNOT_INIT_NOT_CONNECTED = 1,
    COMMAND_INVALID = 2
};

struct DrivetrainStatus_s
{
    bool all_inverters_connected;
    veh_vec<InverterStatus_s> inverter_statuses;
    DrivetrainCmdResponse_e cmd_resp;
    DrivetrainState_e state;
};

struct DrivetrainResetError_s
{
    bool reset_errors; // true: reset the errors present on inverters, false: dont
};

enum DrivetrainModeRequest_e 
{
    UNINITIALIZED = 0,
    INIT_DRIVE_MODE = 1
};

struct DrivetrainInit_s 
{
    DrivetrainModeRequest_e init_drivetrain;
};

class DrivetrainSystem
{
public:
    using CmdVariant = etl::variant<DrivetrainCommand_s, DrivetrainInit_s, DrivetrainResetError_s>;
    DrivetrainSystem() = delete;
    
    // these are functions for interaction with the state machine mostly
    bool hv_over_threshold();
    bool drivetrain_error_present();
    bool drivetrain_ready();

    DrivetrainStatus_s evaluate_drivetrain(CmdVariant cmd);
    DrivetrainState_e get_state();
    DrivetrainStatus_s get_status();

    // DrivetrainDynamicReport_s get_dynamic_data();

    struct InverterFuncts {
        std::function<void(float desired_rpm, float torque_limit_nm)> set_speed;
        std::function<void()> set_idle;
        std::function<void(InverterControlWord_s control_word)> set_inverter_control_word;
        std::function<InverterStatus_s()> get_status;
        std::function<MotorMechanics_s()> get_motor_mechanics; 
    };
    
    DrivetrainSystem(veh_vec<DrivetrainSystem::InverterFuncts> inverter_interfaces);
private:
    bool _check_inverter_flags(std::function<bool(const InverterStatus_s&)> flag_check_func);
    bool _drivetrain_active(float min_active_rpm);

    void _set_state(DrivetrainState_e state);
    
    void _set_drivetrain_disabled();
    void _set_drivetrain_keepalive_idle();
    void _set_enable_drivetrain_hv();
    void _set_enable_drivetrain();
    void _set_drivetrain_error_reset();
    void _set_drivetrain_command(DrivetrainCommand_s cmd);

    DrivetrainState_e _evaluate_state_machine(CmdVariant cmd);

private:
    const float _active_rpm_level = 100;
    veh_vec<InverterFuncts> _inverter_interfaces;
    DrivetrainState_e _state;
    DrivetrainStatus_s _status;
    std::function<bool(const InverterStatus_s &)> _check_inverter_ready_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_connected_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_quit_dc_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_no_errors_present;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_not_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_enabled;    
};

#endif /* DRIVETRAINSYSTEM */