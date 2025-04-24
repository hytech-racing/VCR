#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

#include "etl/variant.h"
#include "etl/delegate.h"

#include <array>
#include <functional>
#include "stdint.h"

#include "SharedFirmwareTypes.h"
#include "SysClock.h"
#include <shared_types.h>
#include "SystemTimeInterface.h"

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
    ENABLED_DRIVE_MODE = 5,
    ERROR = 6, 
    CLEARING_ERRORS = 7
};

/**
 * When user calls evaluate_drivetrain(), this is part of the returned status to
 * indicate if the command was successful, invalid, or 
 */
enum class DrivetrainCmdResponse_e
{
    COMMAND_OK = 0,
    CANNOT_INIT_NOT_CONNECTED = 1, // When requesting init but inverters are not yet requested
    COMMAND_INVALID = 2
};

/**
 * Actual struct that gets returned on drivetrain evaluation. Contains the current DSM state,
 * the command response (OK, INVALID, etc), and each corner's inverter status.
 */
struct DrivetrainStatus_s
{
    bool all_inverters_connected;
    veh_vec<InverterStatus_s> inverter_statuses;
    DrivetrainCmdResponse_e cmd_resp;
    DrivetrainState_e state;
};




/**
 * There are three types of commands going into the DrivetrainSystem. There is the
 * normal DrivetrainCommand (see SharedFirmwareTypes.h), a "reset error" command,
 * and a "init" command.
 */
struct DrivetrainResetError_s
{
    bool reset_errors; // true: reset the errors present on inverters, false: dont
};

enum DrivetrainModeRequest_e 
{
    UNINITIALIZED = 0, // If sending a DrivetrainInit command with UNIITIALIZED, it will not initialize
    INIT_DRIVE_MODE = 1
};

struct DrivetrainInit_s 
{
    DrivetrainModeRequest_e init_drivetrain;
};

/**
 * The DrivetrainSystem is primarily responsible for two things:
 * 1) Updating its internal state machine
 * 2) Determining what commands to give each InverterInterface
 */

class DrivetrainSystem
{
public:
    /**
     * etl::variants allow multiple types to be treated as a single type-- almost like an enum of types.
     * Here, we're just saying that when we refer to CmdVariant, the parameter can be any one of these
     * three options.
     */
    using CmdVariant = etl::variant<DrivetrainCommand_s, DrivetrainInit_s, DrivetrainResetError_s>;
    DrivetrainSystem() = delete;
    
    /**
     * Functions for VSM state transitions (VSM needs to know drivetrain's status to trigger its
     * state transitions).
     */
    bool hv_over_threshold();
    bool drivetrain_error_present();
    bool drivetrain_ready();
    void reset_dt_error();

    /**
     * Drivetrain state machine (DSM) functions
     */
    DrivetrainStatus_s evaluate_drivetrain(CmdVariant cmd);
    DrivetrainState_e get_state();
    DrivetrainStatus_s get_status();

    struct InverterFuncts {
        std::function<void(float desired_rpm, float torque_limit_nm)> set_speed;
        std::function<void()> set_idle;
        std::function<void(InverterControlWord_s control_word)> set_inverter_control_word;
        std::function<InverterStatus_s()> get_status;
        std::function<MotorMechanics_s()> get_motor_mechanics; 
    };
    
    DrivetrainSystem(veh_vec<DrivetrainSystem::InverterFuncts> inverter_interfaces, etl::delegate<void(bool)> set_ef_active_pin, unsigned long ef_pin_enable_delay_ms = 50);
    
private:
    /**
     * Internal functions for handling DSM state transitions.
     */
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
    DrivetrainState_e _state;
    DrivetrainStatus_s _status;

    const float _active_rpm_level = 100;
    veh_vec<InverterFuncts> _inverter_interfaces;

    /**
     * Lambda functions defined on construction for the DSM state transitions.
     */
    std::function<bool(const InverterStatus_s &)> _check_inverter_ready_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_connected_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_quit_dc_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_no_errors_present;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_not_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_enabled;  

    /**
     * Delegate function for setting ef active
     */
    etl::delegate<void(bool)> _set_ef_active_pin;
    unsigned long _last_toggled_ef_active = 0; 
    unsigned long _ef_pin_enable_delay_ms;
};

#endif /* DRIVETRAINSYSTEM */