#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

#include "etl/variant.h"
#include "etl/delegate.h"
#include "etl/singleton.h"
#include <array>
#include <functional>
#include "stdint.h"
#include "SharedFirmwareTypes.h"
#include "SysClock.h"
#include <shared_types.h>
#include "SystemTimeInterface.h"
#include "InverterInterface.h"

constexpr float MIN_ACTIVE_RPM = 100.0;
constexpr int INVERTER_ENABLE_PIN = 2;

// requirements:
// - [x] must support ability to initialize the drivetrain 
// - [x] ability to command inverters individually and be able to return a failure status when attempting to send invalid command for a certain state
// - [x] contain a state machine for managing the state of the drivetrain as a whole (aka: all inverters have the same state)
//  - [x] initialization states included
//  - [ ] different control mode states
// - [x] single point of interaction / control of the drivetrain that can receive "commands"
//      (at least for now, need to see how this works out once we start using it)
// - [x] be decoupled from the inverter class
//      std::function / etl::delegate registered functions for the inverter interface. mostly for ease of testing.
// - [x] be able to reset drivetrain
    // - [ ] 

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

class DrivetrainSystem {
    public:
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
        
        DrivetrainSystem(veh_vec<InverterInterface> inverter_interfaces, etl::delegate<void(bool)> set_ef_active_pin, unsigned long ef_pin_enable_delay_ms = 50);
        
    private:
        /**
         * Internal functions for handling DSM state transitions.
         */
        void _setState(DrivetrainState_e state);
        void _setDrivetrainDisabled();
        void _setDrivetrainKeepaliveIdle();
        void _setDrivetrainCommand(DrivetrainCommand_s);
        void _setEnableDrivetrainHv();
        void _setEnableDrivetrain();
        void _setDrivetrainErrorReset();
        
        void _setEFActive(bool set_active);

        bool _checkInvertersConnected(); 
        bool _checkHvPresent(); 
        bool _checkInvertersReady(); 
        bool _checkQuitDcOn(); 
        bool _checkErrorPresent();
        bool _checkDrivetrainActive();
        bool _checkInvertersEnabled();

        DrivetrainState_e _evaluate_state_machine(CmdVariant cmd);

        DrivetrainState_e _state;
        DrivetrainStatus_s _status;
        veh_vec<InverterInterface> _inverter_interfaces;

        bool _init_drivetrain_flag; 
        bool _reset_errors_flag; 
        bool _idle_flag; 
        bool _set_command_flag; 

        /**
         * Delegate function for setting ef active
         */
        etl::delegate<void(bool)> _set_ef_active_pin;
        unsigned long _last_toggled_ef_active = 0; 
        unsigned long _ef_pin_enable_delay_ms;
};

using DrivetrainInstance = etl::singleton<DrivetrainSystem>;

#endif /* DRIVETRAINSYSTEM */