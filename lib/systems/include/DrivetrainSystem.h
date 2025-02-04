#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

#include "etl/variant.h"

#include <array>
#include <functional>
#include "stdint.h"

#include "SharedFirmwareTypes.h"
#include "SysClock.h"


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
    ENABLING_INVERTERS_SPEED_MODE = 6,
    ENABLING_INVERTERS_TORQUE_MODE = 7,
    ENABLED_SPEED_MODE = 8,
    ENABLED_TORQUE_MODE = 9,
    ERROR = 10
};

enum class DrivetrainCmdResponse_e
{
    COMMAND_OK = 0,
    CANNOT_INIT_NOT_CONNECTED = 1,
    COMMAND_INVALID = 2
};

struct DrivetrainSpeedCommand_s
{
    veh_vec<float> desired_speed_rpm;
    veh_vec<float> torque_limit_nm;
};

struct DrivetrainTorqueCommand_s
{
    veh_vec<float> desired_torque_nm;
};

enum class DrivetrainModeRequest_e
{
    UNINITIALIZED = 0,
    INIT_SPEED_MODE = 1,
    INIT_TORQUE_MODE =2
};

struct DrivetrainInit_s
{
    DrivetrainModeRequest_e init_drivetrain;
};

struct InverterStatus_s
{
    float dc_bus_voltage;
    float torque_nm;
    float speed_rpm;
    float mech_power_w;
    float inverter_temp_c; 
    float motor_temp_c;
    float igbt_temp_c;
    uint16_t error_status_id;
    bool inverter_ready : 1;
    bool quit_dc : 1;
    bool quit_inverter : 1;
    bool error_present : 1;
    bool connected : 1;
    bool hv_present : 1;
};

struct DrivetrainStatus_s
{
    bool all_inverters_connected;
    veh_vec<InverterStatus_s> inverter_statuses;
    DrivetrainCmdResponse_e cmd_resp;
};

struct DrivetrainResetError_s
{
    bool reset_errors; // true: reset the errors present on inverters, false: dont
};

// output pin of micro, read by inverters
struct DrivetrainOutputPins_s
{
    bool torque_mode_pin_state : 1;
};

// the pin set by the inverters themselves ("input": pin being read by micro)
struct DrivetrainInputPins_s
{
    bool torque_mode_enabled_pin_state : 1;
};

struct InverterControlWord_s
{
    bool inverter_enable;
    bool hv_enable;
    bool driver_enable;
    bool remove_error;
};

class DrivetrainSystem
{
public:
    using CmdVariant = etl::variant<DrivetrainSpeedCommand_s, DrivetrainTorqueCommand_s, DrivetrainInit_s, DrivetrainResetError_s>;
    DrivetrainSystem() = delete;

    DrivetrainStatus_s evaluate_drivetrain(CmdVariant cmd);
    DrivetrainState_e get_state();
    // DrivetrainDynamicReport_s get_dynamic_data();

    struct InverterFuncts {
        std::function<void(float desired_rpm, float torque_limit_nm)> set_speed;
        std::function<void(float torque_nm)> set_torque;
        std::function<void()> set_idle;
        std::function<void(InverterControlWord_s control_word)> set_inverter_control_word;
        std::function<InverterStatus_s()> get_status;
    };
    
    DrivetrainSystem(veh_vec<DrivetrainSystem::InverterFuncts> inverter_interfaces);
private:
    bool _check_inverter_flags(std::function<bool(const InverterStatus_s&)> flag_check_func);
    bool _drivetrain_active(float max_active_rpm);

    void _set_state(DrivetrainState_e state);
    void _handle_exit_logic(DrivetrainState_e prev_state);
    void _handle_entry_logic(DrivetrainState_e new_state);
    
    void _set_drivetrain_disabled();
    void _set_drivetrain_keepalive_idle();
    void _set_enable_drivetrain_hv();
    void _set_enable_drivetrain();
    void _set_drivetrain_error_reset();
    
    void _set_drivetrain_speed_command(DrivetrainSpeedCommand_s cmd);
    void _set_drivetrain_torque_command(DrivetrainTorqueCommand_s cmd);

    DrivetrainState_e _evaluate_state_machine(CmdVariant cmd);

private:
    const float _active_rpm_level = 100;
    veh_vec<InverterFuncts> _inverter_interfaces;
    DrivetrainState_e _state;
    std::function<bool(const InverterStatus_s &)> _check_inverter_ready_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_connected_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_quit_dc_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_error_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_not_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_enabled;
    
    std::function<void(const DrivetrainOutputPins_s &)> _set_gpio_state;
    std::function<DrivetrainInputPins_s()> _get_gpio_state;
    
};

#endif /* DRIVETRAINSYSTEM */