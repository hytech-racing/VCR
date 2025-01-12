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


enum class DrivetrainState_e
{
    NOT_CONNECTED = 0,
    NOT_ENABLED_NO_HV_PRESENT = 1,
    NOT_ENABLED_HV_PRESENT = 2,
    ENABLING_INVERTERS_SPEED_MODE = 3,
    ENABLING_INVERTERS_TORQUE_MODE = 4,
    ENABLED_SPEED_MODE = 5,
    ENABLED_TORQUE_MODE = 6,
    ERROR = 7
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

struct DrivetrainInit_s
{
    bool init_drivetrain;
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

class DrivetrainSystem
{
public:
    using CmdVariant = etl::variant<DrivetrainSpeedCommand_s, DrivetrainTorqueCommand_s, DrivetrainInit_s>;
    DrivetrainSystem() = delete;
    
    DrivetrainStatus_s evaluate_drivetrain(CmdVariant cmd);
    DrivetrainState_e get_state();
    // DrivetrainDynamicReport_s get_dynamic_data();

    struct InverterFuncts {
        std::function<void(float desired_rpm, float torque_limit_nm)> set_speed;
        std::function<void(float torque_limit_nm)> set_torque;
        std::function<void()> set_disable_inverter;
        std::function<void()> set_no_torque;
        std::function<void()> reset_inverter;
        std::function<void()> set_enable_inverter;
        std::function<void()> set_enable_hv;
        std::function<InverterStatus_s()> get_status;
    };
    
    // struct DrivetrainInterfaceState_s
    // {

    // };
private:
    bool _check_inverter_flags(std::function<bool(const InverterStatus_s&)> flag_check_func);
private:
    DrivetrainState_e _state;
    veh_vec<InverterFuncts> _inverter_interfaces;
    std::function<bool(const InverterStatus_s &)> _check_inverter_ready_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_connected_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_quit_dc_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_error_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_present_flag;
    std::function<bool(const InverterStatus_s &)> _check_inverter_hv_not_present_flag;

    // MCUInterface *mcu_interface_;
    int _init_time_limit_ms;
    uint16_t _min_hv_voltage;
    int _motor_pole_pairs;
    float _lambda_magnetic_flux_wb, _L_d_inductance_H;
    // startup statuses:
    bool _hv_en_requested, _enable_requested;
    // reset inverters
    bool _reset_requested;
    unsigned long _last_reset_pressed_time;
    unsigned long _reset_interval;
    /// @param curr_time current system tick time (millis()) that sets the init phase start time
    void enable_drivetrain_hv(unsigned long curr_time);
    void request_enable();
    // startup phase 1
    // status check for start of enable
    bool drivetrain_ready();
    // startup phase 2
    bool check_drivetrain_quit_dc_on();

    // final check for drivetrain initialization to check if quit inverter on
    bool drivetrain_enabled();

    unsigned long _curr_system_millis;
    unsigned int _min_cmd_period;
    unsigned long _last_no_torque_cmd_time, last_reset_cmd_time, last_disable_cmd_time, last_general_cmd_time;

    unsigned long _drivetrain_initialization_phase_start_time;
    // DrivetrainCommand_s current_drivetrain_command_;
    // DrivetrainDynamicReport_s dynamic_data_;
    float _max_torque_setpoint_nm;
};

#endif /* DRIVETRAINSYSTEM */