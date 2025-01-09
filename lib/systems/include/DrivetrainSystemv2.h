#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

#include <array>
#include <functional>
#include "stdint.h"

#include "SharedFirmwareTypes.h"
#include "SysClock.h"

enum class drivetrain_state
{
    NOT_ENABLED_NO_HV_PRESENT = 0,
    NOT_ENABLED_HV_PRESENT = 1,
    ENABLING_INVERTERS_SPEED_MODE = 2,
    ENABLING_INVERTERS_TORQUE_MODE = 3,
    ENABLED_SPEED_MODE = 4,
    ENABLED_TORQUE_MODE = 5,
    ERROR = 6
};

// #include "DrivetrainSystemStateMachine.h"
/**
 * As of now, only the minimum functions for VehicleStateMachine to compile have been implemented.
 * TODO: Re-add the rest of the necessary functions
 * TODO: Add DrivetrainSystem.tpp to implement all functions
 */

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
};

class DrivetrainSystem
{
public:
    DrivetrainSystem() =delete;
    DrivetrainSystem(veh_vec<InverterFuncts> inverter_interfaces);
    
    void command_drivetrain(etl::variant<DrivetrainSpeedCommand_s, DrivetrainTorqueCommand_s> cmd);
    // DrivetrainDynamicReport_s get_dynamic_data();

    struct InverterFuncts {
        std::function<void(float desired_rpm, float torque_limit_nm)> _set_speed;
        std::function<void(float torque_limit_nm)> _set_torque;
        std::function<void()> _set_disable_inverter;
        std::function<void()> _set_no_torque;
        std::function<void()> _reset_inverter;
        std::function<void()> _set_enable_inverter;
        std::function<void()> _set_enable_hv;
        std::function<InverterStatus_s()> _get_status;
    };
    
    // struct DrivetrainInterfaceState_s
    // {

    // };
private:
    veh_vec<InverterFuncts> _inverter_interfaces;
    // std::function<>;
    
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

#include "DrivetrainSystem.tpp"
#endif /* DRIVETRAINSYSTEM */