#ifndef DRIVETRAINSYSTEM
#define DRIVETRAINSYSTEM

#include "etl/singleton.h"

#include "SharedFirmwareTypes.h"
#include <array>
#include "stdint.h"
#include "SysClock.h"

enum class DRIVETRAIN_STATE
{
    TRACTIVE_SYSTEM_NOT_ACTIVE = 0,
    TRACTIVE_SYSTEM_ACTIVE = 1,
    ENABLING_INVERTERS = 2,
    ENABLED = 3,
    ERROR = 4
};

// #include "DrivetrainSystemStateMachine.h"
/**
 * As of now, only the minimum functions for VehicleStateMachine to compile have been implemented.
 * TODO: Re-add the rest of the necessary functions
 * TODO: Add DrivetrainSystem.tpp to implement all functions
 */
template <typename InverterType>
class DrivetrainSystem
{
public:
    DrivetrainSystem();
    
    void tick(const SysTick_s &tick);

    void setup_retry()
    {
        reset_drivetrain();
        _hv_en_requested = false;
        _enable_requested = false;
    }

    bool handle_inverter_startup(unsigned long curr_time);

    // on entry logic
    void command_drivetrain_no_torque();
    void command_drivetrain_debug();

    // check to see if init time limit has passed
    bool inverter_init_timeout(unsigned long curr_time);

    bool hv_over_threshold_on_drivetrain();
    void disable();
    void disable_no_pins();
    bool drivetrain_error_occured();
    void reset_drivetrain();
    // void command_drivetrain(const DrivetrainCommand_s &data);

    void enable_drivetrain_reset();
    void check_reset_condition();

    // DrivetrainDynamicReport_s get_dynamic_data();

private:
    // DrivetrainSystem(const std::array<InverterType *, 4> &inverters, MCUInterface *mcu_interface, int init_time_limit_ms, uint16_t min_hv_voltage = 60, int min_cmd_period_ms = 1, float max_torque_setpoint_nm = 21.42)
    //     : inverters_(inverters), init_time_limit_ms_(init_time_limit_ms), min_hv_voltage_(min_hv_voltage), min_cmd_period_(min_cmd_period_ms), max_torque_setpoint_nm_(max_torque_setpoint_nm)
    // {
    //     // values from: https://www.amk-motion.com/amk-dokucd/dokucd/en/content/resources/pdf-dateien/fse/motor_data_sheet_a2370dd_dd5.pdf
    //     motor_pole_pairs_ = 5;
    //     lambda_magnetic_flux_wb_ = 1.0;
    //     hv_en_requested_ = false;
    //     enable_requested_ = false;
    //     reset_requested_ = false;
    //     last_reset_pressed_time_ = 0;
    //     reset_interval_ = 5000;     // ms
    //     curr_system_millis_ = 0;
    //     last_no_torque_cmd_time_ = 0;
    //     last_reset_cmd_time_ = 0;
    //     last_disable_cmd_time_ = 0;
    //     last_general_cmd_time_ = 0; // ms
    //     mcu_interface_ = mcu_interface;
    //     dynamic_data_ = {};
    // }

    std::array<InverterType *, 4> _inverters;
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

#include "DrivetrainSystem_old.tpp"

// using DrivetrainSystemSingleton<> = etl::singleton<DrivetrainSystem<>>;
#endif /* DRIVETRAINSYSTEM */