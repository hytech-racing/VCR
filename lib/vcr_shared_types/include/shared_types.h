#ifndef __SHARED_TYPES_H__
#define __SHARED_TYPES_H__
#include <stdint.h>
namespace HTUnits
{
    using celcius = float;
    using watts = float;
    using var = float;
    using torque_nm = float;
    using speed_rpm = float;
    using volts = float;
};

// for the most part these are mirrors of the lower-level CAN struct data, except with already fully float-ized data

struct InverterStatus_s
{
    bool system_ready : 1;
    bool error : 1;
    bool warning : 1;
    bool quit_dc_on : 1;
    bool dc_on : 1;
    bool quit_inverter_on : 1;
    bool inverter_on : 1;
    bool derating_on : 1;
    bool connected : 1; // if we have received a CAN message from the inverter
    HTUnits::volts dc_bus_voltage;
    uint16_t diagnostic_number;
    int16_t speed_rpm;
};

struct InverterTemps_s
{
    HTUnits::celcius motor_temp;
    HTUnits::celcius inverter_temp;
    HTUnits::celcius igbt_temp;
};

struct InverterPower_s
{
    HTUnits::watts active_power;
    HTUnits::var reactive_power;
};

struct MotorMechanics_s
{
    HTUnits::watts actual_power;
    HTUnits::torque_nm actual_torque;
    HTUnits::speed_rpm actual_speed;
};

struct InverterControlParams_s
{
    uint16_t speed_control_kp;
    uint16_t speed_control_ki;
    uint16_t speed_control_kd;
};

struct InverterFeedbackData_s
{
    InverterStatus_s status;
    InverterTemps_s temps;
    InverterPower_s power;
    MotorMechanics_s motor_mechanics;
    InverterControlParams_s control_params;
};

struct TimestampedData_s
{
    unsigned long last_recv_millis;
};






#endif // __SHARED_TYPES_H__