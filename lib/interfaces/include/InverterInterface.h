#ifndef INVERTERINTERFACE_H
#define INVERTERINTERFACE_H
#include <stdint.h>

#include "FlexCAN_T4.h"
#include "MessageQueueDefine.h"

#include <hytech.h>

namespace HTUnits
{
    using celcius = float;
    using watts = float;
    using var = float;
    using torque_nm = float;
    using speed_rpm = float;
    using volts = float;
};

/**
 * Struct containing id info for this specific inverter interface
 */
struct InverterIds_s
{
    uint32_t mc_setpoint_commands_id; 
    uint32_t mc_torque_command_id; 
};

/** 
 * Drivetrain system accessible structs for 
 * requesting change of state
 */
struct InverterSetpoints_s 
{
    bool inverter_enable : 1;
    bool hv_enable : 1;
    bool driver_enable : 1;
    bool remove_error : 1;
    int16_t speed_rpm_setpoint;
    int16_t positive_torque_limit; 
    int16_t negative_torque_limit;
};

struct InverterTorqueCommand_s
{
    uint16_t torque_command;
};

/** 
 * For the most part these are mirrors of the lower-level CAN struct data, 
 * except with already fully float-ized data
**/
struct InverterStatus_s
{
    bool new_data : 1;
    bool system_ready : 1;
    bool error : 1;
    bool warning : 1;
    bool quit_dc_on : 1;
    bool dc_on : 1;
    bool quit_inverter_on : 1;
    bool inverter_on : 1;
    bool derating_on : 1;
    HTUnits::volts dc_bus_voltage;
    uint16_t diagnostic_number;
};

struct InverterTemps_s
{
    bool new_data : 1;
    HTUnits::celcius motor_temp;
    HTUnits::celcius inverter_temp;
    HTUnits::celcius igbt_temp;
};

struct InverterPower_s
{
    bool new_data : 1;
    HTUnits::watts active_power;
    HTUnits::var reactive_power;
};

struct MotorMechanics_s
{
    bool new_data : 1;
    HTUnits::watts actual_power;
    HTUnits::torque_nm actual_torque;
    HTUnits::speed_rpm actual_speed;
};

struct InverterControlParams_s
{
    bool new_data : 1;
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

/**
 * Inverter interface
 */
class InverterInterface
{

    public: 

        InverterInterface(
            CANBufferType *msg_output_queue, 
            uint32_t mc_setpoint_commands_id,
            uint32_t mc_torque_command_id) : msg_queue_(msg_output_queue) 
        { 
            inverter_ids.mc_setpoint_commands_id = mc_setpoint_commands_id;
            inverter_ids.mc_torque_command_id = mc_torque_command_id;
        }

        /* Request change of state */
        void set_inverter_setpoints(
            bool inverter_enable,
            bool hv_enable,
            bool driver_enable,
            bool remove_error,
            int16_t speed_rpm_setpoint,
            int16_t positive_torque_limit,
            int16_t negative_torque_limit
        );

        void set_torque_command(
            uint16_t torque_command
        );

        // TODO un-public these (they are public for testing)

        /* Recieving callbacks */
        void recieve_MCI_STATUS(CAN_message_t &can_msg);

        void recieve_MCI_TEMPS(CAN_message_t &can_msg);

        void recieve_MCI_DYNAMICS(CAN_message_t &can_msg);

        void recieve_MCI_POWER(CAN_message_t &can_msg);

        void recieve_MCI_FEEDBACK(CAN_message_t &can_msg);


    private: 

        InverterIds_s inverter_ids;

        InverterSetpoints_s _inverter_setpoints;
        InverterTorqueCommand_s _inverter_torque_command;

        // InverterStatus_s _inverter_status;
        // InverterTemps_s _inverter_temps;
        // InverterPower_s _inverter_power;
        // InverterControlParams_s _control_params;
        InverterFeedbackData_s _feedback_data;

        /* Getters */
        InverterStatus_s get_status(); 
        InverterTemps_s get_temps();
        InverterPower_s get_power();
        MotorMechanics_s get_motor_mechanics();
        InverterControlParams_s get_control_params();

        /* Sending */
        template <typename U>
        void enqueue_new_CAN(U *structure, uint32_t (*pack_function)(U *, uint8_t *, uint8_t *, uint8_t *), uint32_t id);

        void send_MC_SETPOINT_COMMAND();

        void send_MC_TORQUE_COMMAND();

        CANBufferType *msg_queue_;
};
#endif // __INVERTERINTERFACE_H__

