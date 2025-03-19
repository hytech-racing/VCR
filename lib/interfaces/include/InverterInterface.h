#ifndef INVERTERINTERFACE_H
#define INVERTERINTERFACE_H
#include <stdint.h>

#include "FlexCAN_T4.h"

#include <hytech.h>
#include "DrivetrainSystem.h"
#include <CANInterface.h>
#include <shared_types.h>

struct InverterParams_s
{   
    float MINIMUM_HV_VOLTAGE; 
};

/**
 * Struct containing id info for this specific inverter interface
 */
struct InverterCANIds_s
{
    uint32_t inv_control_word_id; 
    uint32_t inv_control_input_id; 
    uint32_t inv_control_parameter_id; 
};

/**
 * Inverter interface
 */
class InverterInterface
{

    public: 
        
        InverterInterface(
            uint32_t inv_control_word_id,
            uint32_t inv_control_input_id,
            uint32_t inv_control_params_id,
            InverterParams_s inverter_params) : _inverter_params(inverter_params)
        { 
            inverter_ids.inv_control_word_id = inv_control_word_id;
            inverter_ids.inv_control_parameter_id = inv_control_params_id;
            inverter_ids.inv_control_input_id = inv_control_input_id;
        }

        /* receiving callbacks */
        void receive_INV_STATUS(const CAN_message_t &can_msg, unsigned long curr_millis);

        void receive_INV_TEMPS(const CAN_message_t &can_msg, unsigned long curr_millis);

        void receive_INV_DYNAMICS(const CAN_message_t &can_msg, unsigned long curr_millis);

        void receive_INV_POWER(const CAN_message_t &can_msg, unsigned long curr_millis);

        void receive_INV_FEEDBACK(const CAN_message_t &can_msg, unsigned long curr_millis);

        /* Sending */
        void send_INV_SETPOINT_COMMAND();

        void send_INV_CONTROL_WORD();

        void send_INV_CONTROL_PARAMS(); 

        /* Inverter Functs */
        void set_speed(float desired_rpm, float torque_limit_nm); 

        void set_idle();

        void set_inverter_control_word(InverterControlWord_s control_word);

        /* Getters */
        InverterStatus_s get_status(); 
        InverterTemps_s get_temps();
        InverterPower_s get_power();
        MotorMechanics_s get_motor_mechanics();
        InverterControlFeedback_s get_control_params();

    private: 

        InverterCANIds_s inverter_ids;
        InverterControlInput_s _inverter_control_inputs;
        InverterControlWord_s _inverter_control_word;
        InverterControlParams_s _inverter_control_params;
        InverterFeedbackData_s _feedback_data;
        InverterParams_s _inverter_params;
};


#endif // __INVERTERINTERFACE_H__
