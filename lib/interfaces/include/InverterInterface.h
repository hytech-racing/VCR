#ifndef INVERTERINTERFACE_H
#define INVERTERINTERFACE_H
#include <stdint.h>

#include "FlexCAN_T4.h"
#include "MessageQueueDefine.h"

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
struct InverterIds_s
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
            CANBufferType *msg_output_queue, 
            uint32_t inv_control_word_id,
            uint32_t inv_control_input_id,
            uint32_t inv_control_params_id,
            InverterParams_s inverter_params) : msg_queue_(msg_output_queue), _inverter_params(inverter_params)
        { 
            inverter_ids.inv_control_word_id = inv_control_word_id;
            inverter_ids.inv_control_parameter_id = inv_control_params_id;
            inverter_ids.inv_control_input_id = inv_control_input_id;
        }

        // TODO un-public these (they are public for testing)

        /* receiving callbacks */
        void receive_INV_STATUS(CAN_message_t &can_msg);

        void receive_INV_TEMPS(CAN_message_t &can_msg);

        void receive_INV_DYNAMICS(CAN_message_t &can_msg);

        void receive_INV_POWER(CAN_message_t &can_msg);

        void receive_INV_FEEDBACK(CAN_message_t &can_msg);

        /* Sending */
        void send_INV_SETPOINT_COMMAND();

        void send_INV_CONTROL_WORD();

        void send_INV_CONTROL_PARAMS(); 

        /* Inverter Functs */
        void set_speed(float desired_rpm, float torque_limit_nm); 

        void set_idle();

        void set_inverter_control_word(InverterControlWord_s control_word);

        InverterStatus_s get_inverter_status();

    private: 

        InverterIds_s inverter_ids;
        InverterControlInput_s _inverter_control_inputs;
        InverterControlWord_s _inverter_control_word;
        InverterControlParams_s _inverter_control_params;
        InverterFeedbackData_s _feedback_data;
        InverterParams_s _inverter_params;

        /* Getters */
        InverterStatus_s get_status(); 
        InverterTemps_s get_temps();
        InverterPower_s get_power();
        MotorMechanics_s get_motor_mechanics();
        InverterControlFeedback_s get_control_params();

        CANBufferType *msg_queue_;
};
#endif // __INVERTERINTERFACE_H__