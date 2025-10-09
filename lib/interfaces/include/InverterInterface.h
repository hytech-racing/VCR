
/******************************************************************************
 * @file    InverterInterface.h
 * @brief   Header for any receive/send to the inverters
 ******************************************************************************/
#ifndef INVERTERINTERFACE_H
#define INVERTERINTERFACE_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include "FlexCAN_T4.h"
#include <hytech.h>
#include "DrivetrainSystem.h"
#include <CANInterface.h>
#include <shared_types.h>

/******************************************************************************
 * Public Struct Definitions
 ******************************************************************************/
/**
* @struct inverter_erter_params_s
* @brief contains all the static parameters for an inverter
*/
struct inverter_params_s {   
    float high_voltage_threshold;   /**< The voltage threshold above which the inverter is considered to be at high voltage, in V */
};

/**
* @struct inverter_can_ids_s
* @brief contains all the can ids relevant one inverter
*/
struct inverter_can_ids_s {
    uint32_t inverter_control_word_id; 
    uint32_t inverter_control_input_id; 
    uint32_t inverter_control_parameter_id; 
    uint32_t inverter_temps_id;
    uint32_t inverter_status_id;
    uint32_t inverter_dynamics_id;
};

/**
 * @struct inverter_control_word_s
 * @brief contains the control word to be sent to the inverter
 */
struct inverter_control_word_s {
    bool inverter_enable : 1;
    bool hv_enable : 1;
    bool driver_enable : 1;
    bool remove_error : 1;
};

/**
 * @struct inverter_control_input_s
 * @brief contains the control input to be sent to the inverter
 */
struct inverter_control_input_s {
    int16_t speed_rpm_setpoint;
    float positive_torque_limit; 
    float negative_torque_limit;
};


/**
 * @struct inverter_control_params_ms
 * @brief contains the inverter control params
 */
struct inverter_control_params_s { 
    uint16_t speed_control_kp; 
    uint16_t speed_control_ki;
    uint16_t speed_control_kd;
};

/**
 * @struct inverter_status_s
 * @brief contains the inverter status data
 */
struct inverter_status_s {
    bool new_data : 1;
    unsigned long last_recv_millis = 0; 
    bool hv_present : 1;
    bool connected : 1;
    bool system_ready : 1;
    bool error : 1;
    bool warning : 1;
    bool quit_dc_on : 1;
    bool dc_on : 1;
    bool quit_inverter_on : 1;
    bool inverter_on : 1;
    bool derating_on : 1;
    float dc_bus_voltage;
    uint16_t diagnostic_number;
};

/**
 * @struct inverter_temps_s
 * @brief contains the inverter temps data
 */
struct inverter_temps_s {
    bool new_data : 1;
    unsigned long last_recv_millis = 0; 
    float motor_temp_celcius;
    float inverter_temp_celcius;
    float igbt_temp_celcius;
};

/**
 * @struct inverter_power_s
 * @brief contains the inverter power data
 */
struct inverter_power_s {
    bool new_data : 1;
    unsigned long last_recv_millis = 0; 
    float active_power_watts;
    float reactive_power_watts; // TODO check units
};

/**
 * @struct inverter_motor_mechanics_s
 * @brief contains the motor mechanics data
 */
struct inverter_motor_mechanics_s {
    bool new_data : 1;
    unsigned long last_recv_millis = 0; 
    float actual_power_watts;
    float actual_torque_nm;
    float actual_speed_rpm;
};

/**
 * @struct inverter_control_feedback_s
 * @brief contains the inverter control feedback data
 */
struct inverter_control_feedback_s {
    bool new_data : 1;
    unsigned long last_recv_millis = 0; 
    uint16_t speed_control_kp;
    uint16_t speed_control_ki;
    uint16_t speed_control_kd;
};

/**
 * @struct inverter_feedback_data_s
 * @brief contains all the feedback data from the inverter (in the form of nested structs)
 */
struct inverter_feedback_data_s {
    inverter_status_s status;
    inverter_temps_s temps;
    inverter_power_s power;
    inverter_motor_mechanics_s motor_mechanics;
    inverter_control_feedback_s control_feedback;
};


/******************************************************************************
 * Public Class Declarations
 ******************************************************************************/
/**
 * @class InverterInterface
 * A single instance of this class encapsulates all the methods for sending/geting data from a single inverter
 */
class InverterInterface {

    public: 
        /**
         * Constructs an instance of the inverter interface
         * @param inverter_control_word_id the CAN ID for the inverter control word
         * @param inverter_control_input_id the CAN ID for the inverter control input
         * @param inverter_control_params_id the CAN ID for the inverter control parameters
         * @param inverter_params the static parameters for the inverter
         */
        InverterInterface(
            uint32_t inverter_control_word_id,
            uint32_t inverter_control_input_id,
            uint32_t inverter_control_params_id,
            inverter_params_s inverter_params) : _inverter_params(inverter_params) { 
            _inverter_ids.inverter_control_word_id = inverter_control_word_id;
            _inverter_ids.inverter_control_parameter_id = inverter_control_params_id;
            _inverter_ids.inverter_control_input_id = inverter_control_input_id;
        }

        /**
         * Receives and processes a status CAN message from the inverter, invoked by the 
         * CAN interface when a message with the correct ID is received
         * @param can_msg the CAN message to process
         * @param can_msg the CAN message to process
         * @param curr_millis the current time in milliseconds
         */
        void receiveInverterStatus(const CAN_message_t &can_msg, unsigned long curr_millis);

        /**
         * Receives and processes a temps CAN message from the inverter, invoked by the 
         * CAN interface when a message with the correct ID is received
         * @param can_msg the CAN message to process
         * @param can_msg the CAN message to process
         * @param curr_millis the current time in milliseconds
         */
        void receiveInverterTemps(const CAN_message_t &can_msg, unsigned long curr_millis);

        /**
         * Receives and processes a dynamics CAN message from the inverter, invoked by the 
         * CAN interface when a message with the correct ID is received
         * @param can_msg the CAN message to process
         * @param can_msg the CAN message to process
         * @param curr_millis the current time in milliseconds
         */
        void recieveInverterDynamics(const CAN_message_t &can_msg, unsigned long curr_millis);

        /**
         * Receives and processes a power CAN message from the inverter, invoked by the 
         * CAN interface when a message with the correct ID is received
         * @param can_msg the CAN message to process
         * @param can_msg the CAN message to process
         * @param curr_millis the current time in milliseconds
         */
        void receiveInverterPower(const CAN_message_t &can_msg, unsigned long curr_millis);

        /**
         * Receives and processes a feedback CAN message from the inverter, invoked by the CAN interface when a message with the correct ID is received
         * @param can_msg the CAN message to process
         * @param curr_millis the current time in milliseconds
         */
        void receiveInverterFeedback(const CAN_message_t &can_msg, unsigned long curr_millis);

        /**
         * Sends the current inverter control inputs to the inverter over CAN
         */
        void sendInverterSetpointCommand();

        /**
         * Sends the current inverter control word to the inverter over CAN
         */
        void sendInverterControlWord();

        /**
         * Sends the current inverter control parameters to the inverter over CAN
         */
        void sendInverterControlParams(); 

        /**
         * Sets the desired speed and torque limit for the inverter, populating the struct that is sent
         * by the sendInverterSetpointCommand method
         * @param desired_rpm the desired speed in RPM
         * @param torque_limit_nm the torque limit in Nm
         */
        void setSpeed(float desired_rpm, float torque_limit_nm); 

        /**
         * Sets the inverter to idle mode, which disables the inverter but keeps it ready to be re-enabled
         * without a full power cycle
         */
        void setIdle();

        /**
         * Sets the inverter control word, populating the struct that is sent by the sendInverterControlWord method
         * @param control_word the control word to set
         */
        void setInverterControlWord(inverter_control_word_s control_word);

        /**
         * Returns the current inverter status
         * @return the current inverter status
         */
        inverter_status_s getStatus(); 

        /**
         * Fetches the latest inverter temps
         * @return the latest inverter temps
         */
        inverter_temps_s getTemps();

        /**
         * Fetches the latest inverter power data
         * @return the latest inverter power data
         */
        inverter_power_s getPower();

        /**
         * Fetches the latest motor mechanics data
         * @return the latest motor mechanics data
         */
        inverter_motor_mechanics_s getMotorMechanics();

        /**
         * Fetches the latest inverter control params
         * @return the latest control params
         */
        inverter_control_feedback_s getControlFeedback();

    private: 

        inverter_can_ids_s _inverter_ids;
        inverter_control_input_s _inverter_control_inputs;
        inverter_control_word_s _inverter_control_word;
        inverter_control_params_s _inverter_control_params;
        inverter_feedback_data_s _feedback_data;
        inverter_params_s _inverter_params;
};

#endif // __INVERTERINTERFACE_H__