/******************************************************************************
 * @file    ACUInterface.h
 * @brief   Header for any receive/send to the ACU
 ******************************************************************************/
#ifndef ACUINTERFACE_H
#define ACUINTERFACE_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "FlexCAN_T4.h"
#include "shared_types.h"
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

/******************************************************************************
 * Public Struct Definitions
 ******************************************************************************/
 /**
  * @struct ACUCANInterfaceData_s
  * @brief contains all the data received from the ACU over CAN
  */
struct ACUCANInterfaceData_s {
    bool bms_ok;
    bool imd_ok;
    uint64_t last_recv_millis; 
    bool heartbeat_ok;
};

/******************************************************************************
 * Public Class Declarations
 ******************************************************************************/
/**
 * @class ACUInterface
 * A single instance of this class encapsulates all the methods for sending/geting data from ACU
 */
class ACUInterface {
public:

    ACUInterface() = delete;

    /**
     * Constructs an instance of the ACU interface
     * @param init_millis the initial time in milliseconds
     * @param max_heartbeat_interval_millis the maximum time in milliseconds allowed between ACU heartbeat
     */
    ACUInterface(uint32_t init_millis, uint32_t max_heartbeat_interval_millis) : _max_heartbeat_interval_ms(max_heartbeat_interval_millis) { 
        _curr_data.bms_ok = _curr_data.imd_ok = false;
        _curr_data.last_recv_millis = 0;
    };

    /**
     * Processes an ACU CAN message, invoked by the CAN interface when a message with the correct ID is received
     * @param msg the CAN message to process
     * @param curr_millis the current time in milliseconds
     */
    void receiveAcuOkMessage(const CAN_message_t &msg, unsigned long curr_millis);

    /**
     * Returns whether or not an IMD fault has been detected
     * @return true if no IMD fault, false if IMD fault detected
     */
    bool isImdOk() { return _curr_data.imd_ok; }

    /**
     * Returns whether or not a BMS fault has been detected
     * @return true if no BMS fault, false if BMS fault detected
     */
    bool isBmsOk() { return _curr_data.bms_ok; }

    /**
     * Returns the time in milliseconds of the last received ACU message
     * @return time in milliseconds of last received ACU message
     */
    uint64_t getLastReceivedMessageTimeMillis() { return _curr_data.last_recv_millis; }

    /**
     * Returns whether or not the interface has received the first ACU heartbeat message
     * @return true if the first heartbeat has been received, false otherwise
     */
    bool hasReceivedFirstAcuHeartbeat() { return _received_first_acu_heartbeat; }

    /**
     * Returns the latest ACU data
     * @param curr_millis the current time in milliseconds
     * @return the latest ACU data
     */
    ACUCANInterfaceData_s getLatestData(uint64_t curr_millis);

private:

    ACUCANInterfaceData_s _curr_data;
    bool _received_first_acu_heartbeat = false;
    const uint32_t _max_heartbeat_interval_ms;
};

using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif