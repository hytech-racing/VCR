#ifndef AMS_SYSTEM_H
#define AMS_SYSTEM_H

/* Standard int library */
#include <stdint.h>

/* From shared-firmware-types library */
#include <SharedFirmwareTypes.h>

/* From ETL library */
#include "etl/singleton.h"

/* Heartbeat Interval is the allowable amount of milliseconds between BMS status messages before car delatches */
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 2000;

/**
 * Singleton class for communicating with the BMS. If one of the shutdown conditions is met, this class will turn
 * ams_ok false in the returned AMSSystemData_s struct.
 * 
 * Shutdown conditions:
 * 1) AMS heartbeat times out (StampedACUCoreData's timestamp exceeds heartbeat interval). 2000ms by default.
 */
class AMSSystem
{
public:

    /**
     * Constructor for the AMS Interface
     */
    AMSSystem(unsigned long heartbeat_interval_ms) :
        _has_received_one_message(false),
        _heartbeat_interval_ms(heartbeat_interval_ms)
    {};

    /**
     * Primary function to recalculate the shutdown conditions.
     * @param curr_millis The current timestmap, in milliseconds.
     * @param interface_data All current interface data, including ACUCoreData_s and ACUAllData_s.
     */
    AMSSystemData_s update_ams_system(unsigned long curr_millis, VCRData_s &interface_data);

private:

    /**
     * Initialized to false. Becomes true as soon as at least one ACU message has been received.
     */
    bool _has_received_one_message;

    /**
     * Maximum allowable time between two messages from ACU before triggering shutdown.
     */
    unsigned long _heartbeat_interval_ms;

};

using AMSSystemInstance = etl::singleton<AMSSystem>;

#endif /* AMS_SYSTEM_H */