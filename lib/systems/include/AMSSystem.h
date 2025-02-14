#ifndef AMS_SYSTEM_H
#define AMS_SYSTEM_H

/* Standard int library */
#include <stdint.h>

/* From shared-firmware-types library */
#include <SharedFirmwareTypes.h>

/* From ETL library */
#include "etl/singleton.h"

/* Heartbeat Interval is the allowable amount of milliseconds between BMS status messages before car delatches */
const unsigned long HEARTBEAT_INTERVAL_MS                  = 2000;    // milliseconds

/* The total PCC threshold is the lowest allowable voltage of the entire pack (in Volts) */
constexpr float PACK_CHARGE_CRITICAL_THRESHOLD_VOLTS = 420;

/* The lowest pcc threshold is the lowest allowable single cell voltage */
constexpr float CELL_CHARGE_CRITICAL_THRESHOLD_VOLTS = 3.0f;             // Volts

/**
 * Singleton class for communicating with the BMS. If one of the shutdown conditions is met, this class will return a
 * false for get_bms_ok.
 * 
 * Shutdown conditions:
 * 1) Has not received a message from BMS for more than "HEARTBEAT_INTERVAL_MS" milliseconds.
 * 2) Total pack voltage is below critical threshold.
 * 3) Lowest cell voltage is below critical threshold.
 */
class AMSSystem
{
public:

    /**
     * Constructor for the AMS Interface
     */
    AMSSystem(unsigned long heartbeat_interval_ms, float pack_charge_critical_threshold_volts, float cell_charge_critical_threshold_volts) :
        _has_received_one_message(false),
        _heartbeat_interval_ms(heartbeat_interval_ms),
        _pack_charge_critical_threshold_volts(pack_charge_critical_threshold_volts),
        _cell_charge_critical_threshold_volts(cell_charge_critical_threshold_volts)
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

    /**
     * Minimum allowable voltage on the total pack before triggering shutdown.
     */
    float _pack_charge_critical_threshold_volts;

    /**
     * Minimum allowable voltage on the lowest cell before triggering shutdown.
     */
    float _cell_charge_critical_threshold_volts;

    /**
     * Returns true if heartbeat is OK, false otherwise. Also resets
     * the _last_heartbeat_time_ms to be curr_millis.
     */
    bool _check_heartbeat_ok(unsigned long curr_millis);

};

using AMSSystemInstance = etl::singleton<AMSSystem>;

#endif /* AMS_SYSTEM_H */