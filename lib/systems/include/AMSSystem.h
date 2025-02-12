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
const unsigned long PACK_CHARGE_CRIT_TOTAL_THRESHOLD_VOLTS = 420;

/* The lowest pcc threshold is the lowest allowable single cell voltage */
constexpr float PACK_CHARGE_CRIT_LOWEST_CELL_THRESHOLD = 3.2f;             // Volts

constexpr float DEFAULT_INIT_TEMP       = 40.0f;                           // Celsius
constexpr float DEFAULT_INIT_VOLTAGE    = 3.5f;                            // Volts
constexpr float DEFAULT_TEMP_ALPHA      = 0.75f;                           // IIR filter alpha
constexpr float DEFAULT_VOLTAGE_ALPHA   = 0.8f;                            // IIR filter alpha
constexpr uint16_t MAX_PACK_CHARGE      = 48600;                           // Coulombs

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
    AMSSystem() :
        _last_heartbeat_time_ms(0),
        _cell_temp_alpha(DEFAULT_TEMP_ALPHA),
        _cell_voltage_alpha(DEFAULT_VOLTAGE_ALPHA)
    {};

    /**
     * Initializes the heartbeat timer.
     * 
     * @return The initial AMSSystemData_s state (before receiving any data). This will be the same every time this is called.
     */
    AMSSystemData_s init(unsigned long curr_millis);

    /**
     * Primary function to recalculate the shutdown conditions.
     * @param curr_millis The current timestmap, in milliseconds.
     * @param interface_data All current interface data, including ACUCoreData_s and ACUAllData_s.
     */
    AMSSystemData_s update_ams_system(unsigned long curr_millis, VCRData_s &interface_data);

private:

    /* AMS last heartbeat time */
    unsigned long _last_heartbeat_time_ms;

    /* IIR alpha values */
    float _cell_temp_alpha;
    float _cell_voltage_alpha;

    /**
     * Returns true if heartbeat is OK, false otherwise. Also resets
     * the _last_heartbeat_time_ms to be curr_millis.
     */
    bool _check_heartbeat_ok(unsigned long curr_millis);

};

using AMSSystemInstance = etl::singleton<AMSSystem>;

#endif /* AMS_SYSTEM_H */