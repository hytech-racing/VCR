#ifndef AMS_SYSTEM_H
#define AMS_SYSTEM_H

#include <stdint.h>
#include <SharedFirmwareTypes.h>

/* Heartbeat Interval is the allowable amount of milliseconds between BMS status messages before car delatches */
const unsigned long HEARTBEAT_INTERVAL_MS                  = 2000;    // milliseconds

/* The total PCC threshold is the lowest allowable voltage of the entire pack (in Volts)*/
const unsigned long PACK_CHARGE_CRIT_TOTAL_THRESHOLD_VOLTS = 420;

/* The lowest pcc threshold is the lowest allowable single cell voltage (in 100 microvolts)*/
const unsigned long PACK_CHARGE_CRIT_LOWEST_CELL_THRESHOLD = 35000;   // Equivalent to 3.5V

const float DEFAULT_INIT_TEMP       = 40.0f;                           // Celsius
const float DEFAULT_INIT_VOLTAGE    = 3.5f;                            // Volts
const float DEFAULT_TEMP_ALPHA      = 0.8f;                            // IIR filter alpha
const float DEFAULT_VOLTAGE_ALPHA   = 0.8f;                            // IIR filter alpha
const uint16_t MAX_PACK_CHARGE      = 48600;                           // Coulombs


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
     * Retrieves the singular instance of the AMSSystem
     */
    static AMSSystem& getInstance()
    {
        static AMSSystem instance;
        return instance;
    }

    /**
     * Delete copy constructor and assignment operator to prevent duplication.
     */
    AMSSystem(const AMSSystem&) = delete;
    AMSSystem& operator=(const AMSSystem&) = delete;

    /**
     * Initialize the heartbeat timer.
     */
    void init(unsigned long curr_micros);

    void update_ams_system(unsigned long curr_millis, BMSData_s &bms_data, VCRSystemData_s &system_data);

private:

    /**
     * Constructor for the AMS Interface
     */
    AMSSystem() :
        _last_heartbeat_time(0),
        _filtered_max_cell_temp(DEFAULT_INIT_TEMP),
        _filtered_min_cell_voltage(DEFAULT_INIT_VOLTAGE),
        _cell_temp_alpha(DEFAULT_TEMP_ALPHA),
        _cell_voltage_alpha(DEFAULT_VOLTAGE_ALPHA)
    {};

    /* AMS last heartbeat time */
    unsigned long _last_heartbeat_time;
    
    float _filtered_max_cell_temp;
    float _filtered_min_cell_voltage;

    /* extra filtered information */
    float _filtered_average_cell_voltage;
    float _filtered_max_cell_voltage;
    float _filtered_average_cell_temp;
    float _filtered_min_cell_temp;

    /* IIR alpha values */
    float _cell_temp_alpha;
    float _cell_voltage_alpha;

    /*Storage of BMS Struct*/
    BMSData_s bms_container;


    /* Check if lowest cell temperature is below threshold */
    bool is_below_pack_charge_critical_low_thresh();

    /* Check if total pack charge is above threshold */
    bool is_below_pack_charge_critical_total_thresh();

    /* IIR filter and return filtered max cell temperature */
    float get_filtered_max_cell_temp();
    /* IIR filter and return filtered min cell voltage */
    float get_filtered_min_cell_voltage();    


    /* IIR filter and return filtered average cell voltage */
    float get_filtered_average_cell_voltage();
    /* IIR filter and return filtered max cell voltage */
    float get_filtered_max_cell_voltage();
    /* IIR filter and return filtered average cell temp */
    float get_filtered_average_cell_temp();
    /* IIR filter and return filtered min cell temp */
    float get_filtered_min_cell_temp();


    //Checkers (return true if everything is good)
    /* Checks if "Total pack voltage is below critical threshold" */
    bool check_voltage();

    /* Check if heartbeat received is within interval allowance */
    bool check_heartbeat(unsigned long curr_micros);

};

#endif /* AMS_SYSTEM_H */