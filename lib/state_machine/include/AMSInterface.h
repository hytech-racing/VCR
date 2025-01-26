#ifndef __AMSINTERFACE_H__
#define __AMSINTERFACE_H__

#include <stdint.h>

/* Heartbeat Interval is the allowable amount of time between BMS status messages before car delatches */
const unsigned long HEARTBEAT_INTERVAL                      = 2000;   // milliseconds
/* The total pcc threshold is the lowest allowable voltage of the entire pack (in Volts)*/
const unsigned long PACK_CHARGE_CRIT_TOTAL_THRESHOLD        = 420;
/* The lowest pcc threshold is the lowest allowable single cell voltage (in 100 microvolts)*/
const unsigned long PACK_CHARGE_CRIT_LOWEST_CELL_THRESHOLD  = 35000; //equivalent to 3.5V

const float DEFAULT_INIT_TEMP       = 40.0;
const float DEFAULT_INIT_VOLTAGE    = 3.5;
const float DEFAULT_TEMP_ALPHA      = 0.8;
const float DEFAULT_VOLTAGE_ALPHA   = 0.8;
const uint16_t MAX_PACK_CHARGE      = 48600;
const unsigned long DEFAULT_INITIALIZATION_WAIT_INTERVAL = 5000;
//SW_OK_PIN should be initialized to but for now, using -1.
const int DEFAULT_SW_OK_PIN_ = 37; //number from scehmatic for VCR


/// @brief this class is for interfacing with the AMS (accumulator management system) 
class AMSInterface
{
public:
    /* Method to get instance of object */
    static AMSInterface& getInstance()
    {
        static AMSInterface instance;
        return instance;
    }

    AMSInterface(const AMSInterface&) = delete;
    AMSInterface& operator=(const AMSInterface&) = delete;

    /* Initialize the heartbeat timer */
    void init(unsigned long curr_micros);//unsigned long micros

    /* Init software OK pin by setting high*/
    void set_start_state();

    /* Check if the last heartbeat arrived within allowable interval */
    bool heartbeat_received(unsigned long curr_micros);//micros

    /* Check if either lowest cell or total pack is below threshold*/
    bool pack_charge_is_critical(); 

    //SETTERS//
    /* set the last heartbeat to the current millis time */
    void set_heartbeat(unsigned long curr_millis);

    //GETTERS//
    /* IIR filter and return filtered max cell temperature */
    float get_filtered_max_cell_temp();
    /* IIR filter and return filtered min cell voltage */
    float get_filtered_min_cell_voltage();
    /*gets the derate factor for acc system*/
    float get_acc_derate_factor();

    /*Updates Acc_derate_factor*/
    void calculate_acc_derate_factor();
    

private:

    // Private constructor for the singleton pattern
    /*!
        Constructor for the AMS Interface
        @param sw_ok_pin The software ok pin number.
        This pin is connected to the shutdown line and will go low if the AMS times out
    */
    AMSInterface(int sw_ok_pin, float init_temp, float init_volt, float temp_alpha, float volt_alpha):        
        _pin_software_ok(sw_ok_pin),
        filtered_max_cell_temp(init_temp),
        filtered_min_cell_voltage(init_volt),
        cell_temp_alpha(temp_alpha),
        cell_voltage_alpha(volt_alpha) {};

    /* Overloaded constructor that only takes in software OK pin and uses default voltages and temp*/
    AMSInterface():
        AMSInterface(DEFAULT_SW_OK_PIN_, DEFAULT_INIT_TEMP, DEFAULT_INIT_VOLTAGE, DEFAULT_TEMP_ALPHA, DEFAULT_VOLTAGE_ALPHA) {};

    /* software OK pin */
    int _pin_software_ok;

    /* AMS last heartbeat time */
    unsigned long last_heartbeat_time_;
    
    /* IIR filter parameters */
    float bms_high_temp;
    float bms_low_voltage;
    float filtered_max_cell_temp;
    float filtered_min_cell_voltage;
    float cell_temp_alpha;
    float cell_voltage_alpha;

    float acc_derate_factor;

    // Check if lowest cell temperature is below threshold
    bool is_below_pack_charge_critical_low_thresh();
    // Check if total pack charge is above threshold
    bool is_below_pack_charge_critical_total_thresh();

};

#endif /* __AMSINTERFACE_H__ */

//chage all private variables to be preceded
//look at linter from main branch
//pio check -e /teensy41 in the platformIO new terminal