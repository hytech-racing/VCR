#ifndef __WATCHDOG_INTERFACE_H__
#define __WATCHDOG_INTERFACE_H__


/**
 * This class controls the watchdog timer This class is implemented as a singleton to ensure only one instance of the watchdog timer is active in the system.
 * The WatchdogInterface provides functionality to initialize, monitor, and "kick" the watchdog to prevent system resets.
 * 
 * NOTE:  The WatchdogInterface ensures system responsiveness by requiring periodic "kicks" using the `kick_watchdog()` method. 
 * During initialization, the watchdog pin is configured, and its state is set. If the timer is not reset within the 
 * `WATCHDOG_KICK_INTERVAL`, it assumes the system is unresponsive and may trigger a reset. The singleton design ensures 
 * only one instance manages the watchdog, which will help mantain consistency and control.
 * 
 * IMPORTANT - must call setPin() before init()
 */


#include <Arduino.h>

const unsigned long WATCHDOG_KICK_INTERVAL = 10;    // milliseconds

class WatchdogInterface
{
private:

    /*Private constructor*/
    WatchdogInterface(): _pin_watchdog_input_(-1){};

    /* Watchdog last kicked time */
    unsigned long _watchdog_time;

    /* Watchdog output state */
    bool _watchdog_state;

    /* Hardware interface pins */
    int _pin_watchdog_input_;
    
public:

    /*Getter to return the instance and make a new one if it hasn't been made yet*/
    static WatchdogInterface& getInstance() 
    {
        static WatchdogInterface instance;
        return instance;
    }

    /*Use to set the pin from default (-1) of the Watchdog after calling getInstance() the first time*/
    void setPin(int Pin);


    /*Prevent copying*/
    WatchdogInterface(const WatchdogInterface&) = delete;

    /*Prevent Assignment*/
    WatchdogInterface& operator= (const WatchdogInterface&) = delete;


    /* Initialize interface pin mode */
    void init(unsigned long curr_millis);

    /* Write to Main ECU */
    // Initialize output value
    void set_start_state();

    /* Kick watchdog */
    void kick_watchdog(unsigned long curr_millis);

    /* Getters */
    bool get_watchdog_state();
    /* Setters */
    void set_watchdog_state(bool state);

};

#endif /* __WATCHDOG_INTERFACE_H__ */
