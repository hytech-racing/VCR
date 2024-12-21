#ifndef __WATCHDOG_INTERFACE_H__
#define __WATCHDOG_INTERFACE_H__

#include <Arduino.h>

const unsigned long WATCHDOG_KICK_INTERVAL = 10;    // milliseconds

class WatchdogInterface
{
private:

    /*Declare private instance*/
    static WatchdogInterface* instance;

    /*Private constructor*/
    WatchdogInterface(int wd_input_pin): pin_watchdog_input_(wd_input_pin){};

    /* Watchdog last kicked time */
    unsigned long watchdog_time;

    /* Watchdog output state */
    bool watchdog_state;

    /* Hardware interface pins */
    int pin_watchdog_input_;
    
public:

    /*Getter to return the instance and make a new one if it hasn't been made yet*/
    static WatchdogInterface* getInstance(int wd_input_pin){
        if (instance == nullptr){
            instance = new WatchdogInterface(wd_input_pin);
        }
        return instance;
    }


    /*Prevent copying*/
    WatchdogInterface(const WatchdogInterface&) = delete;
    /*Prevent assignment*/
    WatchdogInterface& operator=(const WatchdogInterface&) = delete;


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
