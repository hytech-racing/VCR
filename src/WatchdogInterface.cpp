#include "WatchdogInterface.h"


/* Pin mode output to watchdog WD */
/*Pin MUST be set with setPin() before calling init()*/
void WatchdogInterface::init(unsigned long curr_millis) 
{
    // Set pin mode        
    pinMode(_pin_watchdog_input_, OUTPUT);

    _watchdog_time = curr_millis;
    set_watchdog_state(HIGH);
}

/* Initial output to watchdog WD */
void WatchdogInterface::set_start_state() 
{

    digitalWrite(_pin_watchdog_input_, HIGH);
    
}

/* Toggle watchdog WD to kick dog */
void WatchdogInterface::kick_watchdog(unsigned long curr_millis) 
{

    if ((curr_millis - _watchdog_time) > WATCHDOG_KICK_INTERVAL) {
        _watchdog_state = !_watchdog_state;
        digitalWrite(_pin_watchdog_input_, _watchdog_state);
        _watchdog_time = curr_millis;
    }

}

/* Get interface status */
bool WatchdogInterface::get_watchdog_state() 
{
    return _watchdog_state;
}

/* Set interface status */
void WatchdogInterface::set_watchdog_state(bool state) 
{
    _watchdog_state = state;
}

/*Set the pin from default (-1) of the Watchdog after calling getInstance() the first time*/
void WatchdogInterface::setPin(int pin)
{
    _pin_watchdog_input_ = pin;
}