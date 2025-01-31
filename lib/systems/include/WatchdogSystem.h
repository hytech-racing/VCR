#ifndef __WATCHDOG_INTERFACE_H__
#define __WATCHDOG_INTERFACE_H__


/**
 * This class controls the boolean _watchdog_state, but does not directly control the watchdog.
 * WatchdogSystem provides functionality to initialize, monitor, and "kick" the watchdog to prevent system resets.
 * 
 * NOTE:  To ensure system responsiveness, WatchdogSystem requires periodic updates by calling the `get_watchdog_state()` method. 
 * This toggles the _watchdog_state (if the interval has passed) and returns the new state, which must then be sent to the watchdog. 
 * If this is not done within the `WATCHDOG_KICK_INTERVAL`, it assumes the system is unresponsive and may trigger a reset. The singleton design ensures 
 * only one instance manages the watchdog, which will help mantain consistency and control.
 * 
 */

const unsigned long WATCHDOG_KICK_INTERVAL = 10;    // milliseconds

class WatchdogSystem
{
private:

    /*Private constructor*/
    WatchdogSystem() : _watchdog_time(0), _watchdog_state(false) {};

    /* Watchdog last kicked time */
    unsigned long _watchdog_time;

    /* Watchdog output state */
    bool _watchdog_state;
    
public:

    /*Getter to return the instance and make a new one if it hasn't been made yet*/
    static WatchdogSystem& getInstance() 
    {
        static WatchdogSystem instance;
        return instance;
    }

    /*Prevent copying*/
    WatchdogSystem(const WatchdogSystem&) = delete;

    /*Prevent Assignment*/
    WatchdogSystem& operator= (const WatchdogSystem&) = delete;

    /* Get/update watchdog state */
    bool get_watchdog_state(unsigned long curr_millis);

};

#endif /* __WATCHDOG_INTERFACE_H__ */
