#include "WatchdogSystem.h"
#include "SharedFirmwareTypes.h"

/* Returns intended watchdog state */
bool WatchdogSystem::get_watchdog_state(unsigned long curr_millis) 
{

    if ((curr_millis - _watchdog_time) > WATCHDOG_KICK_INTERVAL) {
        _watchdog_state = !_watchdog_state;
        _watchdog_time = curr_millis;
    }

    return _watchdog_state;

}