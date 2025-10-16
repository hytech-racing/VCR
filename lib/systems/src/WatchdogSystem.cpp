/******************************************************************************
 * @file    WatchdogSystem.cpp
 * @brief   Header for any receive/send to the inverters
 ******************************************************************************/

 /******************************************************************************
 * Includes
 ******************************************************************************/
#include "WatchdogSystem.h"
#include "SharedFirmwareTypes.h"

/******************************************************************************
 * Public Method Definitions
 ******************************************************************************/
/* Returns intended watchdog state */
bool WatchdogSystem::getWatchdogState(unsigned long curr_millis) 
{

    if ((curr_millis - _watchdog_time) > _watchdog_kick_interval) {
        _watchdog_state = !_watchdog_state;
        _watchdog_time = curr_millis;
    }

    return _watchdog_state;

}