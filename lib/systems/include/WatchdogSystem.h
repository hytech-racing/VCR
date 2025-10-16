/******************************************************************************
 * @file    WatchdogSystem.h
 * @brief   Header for the Watchdog System
 ******************************************************************************/
#ifndef WATCHDOG_SYSTEM_H
#define WATCHDOG_SYSTEM_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <etl/singleton.h>

/******************************************************************************
 * Public Class Definitions
 ******************************************************************************/
/**
 * @class This class controls the boolean _watchdog_state, but does not directly control the
 * watchdog. WatchdogSystem provides functionality to initialize, monitor, and "kick" the watchdog
 * to prevent system resets
 *
 * NOTE:  To ensure system responsiveness, WatchdogSystem requires periodic updates by calling the
 * `get_watchdog_state()` method. This toggles the _watchdog_state (if the interval has passed) and
 * returns the new state, which must then be sent to the watchdog.
 */
class WatchdogSystem {
    public:

        /**
         * Constructs the watchdog system with a specified kick interval.
         * @param kick_interval_ms The interval in milliseconds at which the watchdog state should toggle.
         */
        WatchdogSystem(const unsigned long kick_interval_ms = 10UL)
            : _watchdog_time(0), _watchdog_state(false), _watchdog_kick_interval(kick_interval_ms) {};

        /**
         * Returns the current watchdog state, toggling it if the kick interval has passed.
         * @param curr_millis The current time in milliseconds.
         * @return The current watchdog state (true or false).
         */
        bool getWatchdogState(unsigned long curr_millis);

    private:

        /* Watchdog last kicked time */
        unsigned long _watchdog_time;
        bool _watchdog_state;
        unsigned long _watchdog_kick_interval;
        
};

using WatchdogInstance = etl::singleton<WatchdogSystem>;

#endif /* WATCHDOG_SYSTEM_H */
