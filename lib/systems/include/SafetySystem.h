#ifndef SAFETY_SYSTEM
#define SAFETY_SYSTEM
#include "WatchdogSystem.h"
#include "AMSInterface.h"
#include "DrivetrainSystem.h"

class SafetySystem
{
public:
    static SafetySystem& getInstance()
    {
        static SafetySystem instance;
        return instance;
    }
 
    /**
     * Initializes AMS interface, Watchdog interface, and sets _software_is_ok to true.
     */
    void init();

    /**
     * Updates _software_is_ok by updating AMS interface and Watchdog interface. If either
     * of those have problems, then _software_is_ok goes to false.
     */
    bool update_software_shutdown(unsigned long curr_millis);

    bool get_software_is_ok() {return _software_is_ok;}

    /**
     * Mandatory for singleton classes to delete the copy constructor.
     */
    SafetySystem(const SafetySystem&) = delete;

    /**
     * Mandatory for singleton classes to prevent copying through the assignment operator.
     */
    SafetySystem& operator=(const SafetySystem&) = delete;


private:
    SafetySystem():
    _watchdog{WatchdogSystem::getInstance()},
    _ams{AMSInterface::getInstance()},
    _software_is_ok(false) {};

    /* Software ok status */
    bool _software_is_ok;

};

#endif /* SAFETY_SYSTEM */
