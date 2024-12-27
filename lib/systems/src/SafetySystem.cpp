#include "SafetySystem.h"
//include AMS and Watchdog Interfaces

void SafetySystem::init()
{    
    _software_is_ok = true; 

    // AMS and Watchdog objects that are each an instance of their respective interfaces 
    // will be defined when AMSInterface and WatchdogInterface files are included
    ams = AMSInterface.getInstance();
    watchdog = WatchdogInterface.getInstance();
}

void SafetySystem::update_software_shutdown(unsigned long curr_millis)
{
    /** 
     * Updates _software_is_ok by updating AMS interface and Watchdog interface. If either
     * of those have problems, then _software_is_ok goes to false. - need AMS and Watchdog files to do this
    */

   _software_is_ok = true;

   //if the ams doesn't send a signal back within the curr_millis, set _software_is_ok to false

   //kick the watchdog by curr_millis - if it's greater than the time it was last kicked, set software_is_ok to false??
   //otherwise, look at watchdog functions and see if there's something like it sends back a false signal after being kicked for curr_millis

}