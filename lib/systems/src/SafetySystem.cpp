#include "SafetySystem.h"



void SafetySystem::init()
{    
    _software_is_ok = true;  

    _ams -> set_start_state();
    
}

bool SafetySystem::update_software_shutdown(unsigned long curr_millis)
{
    /** 
     * Updates _software_is_ok by updating AMS interface and Watchdog interface. If either
     * of those have problems, then _software_is_ok goes to false. 
    */

   _software_is_ok = true;

   //1. checks whether watchdog state is HIGH (true?) after watchdog is kicked; 
   //2. checks if the last AMS heartbeat was recieved  
   if (!(_ams -> heartbeat_received(curr_millis))) {
     _software_is_ok = false;
   }

   return _software_is_ok;

}