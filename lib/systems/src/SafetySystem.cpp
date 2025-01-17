#include "SafetySystem.h"



void SafetySystem::init()
{    
    _software_is_ok = true;  

    _ams -> set_start_state();
    _watchdog -> set_state_state();


    
}

void SafetySystem::update_software_shutdown(unsigned long curr_millis)
{
    /** 
     * Updates _software_is_ok by updating AMS interface and Watchdog interface. If either
     * of those have problems, then _software_is_ok goes to false. 
    */

   _software_is_ok = true;

   //1. checks whether watchdog state is HIGH (true?) after watchdog is kicked; 
   //2. checks if the last AMS heartbeat was recieved  
   if (!(_watchdog -> get_watchdog_state()) && !(_ams -> heartbeat_recieved(curr_millis))) {
     _software_is_ok = false;
   }

   if (_software_is_ok) {
      _ams -> set_state_ok_high(true);
   } else {
      _ams -> set_state_ok_high(false);
   }

   //kick watchdog every cycle
   _watchdog -> kick_watchdog(curr_millis);


   


}