#ifndef VCR_TASKS
#define VCR_TASKS

/* From shared_firmware_types library */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "WatchdogSystem.h"
#include "VCR_Constants.h"
#include "VehicleStateMachine.h"
#include "VCR_Globals.h"
#include "Buzzer.h"

/**
 * The "tick state machine" task will simply call the state machine's tick function with the current
 * timestamp in micros. No init function is necessary. The tick function makes use of the other systems'
 * singleton classes to minimize the need for passing instances around.
 */
void run_tick_state_machine_task();
    
/**
 * The read_adc0 task will command adc0 to sample all eight channels, convert the outputs, and
 * store them in structs defined in shared_firmware_types. This function relies on adc_0 being
 * defined in VCRGlobals.h.
 */
bool init_read_adc0_task();
void run_read_adc0_task();

/**
 * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You may edit this
 * manually to add sensors.
 * 
 * The read_adc1 task will command adc1 to sample all eight channels, convert the outputs, and
 * store them in a struct defined in shared_firmware_types. This function relies on adc_1 being
 * defined in VCRGlobals.h.
 */
bool init_read_adc1_task();
void run_read_adc1_task();

/**
 * This task will update the buzzer_is_active voidean in the VCRSystemData struct by calling the
 * update function of the buzzer controller.
 */
void run_update_buzzer_controller_task();



/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin
 */
void run_kick_watchdog();

void handle_send_suspension_CAN_data();

#endif /* VCR_TASKS */