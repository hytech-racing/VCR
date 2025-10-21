#ifndef VCR_INTERFACETASKS
#define VCR_INTERFACETASKS

/* From shared_firmware_types library */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "WatchdogSystem.h"
#include "VCR_Constants.h"
#include "VehicleStateMachine.h"
#include "VCR_Globals.h"
#include <ht_task.hpp>
#include "IOExpander.h"


/**
 * The read_adc0 task will command adc0 to sample all eight channels, convert the outputs, and
 * store them in structs defined in shared_firmware_types. This function relies on adc_0 being
 * defined in VCRGlobals.h.
 */
HT_TASK::TaskResponse init_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse init_adc_bundle();

/**
 * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You may edit this
 * manually to add sensors.
 * 
 * The read_adc1 task will command adc1 to sample all eight channels, convert the outputs, and
 * store them in a struct defined in shared_firmware_types. This function relies on adc_1 being
 * defined in VCRGlobals.h.
 */
HT_TASK::TaskResponse init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

HT_TASK::TaskResponse run_sample_flowmeter(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task will update the buzzer_is_active voidean in the VCRSystemData struct by calling the
 * update function of the buzzer controller.
 */
HT_TASK::TaskResponse run_update_buzzer_controller_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
HT_TASK::TaskResponse init_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Uses the i2c IOExpander to sense the shutdown line.
 */
HT_TASK::TaskResponse init_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse read_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Handles sending of suspension CAN message data (load cell and shock pot data)
 */
HT_TASK::TaskResponse enqueue_suspension_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Handles sending of coolant temperature data
*/
HT_TASK::TaskResponse enqueue_coolant_temp_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Enqueues all inverter CAN data. This will add all inverter data to the CAN queue, and then
 * the send_all_data task will empty the queue.
 */
HT_TASK::TaskResponse enqueue_inverter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Enqueues all data needed for dashboard. 
 */
HT_TASK::TaskResponse enqueue_dashboard_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Sends all CAN data from the TX buffers of both telem and inverter CAN lines.
 */
HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * This task will tick the AMS system and will update the software shutdown if necessary.
 */
HT_TASK::TaskResponse init_acu_heartbeat(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse update_acu_heartbeat(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task reads the received pedals data and determines whether to turn on the brake light or not.
 */
HT_TASK::TaskResponse init_update_brakelight_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse run_update_brakelight_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Task for sending all ethernet data
 */
HT_TASK::TaskResponse handle_send_VCR_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized VCR)

#endif /* VCR_INTERFACETASKS */
