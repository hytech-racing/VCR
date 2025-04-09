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
bool init_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool init_adc_bundle();

/**
 * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You may edit this
 * manually to add sensors.
 * 
 * The read_adc1 task will command adc1 to sample all eight channels, convert the outputs, and
 * store them in a struct defined in shared_firmware_types. This function relies on adc_1 being
 * defined in VCRGlobals.h.
 */
bool init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task will update the buzzer_is_active voidean in the VCRSystemData struct by calling the
 * update function of the buzzer controller.
 */
bool run_update_buzzer_controller_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
bool init_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Uses the i2c IOExpander to sense the shutdown line.
 */
bool init_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool read_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Handles sending of suspension CAN message data (load cell and shock pot data)
 */
bool enqueue_suspension_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Enqueues all inverter CAN data. This will add all inverter data to the CAN queue, and then
 * the send_all_data task will empty the queue.
 */
bool enqueue_inverter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * handles sending of all data on all interfaces
 */
bool handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * This task will tick the AMS system and will update the software shutdown if necessary.
 */
bool init_ams_system_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_ams_system_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task reads the received pedals data and determines whether to turn on the brake light or not.
 */
bool init_update_brakelight_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_update_brakelight_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

// task for sending all ethernet data
bool handle_send_VCR_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized VCR)
#endif /* VCR_INTERFACETASKS */
