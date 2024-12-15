/**
 * This file includes all of the Task definitions required for the Task Scheduler. See the Task
 * Scheduler GitHub (https://github.com/hytech-racing/HT_SCHED) and the relevant BookStack page
 * (https://wiki.hytechracing.org/books/ht09-design/page/ht-task-scheduler) for usage directions.
 * 
 * Generally, defining a task takes three steps:
 * 1) Define the "init" function. Name this function init_<taskname>. This init funciton's inputs
 *    MUST be the same for all init functions, taking in sysMicros and a taskInfo reference.
 * 2) Define the "run" function. Name this function run_<taskname>. Similar to the init function,
 *    this function's inputs MUST be sysMicros and taskInfo.
 * 3) Define the function itself. This requires using the HT_TASK::Task constructor and passing
 *    in your init function, your run function, a priority level, and a loop interval (in micros).
 * 4) Add the function to your scheduler.
 * 
 */

/* -------------------------------------------------- */
/*                   TASK PRIORITIES                  */
/* -------------------------------------------------- */
/*
 * (1-10) - Car-critical functions (watchdog, shutdown circuit)
 * (11-100) - Performance-critical functions (reading pedals, interfacing w/ inverters, etc)
 * (100+) - Telemetry, logging functions (could be idle functions, too)
 */

#ifndef VCR_TASKS
#define VCR_TASKS

/* From HT_SCHED library */
#include "ht_sched.hpp"

/* From shared_firmware_types library */
#include "SharedFirmwareTypes.h"

<<<<<<< HEAD
=======
/* Local includes */
#include "VCR_Constants.h"
#include "VCR_Globals.h"
#include "Buzzer.h"

>>>>>>> cb1b012 (Switched buzzer reference to use singleton pattern instead of global instance)
/**
 * This "Test" function is purely for validation of the HT_SCHED dependency. This is intended to be removed when
 * further development occurs.
 */
bool init_test_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_test_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
extern HT_TASK::Task test_task;



/**
 * The read_adc0 task will command adc0 to sample all eight channels, convert the outputs, and
 * store them in structs defined in shared_firmware_types. This function relies on adc_0 being
 * defined in ADC_interface.h.
 */
bool init_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
extern HT_TASK::Task read_adc0_task;



/**
 * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You may edit this
 * manually to add sensors.
 * 
 * The read_adc1 task will command adc0 to sample all eight channels, convert the outputs, and
 * store them in a struct defined in shared_firmware_types. This function relies on adc_1 being
 * defined in ADC_interface.h.
 */
<<<<<<< HEAD
bool init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
extern HT_TASK::Task read_adc1_task;
=======
bool init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    // Initialize all eight channels to scale = 1, offset = 0
    adc_1.setChannelScaleAndOffset(0, 1, 0);
    adc_1.setChannelScaleAndOffset(1, 1, 0);
    adc_1.setChannelScaleAndOffset(2, 1, 0);
    adc_1.setChannelScaleAndOffset(3, 1, 0);
    adc_1.setChannelScaleAndOffset(4, 1, 0);
    adc_1.setChannelScaleAndOffset(5, 1, 0);
    adc_1.setChannelScaleAndOffset(6, 1, 0);
    adc_1.setChannelScaleAndOffset(7, 1, 0);

    hal_printf("Initialized ADC0 at %d (micros)\n", sysMicros);

    return true;
}

bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    adc_1.sample(); // Samples all eight channels.
    adc_1.convert(); // Converts all eight channels.

    return true;
}

HT_TASK::Task read_adc1_task = HT_TASK::Task(init_read_adc1_task, run_read_adc1_task, 100, 40000UL); // 20000us is 25Hz



/**
 * Updates the BuzzerController system. No initialization necessary.
 */
bool run_update_buzzer_controller_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    system_data.buzzer_is_active = BuzzerController::getInstance().buzzer_is_active(sysMicros / 1000); // pass in sysMillis into buzzer_is_active check
}

HT_TASK::Task update_buzzer_controller_task = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, run_update_buzzer_controller_task, 5, 1000UL); // 1000us is 1kHz


>>>>>>> cb1b012 (Switched buzzer reference to use singleton pattern instead of global instance)

#endif /* VCR_TASKS */