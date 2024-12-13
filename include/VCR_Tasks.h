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

/* From shared-systems-lib */
#include "Logger.h"

/* From shared_firmware_types library */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCR_Constants.h"
#include "ADC_interface.h"

/**
 * This "Test" function is purely for validation of the HT_SCHED dependency. This is intended to be removed when
 * further development occurs.
 */
bool init_test_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    hal_printf("Initialized function at %d (micros)\n", sysMicros);
    return true;
}

bool run_test_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    hal_printf("Ran function at %d (micros) for the %dth time\n", sysMicros, taskInfo.executions);
    return true;
}

HT_TASK::Task test_task = HT_TASK::Task(init_test_task, run_test_task, 1, 100000UL); // 100,000us is 10hz




/**
 * The read_adc0 task will command adc0 to sample all eight channels, convert the outputs, and
 * store them in structs defined in shared_firmware_types. This function relies on ADC_0 and
 * ADC_1 being defined in ADC_interface.h.
 */
bool init_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    ADC_0.setChannelScaleAndOffset(GLV_SENSE_CHANNEL, GLV_SENSE_SCALE, GLV_SENSE_OFFSET);
    ADC_0.setChannelScaleAndOffset(CURRENT_SENSE_CHANNEL, CURRENT_SENSE_SCALE, CURRENT_SENSE_OFFSET);
    ADC_0.setChannelScaleAndOffset(REFERENCE_SENSE_CHANNEL, REFERENCE_SENSE_SCALE, REFERENCE_SENSE_OFFSET);
    ADC_0.setChannelScaleAndOffset(RL_LOADCELL_CHANNEL, RL_LOADCELL_SCALE, RL_LOADCELL_OFFSET);
    ADC_0.setChannelScaleAndOffset(RR_LOADCELL_CHANNEL, RL_LOADCELL_SCALE, RL_LOADCELL_OFFSET);
    ADC_0.setChannelScaleAndOffset(RL_SUS_POT_CHANNEL, RL_SUS_POT_SCALE, RL_SUS_POT_OFFSET);
    ADC_0.setChannelScaleAndOffset(RR_SUS_POT_CHANNEL, RR_SUS_POT_SCALE, RR_SUS_POT_OFFSET);

    hal_printf("Initialized ADC0 at %d (micros)\n", sysMicros);

    return true;
}

bool run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    ADC_0.sample(); // Samples all eight channels.
    ADC_0.convert(); // Converts all eight channels.

    hal_printf("%4d, %8.2f\n", ADC_0.data.conversions[0].raw, ADC_0.data.conversions[0].conversion); // Prints channel 0.
    return true;
}

HT_TASK::Task read_adc0_task = HT_TASK::Task(init_read_adc0_task, run_read_adc0_task, 10, 1000UL); // 1000us is 1kHz

#endif /* VCR_TASKS */