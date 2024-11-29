#ifndef VCR_TASKS
#define VCR_TASKS

/**
 * This file includes all of the Task definitions required for the Task Scheduler. See the Task
 * Scheduler GitHub (https://github.com/hytech-racing/HT_SCHED) and the relevant BookStack page
 * (https://wiki.hytechracing.org/books/ht09-design/page/ht-task-scheduler) for usage directions.
 */

#include "ht_sched.hpp"
#include "Logger.h"

bool test_init_function(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    hal_printf("Initialized function at %d (micros)\n", sysMicros);
    return true;
}

bool test_run_function(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    hal_printf("Ran function at %d (micros) for the %dth time\n", sysMicros, taskInfo.executions);
    return true;
}

HT_TASK::Task test_task = HT_TASK::Task(test_init_function, test_run_function, 1, 100000UL); // 100,000us is 10hz

#endif /* VCR_TASKS */