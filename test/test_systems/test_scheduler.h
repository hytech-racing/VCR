#ifndef SCHEDULER_TEST
#define SCHEDULER_TEST

#include "ht_sched.hpp"
#include "VCR_Tasks.h"
#include "Logger.h"
#include <chrono>

auto start_time = std::chrono::high_resolution_clock::now();
unsigned long stdMicros()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
    return static_cast<unsigned long>(elapsed);
}



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



TEST(SchedulerTest, basic_functionality)
{
    HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();
    scheduler.setTimingFunction(stdMicros);

    scheduler.schedule(test_task);

    auto test_start_time = std::chrono::high_resolution_clock::now();
    while (std::chrono::high_resolution_clock::now() - test_start_time < std::chrono::seconds(2)) // run for 2sec
    {
        scheduler.run();
    }
    
}

#endif /* SCHEDULER_TEST */