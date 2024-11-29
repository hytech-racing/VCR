#ifndef SCHEDULER_TEST
#define SCHEDULER_TEST

#include "ht_sched.hpp"
#include "VCR_Tasks.h"

auto start_time = std::chrono::high_resolution_clock::now();
unsigned long stdMicros()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
    return static_cast<unsigned long>(elapsed);
}

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