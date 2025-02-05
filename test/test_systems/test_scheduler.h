#ifndef SCHEDULER_TEST
#define SCHEDULER_TEST

#include "ht_sched.hpp"
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
    // hal_printf("Initialized function at %d (micros)\n", sysMicros);
    return true;
}

bool run_test_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // hal_printf("Ran function at %d (micros) for the %dth time\n", sysMicros, taskInfo.executions);
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


unsigned int timestamp = (unsigned int) (4294967296UL - 10000UL); // Start 10ms early
unsigned long testMicrosFunction() // Increments by 100 microseconds every function call
{
    return timestamp;
}

int num_runs = 0;
bool run_test_overflow_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    num_runs++;
    hal_printf("Ran task for %uth time\n", num_runs);
    return true;
}
HT_TASK::Task test_overflow_task = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, run_test_overflow_task, 1, 1000UL); // 1,000us is 1kHz

TEST(SchedulerTest, microsFunctionOverflow)
{
    HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();
    scheduler.setTimingFunction(testMicrosFunction);

    scheduler.schedule(test_overflow_task);

    for (int i = 1; i < 50; ++i) { // Run 30 cycles of the overflow task
        for (int j = 0; j < 10; ++j) {
            scheduler.run();
            timestamp += 100; // Increment timestamp by 100ms every time
        }
        hal_printf("Timestamp is %u\n", timestamp);
        ASSERT_EQ(num_runs, i);
    }

}

#endif /* SCHEDULER_TEST */