#include "gtest/gtest.h"
#include "ht_sched.hpp"

uint32_t start = UINT32_MAX - 4900;
uint32_t curr_time = start;

unsigned long fakeMicros()
{
    return curr_time;
}

bool task1Run(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    std::cout << "task1 ran at " << sysMicros << "\n";
    return true;
}
HT_TASK::Task task1 = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, task1Run, 1, 1000UL); // 20000us is 50hz

TEST(TestSchedulerOverflow, justBeforeOverflow) {
    HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();
    scheduler.setTimingFunction(fakeMicros);
    scheduler.schedule(task1);
    curr_time = start;

    for (int i = 0; i < 100; ++i) {
        std::cout << "scheduler ticked at " << curr_time << " micros\n";
        scheduler.run();
        curr_time += 99;
    }

    ASSERT_EQ(curr_time, curr_time + 100);

}