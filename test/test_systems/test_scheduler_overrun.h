#ifndef SCHEDULER_TEST
#define SCHEDULER_TEST

// #define _TASK_MICRO_RES // NOLINT
#define _TASK_DEFINE_MILLIS
#define _TASK_TICKLESS
#include <TScheduler.hpp>
#include "Logger.h"
#include "SystemTimeInterface.h"



bool init_test_task()
{
    // hal_printf("Initialized function at %d (micros)\n", sysMicros);
    return true;
}

void run_test_task()
{
    
}

unsigned int timestamp; // Start 10ms early

extern "C" {
unsigned long micros(void)
{
    return timestamp;
}
unsigned long millis(void)
{
    return timestamp;
}
}

int num_runs = 0;
int num_runs2 = 0;
void run_test_overflow_task()
{
    num_runs++;
    hal_printf("Ran task for %uth time\n", num_runs);
}

void run_test_overflow_task2()
{
    num_runs2++;
    hal_printf("Ran task for %uth time\n", num_runs);
}


TEST(SchedulerTest, microsFunctionOverflow)
{
    timestamp = (unsigned long int) (4294967296UL - 1000000UL);
    hal_printf("creating scheduler\n");
    TsScheduler task_scheduler;
    TsTask test_task(100000, TASK_FOREVER, &run_test_overflow_task, &task_scheduler, false);
    TsTask test_task2(100000, TASK_FOREVER, &run_test_overflow_task2, &task_scheduler, false);
    hal_printf("Initialized\n");
    // expected
    test_task.enable();
    test_task2.enable();
    num_runs = 0;
    num_runs2 = 0;
    for (int i = 0; i < 50; ++i) { // Run 30 cycles of the overflow task
        int j = 1;
        hal_printf("Timestamp before is %u\n", timestamp);
        for (; j <= 100; ++j) { // execute the task 10 times
            
            task_scheduler.execute();
            hal_printf("one execute\n");
            hal_printf("Timestamp is %u\n", timestamp);
            hal_printf("%i\n", task_scheduler.timeUntilNextIteration(test_task));
            timestamp += 10000; // fast-forward 50ms every time
            sys_time::set_millis(timestamp);
        }
        
        ASSERT_EQ(num_runs, j/10); // 100000 / 10000 = ten inner loop cycles to one run of the task
        ASSERT_EQ(num_runs2, j/10); // 100000 / 10000 = ten inner loop cycles to one run of the task
        num_runs = 0;
        num_runs2 = 0;
    }

}

#endif /* SCHEDULER_TEST */