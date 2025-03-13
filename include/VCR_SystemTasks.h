#ifndef VCR_SYSTEMTASKS_H
#define VCR_SYSTEMTASKS_H

#include "VCFInterface.h"

#include "SharedFirmwareTypes.h"

#include "DrivetrainSystem.h"
#include "VCRCANInterfaceImpl.h"
#include "VehicleStateMachine.h"
#include "etl/delegate.h"
#include "shared_types.h"
#include "SystemTimeInterface.h"

#include <iostream>
#include <chrono>
#include "ht_sched.hpp"
#include "ht_task.hpp"


/**
 * Generally, our main "loop" (not actually a loop) will run as fast as possible and will go through
 * three stages.
 * 
 * First, it will sample all asynchronous data (i.e. CAN and Ethernet), since those will
 * asynchronously arrive and be added to a buffer. This ensures that we are processing this data as
 * quickly as possible, instead of simply at the next "tick" at 50/100/1000Hz.
 * 
 * Second, it will evaluate all systems based on this udpated data.
 * 
 * Third, it will tick the state machine using this updated data.
 * 
 * Asynchronously, we will be doing all *sending* at a fixed rate (like sending CAN/Ethernet messages).
 */

 /**
  * Bundle of all asynchronous interfaces (CAN and Ethernet, mainly). This can be passed into the
  * sample_async_data function as a single bundle.
  */
struct VCRAsynchronousInterfaces {
    explicit VCRAsynchronousInterfaces(CANInterfaces &can_ints) : can_interfaces(can_ints) {}
    VCRAsynchronousInterfaces() = delete;

    CANInterfaces &can_interfaces;
};

/**
 * Reads all asynchronously-arriving data (CAN and Ethernet) from their respective buffers and updates their
 * timestamped values in VCRInterfaceData_s (returns the new struct). Takes in a bundle of all the asynchronous
 * interfaces to be ticked. 
 */
VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
    VCRAsynchronousInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data);

/** 
 * Ticks all hardware-abstracted systems based on the current VCRInterfaceData passed in. Takes in a bundle
 * of references to all the relevant systems. Systems that will be handled here include anything that we want
 * to be immediately handled every loop but does not depend on the state of the car or that the state machine 
 * does not depend upon for determing the state.
 */
VCRSystemData_s evaluate_async_systems(const VCRInterfaceData_s &interface_data);


/**
 * Ticks the state machine based on the new interface and system data.
 */
VehicleState_e evaluate_state_machine(const VCRSystemData_s &system_data,
                                  const VCRInterfaceData_s &interface_data,
                                  VehicleStateMachine &state_machine);


void big_task(etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
              VCRAsynchronousInterfaces &interface_ref_container,
              VehicleStateMachine &state_machine, const VCRInterfaceData_s &cur_vcr_int_data);


HT_TASK::Task schedMon = HT_TASK::Task(
    std::bind(&HT_SCHED::Scheduler::initSchedMon, std::ref(scheduler), std::placeholders::_1, std::placeholders::_2), 
    std::bind(&HT_SCHED::Scheduler::schedMon, std::ref(scheduler), std::placeholders::_1, std::placeholders::_2), 
    100000UL,
    0
);

auto start_time = std::chrono::high_resolution_clock::now();

unsigned long stdMicros()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
    return static_cast<unsigned long>(elapsed);
}

bool task1F(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    std::cout << "task1 exec " << taskInfo.executions << "\n";
    return true;
}

HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

HT_TASK::Task task1 = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, task1F, 2, 20000UL); // 20000us is 50hz

int main(void)
{
    scheduler.setTimingFunction(stdMicros);
    
    // all tasks
    scheduler.schedule(task1);
    scheduler.schedule(schedMon);

    while (std::chrono::high_resolution_clock::now() - start_time < std::chrono::seconds(1))
    {
        scheduler.run();
    }

    return 0;
}

#endif // __VCR_SYSTEMTASKS_H__