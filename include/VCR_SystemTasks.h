#ifndef VCR_SYSTEMTASKS_H
#define VCR_SYSTEMTASKS_H

#include "VCFInterface.h"

#include "SharedFirmwareTypes.h"

#include "DrivetrainSystem.h"
#include "VCRCANInterfaceImpl.h"
#include "VehicleStateMachine.h"
#include "etl/delegate.h"
#include "shared_types.h"

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
struct VCRInterfaces {
    explicit VCRInterfaces(CANInterfaces &can_ints) : can_interfaces(can_ints) {}
    VCRInterfaces() = delete;

    CANInterfaces &can_interfaces;
};

/**
 * Bundle of all systems that must be ticked during evaluate_systems(). Can be passed in as a single argument.
 */
struct VCRSystems {};

/**
 * Reads all asynchronously-arriving data (CAN and Ethernet) from their respective buffers and updates their
 * timestamped values in VCRInterfaceData_s (returns the new struct). Takes in a bundle of all the asynchronous
 * interfaces to be ticked.
 */
VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
    VCRInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data);

/** 
 * Ticks all hardware-abstracted systems based on the current VCRInterfaceData passed in. Takes in a bundle
 * of references to all the relevant systems.
 */
VCRSystemData_s evaluate_systems(const VCRInterfaceData_s &interface_data,
                                 VCRSystems &systems_ref_container);

/**
 * Ticks the state machine based on the new interface and system data.
 */
CarState_e evaluate_state_machine(const VCRSystemData_s &system_data,
                                  const VCRInterfaceData_s &interface_data,
                                  VehicleStateMachine &state_machine);
void update_interfaces(const VCRData_s &system_data, const VCRInterfaceData_s &interface_data);

#endif // __VCR_SYSTEMTASKS_H__
