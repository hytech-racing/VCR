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
CarState_e evaluate_state_machine(const VCRSystemData_s &system_data,
                                  const VCRInterfaceData_s &interface_data,
                                  VehicleStateMachine &state_machine);

<<<<<<< HEAD
};

// outlining the main process that that is for handling the main systems 
// that need interface data and produce outputs that need to get sent

InterfaceData_s sample_async_data(VCRInterfaces& interface_ref_container);
VCRData_s evaluate_systems(const InterfaceData_s &interface_data, VCRSystems &systems_ref_container);
VehicleState_e evaluate_state_machine(VehicleStateMachine& state_machine);
void update_interfaces(const VCRData_s& system_data, const InterfaceData_s& interface_data);

// void big_task(unsigned long curr_millis);
#endif // __VCR_SYSTEMTASKS_H__
=======
#endif // __VCR_SYSTEMTASKS_H__
>>>>>>> main
