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
#include "QNEthernet.h"
#include "ht_sched.hpp"
#include "controls.h"


/**
 * Generally, our main "loop" (not actually a loop) will run as fast as possible and will go through
 * three stages.
 * 
 * First, it will sample all asynchronous data (i.e. CAN and Ethernet), since those will
 * asynchronously arrive and be added to a buffer. This ensures that we are processing this data as
 * quickly as possible, instead of simply at the next "tick" at 50/100/1000Hz.
 * 
 * Second, it will evaluate all systems based on this updated data.
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

using VCRAsynchronousInterfacesInstance = etl::singleton<VCRAsynchronousInterfaces>;

/**
 * Wrapper to contain all Protobuf send/recv sockets.
 */
struct ProtobufSockets_s {
    explicit ProtobufSockets_s(qindesign::network::EthernetUDP &vcr_data_send, qindesign::network::EthernetUDP &vcf_data_recv) :
        vcr_data_send_socket(vcr_data_send),
        vcf_data_recv_socket(vcf_data_recv) {};

    qindesign::network::EthernetUDP &vcr_data_send_socket;
    qindesign::network::EthernetUDP &vcf_data_recv_socket;
};

using ProtobufSocketsInstance = etl::singleton<ProtobufSockets_s>;

/**
 * Reads all asynchronously-arriving data (CAN and Ethernet) from their respective buffers and updates their
 * timestamped values in VCRInterfaceData_s (returns the new struct). Takes in a bundle of all the asynchronous
 * interfaces to be ticked. 
 */
VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
    VCRAsynchronousInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data,
    ProtobufSockets_s sockets
);

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


HT_TASK::TaskResponse run_main_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

#endif // __VCR_SYSTEMTASKS_H__
