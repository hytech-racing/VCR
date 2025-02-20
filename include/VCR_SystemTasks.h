#ifndef VCR_SYSTEMTASKS_H
#define VCR_SYSTEMTASKS_H

#include "VCFInterface.h"

#include "SharedFirmwareTypes.h"

#include "DrivetrainSystem.h"
#include "VCRCANInterfaceImpl.h"
#include "VehicleStateMachine.h"
#include "etl/delegate.h"
#include "shared_types.h"

struct VCRAsynchronousInterfaces {
    explicit VCRAsynchronousInterfaces(CANInterfaces &can_ints) : can_interfaces(can_ints) {}
    VCRAsynchronousInterfaces() = delete;

    CANInterfaces &can_interfaces;
};

// outlining the main process that that is for handling the main systems
// that need interface data and produce outputs that need to get sent

VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
    VCRAsynchronousInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data);

VCRSystemData_s evaluate_async_systems(const VCRInterfaceData_s &interface_data);
CarState_e evaluate_state_machine(const VCRSystemData_s &system_data,
                                  const VCRInterfaceData_s &interface_data,
                                  VehicleStateMachine &state_machine);

#endif // __VCR_SYSTEMTASKS_H__
