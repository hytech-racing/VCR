#ifndef VCR_SYSTEMTASKS_H
#define VCR_SYSTEMTASKS_H

#include "VCFInterface.h"

#include "SharedFirmwareTypes.h"

#include "DrivetrainSystem.h"
#include "VCRCANInterfaceImpl.h"
#include "VehicleStateMachine.h"
#include "etl/delegate.h"
#include "shared_types.h"

struct VCRInterfaces {
    explicit VCRInterfaces(CANInterfaces &can_ints) : can_interfaces(can_ints) {}
    VCRInterfaces() = delete;

    CANInterfaces &can_interfaces;
};

struct VCRSystems {};

// outlining the main process that that is for handling the main systems
// that need interface data and produce outputs that need to get sent

VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
    VCRInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data);

VCRSystemData_s evaluate_systems(const VCRInterfaceData_s &interface_data,
                                 VCRSystems &systems_ref_container);
CarState_e evaluate_state_machine(const VCRSystemData_s &system_data,
                                  const VCRInterfaceData_s &interface_data,
                                  VehicleStateMachine &state_machine);
void update_interfaces(const VCRData_s &system_data, const VCRInterfaceData_s &interface_data);

#endif // __VCR_SYSTEMTASKS_H__
