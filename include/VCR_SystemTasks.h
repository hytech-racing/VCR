#ifndef __VCR_SYSTEMTASKS_H__
#define __VCR_SYSTEMTASKS_H__

#include <tuple>
#include <utility>

#include "VCFInterface.h"

#include "SharedFirmwareTypes.h"

#include "VCRCANInterfaceImpl.h"
#include "VehicleStateMachine.h"
#include "DrivetrainSystem.h"

#include "shared_types.h"

struct VCRInterfaces
{
    explicit VCRInterfaces(CANInterfaces& can_ints) : can_interfaces(can_ints) {}
    VCRInterfaces() = delete;

    CANInterfaces& can_interfaces;
};

struct VCRSystems
{

};

struct InterfaceData_s
{
    PedalsData_s current_pedals_data;
};

struct SystemData_s
{

};

// outlining the main process that that is for handling the main systems 
// that need interface data and produce outputs that need to get sent

InterfaceData_s sample_interfaces(unsigned long curr_millis, VCRInterfaces& interface_ref_container);
VCRSystemData_s evaluate_systems(unsigned long curr_millis, const InterfaceData_s &interface_data, VCRSystems &systems_ref_container);
CarState_e evaluate_state_machine(unsigned long current_millis, const VCRSystemData_s& system_data, const InterfaceData_s& interface_data, VehicleStateMachine& state_machine);
void update_interfaces(const VCRSystemData_s& system_data, const InterfaceData_s& interface_data);

// void big_task(unsigned long curr_millis);
#endif // __VCR_SYSTEMTASKS_H__