#ifndef VCR_SYSTEMTASKS_H
#define VCR_SYSTEMTASKS_H


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
    PedalsSystemData_s pedals_data;
};

struct SystemData_s
{

};

// outlining the main process that that is for handling the main systems 
// that need interface data and produce outputs that need to get sent

InterfaceData_s sample_async_data(VCRInterfaces& interface_ref_container);
VCRData_s evaluate_systems(const InterfaceData_s &interface_data, VCRSystems &systems_ref_container);
CarState_e evaluate_state_machine(const VCRData_s& system_data, const InterfaceData_s& interface_data, VehicleStateMachine& state_machine);
void update_interfaces(const VCRData_s& system_data, const InterfaceData_s& interface_data);

// void big_task(unsigned long curr_millis);
#endif // __VCR_SYSTEMTASKS_H__