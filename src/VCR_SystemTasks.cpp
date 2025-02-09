#include "etl/delegate.h"
#include "FlexCAN_T4.h"


#include "SharedFirmwareTypes.h"
#include "CANInterface.h"

#include "VCR_SystemTasks.h"
#include "VCFInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "VehicleStateMachine.h"

auto recv_call = etl::delegate<void(CANInterfaces& , const CAN_message_t& , unsigned long )>::create<VCRCANInterfaceImpl::vcr_CAN_recv>();

InterfaceData_s sample_interfaces(unsigned long curr_millis, VCRInterfaces & interface_ref_container)
{
    // process ring buffer is from CANInterface. TODO put into namespace
    process_ring_buffer(CAN2_rxBuffer, interface_ref_container.can_interfaces, curr_millis, recv_call);
    process_ring_buffer(CAN3_rxBuffer, interface_ref_container.can_interfaces, curr_millis, recv_call);

    auto current_vcf_data = interface_ref_container.can_interfaces.vcf_interface.get_latest_data();
    InterfaceData_s interf_data = {.current_pedals_data = current_vcf_data.pedals_data.recvd_data };
    
    return interf_data;
}


// used for systems that dont need to interact with the state machine
VCRSystemData_s evaluate_systems(unsigned long curr_millis, const InterfaceData_s &interface_data, VCRSystems &systems)
{   
    VCRSystemData_s sys_data;
    

    return sys_data;
}

CarState_e evaluate_state_machine(unsigned long current_millis, const VCRSystemData_s& system_data, const InterfaceData_s& interface_data, VehicleStateMachine& state_machine)
{
    // TODO make tick stat machine function return the state
    state_machine.tick_state_machine(current_millis, system_data);
    return {};
}

void big_task(unsigned long curr_millis, VCRInterfaces & interface_ref_container, VCRSystems &systems, VehicleStateMachine& state_machine)
{
    auto interface_data = sample_interfaces(curr_millis, interface_ref_container);

    auto sys_data = evaluate_systems(curr_millis, interface_data, systems);

    auto curr_state = evaluate_state_machine(curr_millis, sys_data, interface_data, state_machine);

}
