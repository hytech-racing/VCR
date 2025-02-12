#include "etl/delegate.h"
#include "FlexCAN_T4.h"


#include "SharedFirmwareTypes.h"
#include "CANInterface.h"

#include "VCR_SystemTasks.h"
#include "VCFInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "VehicleStateMachine.h"

auto recv_call = etl::delegate<void(CANInterfaces& , const CAN_message_t& , unsigned long )>::create<VCRCANInterfaceImpl::vcr_CAN_recv>();

VCRInterfaceData_s sample_async_data(unsigned long millis, VCRInterfaces & interface_ref_container, const VCRInterfaceData_s & cur_vcr_int_data)
{
    VCRInterfaceData_s ret = cur_vcr_int_data;
    // process ring buffer is from CANInterface. TODO put into namespace
    process_ring_buffer(inverter_can_rx_buffer, interface_ref_container.can_interfaces, millis, recv_call);
    process_ring_buffer(telem_can_rx_buffer, interface_ref_container.can_interfaces, millis, recv_call);

    auto vcf_data = interface_ref_container.can_interfaces.vcf_interface.get_latest_data();
    ret.recvd_pedals_data = vcf_data.stamped_pedals;
    
    return ret;
}


// used for systems that dont need to interact with the state machine. i dont think that we need to pass in the previous s
VCRSystemData_s evaluate_systems(unsigned long millis, const VCRInterfaceData_s &interface_data, VCRSystems &systems)
{   
    VCRSystemData_s sys_data;
    return sys_data;
}

CarState_e evaluate_state_machine(unsigned long millis, const VCRData_s& system_data, const InterfaceData_s& interface_data, VehicleStateMachine& state_machine)
{
    // TODO make tick stat machine function return the state
    state_machine.tick_state_machine(millis, system_data);
    return {};
}

void big_task(unsigned long millis, VCRInterfaces & interface_ref_container, VCRSystems &systems, VehicleStateMachine& state_machine)
{
    auto interface_data = sample_async_data(millis, interface_ref_container);

    auto sys_data = evaluate_systems(millis, interface_data, systems);

    auto state = evaluate_state_machine(millis, sys_data, interface_data, state_machine);

}
