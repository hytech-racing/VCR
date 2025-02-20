#include "FlexCAN_T4.h"
#include "VCR_Globals.h"
#include "etl/delegate.h"

#include "CANInterface.h"
#include "SharedFirmwareTypes.h"

#include "VCFInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "VCR_SystemTasks.h"
#include "VehicleStateMachine.h"

#include "SystemTimeInterface.h"

VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
    VCRAsynchronousInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data) {
    VCRInterfaceData_s ret = cur_vcr_int_data;
    // process ring buffer is from CANInterface. TODO put into namespace
    process_ring_buffer(VCRCANInterfaceImpl::inverter_can_rx_buffer, interface_ref_container.can_interfaces,
                        sys_time::hal_millis(), recv_call);
    process_ring_buffer(VCRCANInterfaceImpl::telem_can_rx_buffer, interface_ref_container.can_interfaces,
                        sys_time::hal_millis(), recv_call);

    auto vcf_data = interface_ref_container.can_interfaces.vcf_interface.get_latest_data();
    auto drivebrain_data = interface_ref_container.can_interfaces.db_interface.get_latest_data();
    ret.recvd_pedals_data = vcf_data.stamped_pedals;
    ret.latest_drivebrain_command = drivebrain_data;
    return ret;
}

VCRSystemData_s evaluate_async_systems(const VCRInterfaceData_s &interface_data) {
    VCRSystemData_s sys_data;


    /*
    this could include: 
    - controllers we want to always be evaluating regardless of if they are active or not
    - low-level filters / estimators
    - debug systems
    - low-level parameter system 
    - <etc>
    */ 
    return sys_data;
}

CarState_e evaluate_state_machine(const VCRData_s &system_data,
                                  const VCRInterfaceData_s &interface_data,
                                  VehicleStateMachine &state_machine) {
    // TODO make tick stat machine function return the state
    state_machine.tick_state_machine(sys_time::hal_millis(), system_data);
    return {};
}

void big_task(etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
              VCRAsynchronousInterfaces &interface_ref_container,
              VehicleStateMachine &state_machine, const VCRInterfaceData_s &cur_vcr_int_data) {
    auto interface_data = sample_async_data(recv_call, interface_ref_container, cur_vcr_int_data);

    auto sys_data = evaluate_async_systems(interface_data);

    auto state = evaluate_state_machine(sys_data, interface_data, state_machine);

    vcr_data.system_data = sys_data;
    vcr_data.interface_data = interface_data;
    // TODO the car state needs to be part of the vcr data (this is not in interface or system data)

}
