#include "FlexCAN_T4.h"
#include "VCR_Globals.h"
#include "etl/delegate.h"

#include "CANInterface.h"
#include "SharedFirmwareTypes.h"

#include "VCFInterface.h"
#include "ACUInterface.h"
#include "VCREthernetInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "VCR_SystemTasks.h"


VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
    VCRAsynchronousInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data)
{
    VCRInterfaceData_s ret = cur_vcr_int_data;
    // process ring buffer is from CANInterface. TODO put into namespace
    process_ring_buffer(VCRCANInterfaceImpl::inverter_can_rx_buffer, interface_ref_container.can_interfaces,
                        sys_time::hal_millis(), recv_call);
    process_ring_buffer(VCRCANInterfaceImpl::telem_can_rx_buffer, interface_ref_container.can_interfaces,
                        sys_time::hal_millis(), recv_call);

    auto vcf_data = interface_ref_container.can_interfaces.vcf_interface.get_latest_data();
    auto acu_data = interface_ref_container.can_interfaces.acu_interface.get_latest_data(sys_time::hal_millis());
    auto drivebrain_data = interface_ref_container.can_interfaces.db_interface.get_latest_data();
    ret.recvd_pedals_data = vcf_data.stamped_pedals;
    ret.front_loadcell_data = vcf_data.front_loadcell_data;
    ret.front_suspot_data = vcf_data.front_suspot_data;
    ret.dash_input_state = vcf_data.dash_input_state;
    ret.latest_drivebrain_command = drivebrain_data;

    return ret;
}

HT_TASK::TaskResponse run_async_main_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCRCANInterfaceImpl::vcr_CAN_recv>();

    bool torque_mode_cycle_button_was_pressed = vcr_data.interface_data.dash_input_state.mode_btn_is_pressed;

    VCRInterfaceData_s new_interface_data = sample_async_data(main_can_recv, VCRAsynchronousInterfacesInstance::instance(), vcr_data.interface_data);

    // If torque button was released (it was pressed before updating and now it's not)
    if (torque_mode_cycle_button_was_pressed && !new_interface_data.dash_input_state.mode_btn_is_pressed)
    {
        VCRControlsInstance::instance().cycle_torque_limit();
        VCFInterfaceInstance::instance().enqueue_torque_mode_LED_message(VCRControlsInstance::instance().get_current_torque_limit());
    }

    auto tc_mux_status = VCRControlsInstance::instance().get_tc_mux_status();
    vcr_data.system_data.tc_mux_status = tc_mux_status;

    VehicleState_e state = VehicleStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis()); // NOLINT (linter says state is not initialized?)
    vcr_data.system_data.vehicle_state_machine_state = state;
    
    vcr_data.interface_data = new_interface_data;

    return HT_TASK::TaskResponse::YIELD;
}