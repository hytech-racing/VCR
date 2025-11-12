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
#include <string>


VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recv_call,
    VCRAsynchronousInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data)
{
    VCRInterfaceData_s ret = cur_vcr_int_data;
    // process ring buffer is from CANInterface. TODO put into namespace
    process_ring_buffer(VCRCANInterfaceImpl::inverter_can_rx_buffer, interface_ref_container.can_interfaces,
                        sys_time::hal_millis(), recv_call, CANInterfaceType_e::INVERTER);
    process_ring_buffer(VCRCANInterfaceImpl::telem_can_rx_buffer, interface_ref_container.can_interfaces,
                        sys_time::hal_millis(), recv_call, CANInterfaceType_e::TELEM);
    process_ring_buffer(VCRCANInterfaceImpl::auxillary_can_rx_buffer, interface_ref_container.can_interfaces,
                        sys_time::hal_millis(), recv_call, CANInterfaceType_e::AUX);

    auto vcf_data = interface_ref_container.can_interfaces.vcf_interface.get_latest_data();
    auto acu_data = interface_ref_container.can_interfaces.acu_interface.get_latest_data(sys_time::hal_millis());
    auto drivebrain_telem_data = interface_ref_container.can_interfaces.db_interface.get_latest_telem_drivebrain_command();
    auto drivebrain_auxillary_data = interface_ref_container.can_interfaces.db_interface.get_latest_auxillary_drivebrain_command();

    auto fl_inv_mechanics = interface_ref_container.can_interfaces.fl_inverter_interface.get_motor_mechanics();
    auto fr_inv_mechanics = interface_ref_container.can_interfaces.fr_inverter_interface.get_motor_mechanics();
    auto rl_inv_mechanics = interface_ref_container.can_interfaces.rl_inverter_interface.get_motor_mechanics();
    auto rr_inv_mechanics = interface_ref_container.can_interfaces.rr_inverter_interface.get_motor_mechanics();

    ret.inverter_data.FL.speed_rpm = fl_inv_mechanics.actual_speed;
    ret.inverter_data.FR.speed_rpm = fr_inv_mechanics.actual_speed;
    ret.inverter_data.RL.speed_rpm = rl_inv_mechanics.actual_speed;
    ret.inverter_data.RR.speed_rpm = rr_inv_mechanics.actual_speed;

    ret.recvd_pedals_data = vcf_data.stamped_pedals;
    ret.front_loadcell_data = vcf_data.front_loadcell_data;
    ret.front_suspot_data = vcf_data.front_suspot_data;
    ret.dash_input_state = vcf_data.dash_input_state;
    ret.latest_drivebrain_telem_command = drivebrain_telem_data;
    ret.latest_drivebrain_auxillary_command = drivebrain_auxillary_data;

    return ret;
}

HT_TASK::TaskResponse run_async_main_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)>::create<VCRCANInterfaceImpl::vcr_CAN_recv>();

    bool torque_mode_cycle_button_was_pressed = vcr_data.interface_data.dash_input_state.mode_btn_is_pressed;

    VCRInterfaceData_s new_interface_data = sample_async_data(main_can_recv, VCRAsynchronousInterfacesInstance::instance(), vcr_data.interface_data);
    
    vcr_data.system_data.drivetrain_data.measuredSpeeds = {new_interface_data.inverter_data.FL.speed_rpm, new_interface_data.inverter_data.FR.speed_rpm, new_interface_data.inverter_data.RL.speed_rpm, new_interface_data.inverter_data.RR.speed_rpm};
    
    // If torque button was released (it was pressed before updating and now it's not)
    if (torque_mode_cycle_button_was_pressed && !new_interface_data.dash_input_state.mode_btn_is_pressed)
    {
        VCRControlsInstance::instance().cycle_torque_limit();
        VCFInterfaceInstance::instance().enqueue_torque_mode_LED_message(VCRControlsInstance::instance().get_current_torque_limit());
    }

    auto tc_mux_status = VCRControlsInstance::instance().get_tc_mux_status();
    vcr_data.system_data.tc_mux_status = tc_mux_status;

    vcr_data.system_data.vehicle_state_machine_state = VehicleStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());
    
    vcr_data.system_data.drivetrain_state_machine_state = DrivetrainInstance::instance().get_state();

    vcr_data.interface_data = new_interface_data;

    vcr_data.system_data.db_cntrl_status.drivebrain_is_in_control = VCRControlsInstance::instance().drivebrain_is_in_control();
    vcr_data.system_data.db_cntrl_status.drivebrain_controller_timing_failure = VCRControlsInstance::instance().drivebrain_timing_failure();
    
    return HT_TASK::TaskResponse::YIELD;
}