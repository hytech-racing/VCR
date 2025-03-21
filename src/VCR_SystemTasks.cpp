#include "FlexCAN_T4.h"
#include "VCR_Globals.h"
#include "etl/delegate.h"

#include "CANInterface.h"
#include "SharedFirmwareTypes.h"

#include "VCFInterface.h"
#include "VCREthernetInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "VCR_SystemTasks.h"


VCRInterfaceData_s sample_async_data(
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> recv_call,
    VCRAsynchronousInterfaces &interface_ref_container, const VCRInterfaceData_s &cur_vcr_int_data,
    ProtobufSockets_s sockets)
{
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

    etl::optional<hytech_msgs_VCFData_s> vcf_data_protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCFData_s_size, hytech_msgs_VCFData_s>(&sockets.vcf_data_recv_socket, &hytech_msgs_VCFData_s_msg);
    if (vcf_data_protoc_struct)
        VCREthernetInterface::receive_pb_msg_vcf(vcf_data_protoc_struct.value(), vcr_data, millis());

    etl::optional<hytech_msgs_ACUCoreData_s> acu_core_data_protoc_struct = handle_ethernet_socket_receive<hytech_msgs_ACUCoreData_s_size, hytech_msgs_ACUCoreData_s>(&sockets.acu_core_data_recv_socket, &hytech_msgs_ACUCoreData_s_msg);
    if (acu_core_data_protoc_struct)
        VCREthernetInterface::receive_pb_msg_acu_core_data(acu_core_data_protoc_struct.value(), vcr_data, millis());
    
    etl::optional<hytech_msgs_ACUAllData_s> acu_all_data_protoc_struct = handle_ethernet_socket_receive<hytech_msgs_ACUAllData_s_size, hytech_msgs_ACUAllData_s>(&sockets.acu_all_data_recv_socket, &hytech_msgs_ACUAllData_s_msg);
    if (acu_all_data_protoc_struct)
        VCREthernetInterface::receive_pb_msg_acu_all_data(acu_all_data_protoc_struct.value(), vcr_data);

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