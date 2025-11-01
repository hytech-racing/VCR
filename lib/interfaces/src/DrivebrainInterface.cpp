#include "DrivebrainInterface.h"

#include "FlexCAN_T4.h"

#include "CANInterface.h"
#include "VCRCANInterfaceImpl.h"

#include "hytech.h" // HT_can

#include "hytech_msgs.pb.h"
#include <cstdint>

DrivebrainInterface::DrivebrainInterface(const RearLoadCellData_s &rear_load_cell_data,
                                         const RearSusPotData_s &rear_suspot_data,
                                         const ThermistorData_s &coolant_temperature_data_0,
                                         const ThermistorData_s &coolant_temperature_data_1,
                                         const ThermistorData_s &flowmeter_data,
                                         IPAddress drivebrain_ip, uint16_t vcr_data_port,
                                         qindesign::network::EthernetUDP *udp_socket)
    : _suspension_data{.rear_load_cell_data = rear_load_cell_data,
                       .rear_suspot_data = rear_suspot_data},
      _thermistor_data{.coolant_temperature_0_data = coolant_temperature_data_0,
                       .coolant_temperature_1_data = coolant_temperature_data_1,
                       .flowmeter_data = flowmeter_data 
                        },
      _drivebrain_ip(drivebrain_ip),
      _vcr_data_port(vcr_data_port),
      _udp_socket(udp_socket) { };

StampedDrivetrainCommand_s DrivebrainInterface::get_latest_data() {
    return _latest_drivebrain_command;
}

void DrivebrainInterface::receive_drivebrain_speed_command(const CAN_message_t &msg,
                                                           unsigned long curr_millis) {
    DRIVEBRAIN_SPEED_SET_INPUT_t drivebrain_msg;

    Unpack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(&drivebrain_msg, &msg.buf[0], msg.len);

    _latest_drivebrain_command.desired_speeds.recvd = true;
    _latest_drivebrain_command.desired_speeds.last_recv_millis = curr_millis;

    _latest_drivebrain_command.desired_speeds.veh_vec_data = {
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_fl),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_fr),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_rl),
        static_cast<float>(drivebrain_msg.drivebrain_set_rpm_rr)};
};

void DrivebrainInterface::receive_drivebrain_torque_lim_command(const CAN_message_t &msg,
                                                                unsigned long curr_millis) {
    DRIVEBRAIN_TORQUE_LIM_INPUT_t drivebrain_msg;

    Unpack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(&drivebrain_msg, &msg.buf[0], msg.len);

    _latest_drivebrain_command.torque_limits.recvd = true;
    _latest_drivebrain_command.torque_limits.last_recv_millis = curr_millis;

    _latest_drivebrain_command.torque_limits.veh_vec_data = {
        static_cast<float>(
            HYTECH_drivebrain_torque_fl_ro_fromS(drivebrain_msg.drivebrain_torque_fl_ro)),
        static_cast<float>(
            HYTECH_drivebrain_torque_fr_ro_fromS(drivebrain_msg.drivebrain_torque_fr_ro)),
        static_cast<float>(
            HYTECH_drivebrain_torque_rl_ro_fromS(drivebrain_msg.drivebrain_torque_rl_ro)),
        static_cast<float>(
            HYTECH_drivebrain_torque_rr_ro_fromS(drivebrain_msg.drivebrain_torque_rr_ro))};
}

void DrivebrainInterface::handle_enqueue_suspension_CAN_data() {
    REAR_SUSPENSION_t rear_sus_msg;
    rear_sus_msg.rl_load_cell = _suspension_data.rear_load_cell_data.RL_loadcell_analog;
    rear_sus_msg.rr_load_cell = _suspension_data.rear_load_cell_data.RR_loadcell_analog;
    rear_sus_msg.rl_shock_pot = _suspension_data.rear_suspot_data.RL_sus_pot_analog;
    rear_sus_msg.rr_shock_pot = _suspension_data.rear_suspot_data.RR_sus_pot_analog;

    CAN_util::enqueue_msg(&rear_sus_msg, &Pack_REAR_SUSPENSION_hytech,
                          VCRCANInterfaceImpl::telem_can_rx_buffer); //CHANGE BACK TO TELEM_CAN_TX_BUFFER
}

void DrivebrainInterface::handle_enqueue_coolant_temp_CAN_data() {
    REAR_THERMISTORS_DATA_t thermistor_msg;
    thermistor_msg.thermistor_0_deg_C_ro = HYTECH_thermistor_0_deg_C_ro_toS(_thermistor_data.coolant_temperature_0_data.thermistor_degrees_C);
    thermistor_msg.thermistor_1_deg_C_ro = HYTECH_thermistor_1_deg_C_ro_toS(_thermistor_data.coolant_temperature_1_data.thermistor_degrees_C);
    thermistor_msg.thermistor_2_deg_C_ro = HYTECH_thermistor_2_deg_C_ro_toS(_thermistor_data.flowmeter_data.thermistor_degrees_C);
    CAN_util::enqueue_msg(&thermistor_msg, &Pack_REAR_THERMISTORS_DATA_hytech, VCRCANInterfaceImpl::telem_can_tx_buffer);
}

void DrivebrainInterface::handle_send_ethernet_data(const hytech_msgs_VCRData_s &data) {
    handle_ethernet_socket_send_pb<hytech_msgs_VCRData_s_size>(_drivebrain_ip, _vcr_data_port, _udp_socket, data,
                                   hytech_msgs_VCRData_s_fields);
}