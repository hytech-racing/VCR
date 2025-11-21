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
                                         // add aero data?
                                         IPAddress drivebrain_ip, uint16_t vcr_data_port,
                                         qindesign::network::EthernetUDP *udp_socket)
    : _suspension_data{.rear_load_cell_data = rear_load_cell_data,
                       .rear_suspot_data = rear_suspot_data},
      _thermistor_data{.coolant_temperature_0_data = coolant_temperature_data_0,
                       .coolant_temperature_1_data = coolant_temperature_data_1},
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

void DrivebrainInterface::receive_drivebrain_aero11_CAN_data(const CAN_message_t &msg,
                                                            unsigned long curr_millis) {
    AERO_PRESSURE_SENSOR_11_t drivebrain_msg;

    Unpack_AERO_PRESSURE_SENSOR_11_hytech(&drivebrain_msg, &msg.buf[0], msg.len);
    
    _aero_data.aero_pressure_sensor_1.aero_channel_0 = static_cast<float>(HYTECH_aero_channel_0_ro_fromS(drivebrain_msg.aero_channel_0_ro));
    _aero_data.aero_pressure_sensor_1.aero_channel_1 = static_cast<float>(HYTECH_aero_channel_1_ro_fromS(drivebrain_msg.aero_channel_1_ro));
    _aero_data.aero_pressure_sensor_1.aero_channel_2 = static_cast<float>(HYTECH_aero_channel_2_ro_fromS(drivebrain_msg.aero_channel_2_ro));
    _aero_data.aero_pressure_sensor_1.aero_channel_3 = static_cast<float>(HYTECH_aero_channel_3_ro_fromS(drivebrain_msg.aero_channel_3_ro));
    }
void DrivebrainInterface::receive_drivebrain_aero12_CAN_data(const CAN_message_t &msg,
                                                            unsigned long curr_millis) {
    AERO_PRESSURE_SENSOR_12_t drivebrain_msg;

    Unpack_AERO_PRESSURE_SENSOR_12_hytech(&drivebrain_msg, &msg.buf[0], msg.len);
    
    _aero_data.aero_pressure_sensor_1.aero_channel_4 = static_cast<float>(HYTECH_aero_channel_4_ro_fromS(drivebrain_msg.aero_channel_4_ro));
    _aero_data.aero_pressure_sensor_1.aero_channel_5 = static_cast<float>(HYTECH_aero_channel_5_ro_fromS(drivebrain_msg.aero_channel_5_ro));
    _aero_data.aero_pressure_sensor_1.aero_channel_6 = static_cast<float>(HYTECH_aero_channel_6_ro_fromS(drivebrain_msg.aero_channel_6_ro));
    _aero_data.aero_pressure_sensor_1.aero_channel_7 = static_cast<float>(HYTECH_aero_channel_7_ro_fromS(drivebrain_msg.aero_channel_7_ro));
    }
void DrivebrainInterface::receive_drivebrain_aero21_CAN_data(const CAN_message_t &msg,
                                                            unsigned long curr_millis) {
    AERO_PRESSURE_SENSOR_21_t drivebrain_msg;

    Unpack_AERO_PRESSURE_SENSOR_21_hytech(&drivebrain_msg, &msg.buf[0], msg.len);
    
    _aero_data.aero_pressure_sensor_2.aero_channel_0 = static_cast<float>(HYTECH_aero_channel_0_ro_fromS(drivebrain_msg.aero_channel_0_ro));
    _aero_data.aero_pressure_sensor_2.aero_channel_1 = static_cast<float>(HYTECH_aero_channel_1_ro_fromS(drivebrain_msg.aero_channel_1_ro));
    _aero_data.aero_pressure_sensor_2.aero_channel_2 = static_cast<float>(HYTECH_aero_channel_2_ro_fromS(drivebrain_msg.aero_channel_2_ro));
    _aero_data.aero_pressure_sensor_2.aero_channel_3 = static_cast<float>(HYTECH_aero_channel_3_ro_fromS(drivebrain_msg.aero_channel_3_ro));
    }
void DrivebrainInterface::receive_drivebrain_aero22_CAN_data(const CAN_message_t &msg,
                                                            unsigned long curr_millis) {
    AERO_PRESSURE_SENSOR_22_t drivebrain_msg;

    Unpack_AERO_PRESSURE_SENSOR_22_hytech(&drivebrain_msg, &msg.buf[0], msg.len);
    
    _aero_data.aero_pressure_sensor_2.aero_channel_4 = static_cast<float>(HYTECH_aero_channel_4_ro_fromS(drivebrain_msg.aero_channel_4_ro));
    _aero_data.aero_pressure_sensor_2.aero_channel_5 = static_cast<float>(HYTECH_aero_channel_5_ro_fromS(drivebrain_msg.aero_channel_5_ro));
    _aero_data.aero_pressure_sensor_2.aero_channel_6 = static_cast<float>(HYTECH_aero_channel_6_ro_fromS(drivebrain_msg.aero_channel_6_ro));
    _aero_data.aero_pressure_sensor_2.aero_channel_7 = static_cast<float>(HYTECH_aero_channel_7_ro_fromS(drivebrain_msg.aero_channel_7_ro));
    }
void DrivebrainInterface::receive_drivebrain_aero31_CAN_data(const CAN_message_t &msg,
                                                            unsigned long curr_millis) {
    AERO_PRESSURE_SENSOR_31_t drivebrain_msg;

    Unpack_AERO_PRESSURE_SENSOR_31_hytech(&drivebrain_msg, &msg.buf[0], msg.len);
    
    _aero_data.aero_pressure_sensor_3.aero_channel_0 = static_cast<float>(HYTECH_aero_channel_0_ro_fromS(drivebrain_msg.aero_channel_0_ro));
    _aero_data.aero_pressure_sensor_3.aero_channel_1 = static_cast<float>(HYTECH_aero_channel_1_ro_fromS(drivebrain_msg.aero_channel_1_ro));
    _aero_data.aero_pressure_sensor_3.aero_channel_2 = static_cast<float>(HYTECH_aero_channel_2_ro_fromS(drivebrain_msg.aero_channel_2_ro));
    _aero_data.aero_pressure_sensor_3.aero_channel_3 = static_cast<float>(HYTECH_aero_channel_3_ro_fromS(drivebrain_msg.aero_channel_3_ro));
    }
void DrivebrainInterface::receive_drivebrain_aero32_CAN_data(const CAN_message_t &msg,
                                                            unsigned long curr_millis) {
    AERO_PRESSURE_SENSOR_32_t drivebrain_msg;

    Unpack_AERO_PRESSURE_SENSOR_32_hytech(&drivebrain_msg, &msg.buf[0], msg.len);
    
    _aero_data.aero_pressure_sensor_3.aero_channel_4 = static_cast<float>(HYTECH_aero_channel_4_ro_fromS(drivebrain_msg.aero_channel_4_ro));
    _aero_data.aero_pressure_sensor_3.aero_channel_5 = static_cast<float>(HYTECH_aero_channel_5_ro_fromS(drivebrain_msg.aero_channel_5_ro));
    _aero_data.aero_pressure_sensor_3.aero_channel_6 = static_cast<float>(HYTECH_aero_channel_6_ro_fromS(drivebrain_msg.aero_channel_6_ro));
    _aero_data.aero_pressure_sensor_3.aero_channel_7 = static_cast<float>(HYTECH_aero_channel_7_ro_fromS(drivebrain_msg.aero_channel_7_ro));
    }
void DrivebrainInterface::receive_drivebrain_aero41_CAN_data(const CAN_message_t &msg,
                                                            unsigned long curr_millis) {
    AERO_PRESSURE_SENSOR_41_t drivebrain_msg;

    Unpack_AERO_PRESSURE_SENSOR_41_hytech(&drivebrain_msg, &msg.buf[0], msg.len);
    
    _aero_data.aero_pressure_sensor_4.aero_channel_0 = static_cast<float>(HYTECH_aero_channel_0_ro_fromS(drivebrain_msg.aero_channel_0_ro));
    _aero_data.aero_pressure_sensor_4.aero_channel_1 = static_cast<float>(HYTECH_aero_channel_1_ro_fromS(drivebrain_msg.aero_channel_1_ro));
    _aero_data.aero_pressure_sensor_4.aero_channel_2 = static_cast<float>(HYTECH_aero_channel_2_ro_fromS(drivebrain_msg.aero_channel_2_ro));
    _aero_data.aero_pressure_sensor_4.aero_channel_3 = static_cast<float>(HYTECH_aero_channel_3_ro_fromS(drivebrain_msg.aero_channel_3_ro));
    }
void DrivebrainInterface::receive_drivebrain_aero42_CAN_data(const CAN_message_t &msg,
                                                            unsigned long curr_millis) {
    AERO_PRESSURE_SENSOR_42_t drivebrain_msg;

    Unpack_AERO_PRESSURE_SENSOR_42_hytech(&drivebrain_msg, &msg.buf[0], msg.len);
    
    _aero_data.aero_pressure_sensor_4.aero_channel_4 = static_cast<float>(HYTECH_aero_channel_4_ro_fromS(drivebrain_msg.aero_channel_4_ro));
    _aero_data.aero_pressure_sensor_4.aero_channel_5 = static_cast<float>(HYTECH_aero_channel_5_ro_fromS(drivebrain_msg.aero_channel_5_ro));
    _aero_data.aero_pressure_sensor_4.aero_channel_6 = static_cast<float>(HYTECH_aero_channel_6_ro_fromS(drivebrain_msg.aero_channel_6_ro));
    _aero_data.aero_pressure_sensor_4.aero_channel_7 = static_cast<float>(HYTECH_aero_channel_7_ro_fromS(drivebrain_msg.aero_channel_7_ro));
    }

void DrivebrainInterface::handle_enqueue_suspension_CAN_data() {
    REAR_SUSPENSION_t rear_sus_msg;
    rear_sus_msg.rl_load_cell = _suspension_data.rear_load_cell_data.RL_loadcell_analog;
    rear_sus_msg.rr_load_cell = _suspension_data.rear_load_cell_data.RR_loadcell_analog;
    rear_sus_msg.rl_shock_pot = _suspension_data.rear_suspot_data.RL_sus_pot_analog;
    rear_sus_msg.rr_shock_pot = _suspension_data.rear_suspot_data.RR_sus_pot_analog;

    CAN_util::enqueue_msg(&rear_sus_msg, &Pack_REAR_SUSPENSION_hytech,
                          VCRCANInterfaceImpl::telem_can_tx_buffer);
}

void DrivebrainInterface::handle_enqueue_coolant_temp_CAN_data() {
    REAR_THERMISTORS_DATA_t thermistor_msg;
    thermistor_msg.thermistor_0_deg_C_ro = HYTECH_thermistor_0_deg_C_ro_toS(_thermistor_data.coolant_temperature_0_data.thermistor_degrees_C);
    thermistor_msg.thermistor_1_deg_C_ro = HYTECH_thermistor_1_deg_C_ro_toS(_thermistor_data.coolant_temperature_1_data.thermistor_degrees_C);

    CAN_util::enqueue_msg(&thermistor_msg, &Pack_REAR_THERMISTORS_DATA_hytech, VCRCANInterfaceImpl::telem_can_tx_buffer);
}

//aero enqueue functions
void DrivebrainInterface::handle_enqueue_aero11_CAN_data() {
    AERO_PRESSURE_SENSOR_11_t aero11_msg;

    aero11_msg.aero_channel_0_ro = HYTECH_aero_channel_0_ro_toS(_aero_data.aero_pressure_sensor_1.aero_channel_0);
    aero11_msg.aero_channel_1_ro = HYTECH_aero_channel_1_ro_toS(_aero_data.aero_pressure_sensor_1.aero_channel_1);
    aero11_msg.aero_channel_2_ro = HYTECH_aero_channel_2_ro_toS(_aero_data.aero_pressure_sensor_1.aero_channel_2);
    aero11_msg.aero_channel_3_ro = HYTECH_aero_channel_3_ro_toS(_aero_data.aero_pressure_sensor_1.aero_channel_3);

    CAN_util::enqueue_msg(&aero11_msg, &Pack_AERO_PRESSURE_SENSOR_11_hytech, VCRCANInterfaceImpl::CAN1_txBuffer);
 
}

void DrivebrainInterface::handle_enqueue_aero12_CAN_data() {
    AERO_PRESSURE_SENSOR_12_t aero12_msg;

    aero12_msg.aero_channel_4_ro = HYTECH_aero_channel_4_ro_toS(_aero_data.aero_pressure_sensor_1.aero_channel_4);
    aero12_msg.aero_channel_5_ro = HYTECH_aero_channel_5_ro_toS(_aero_data.aero_pressure_sensor_1.aero_channel_5);
    aero12_msg.aero_channel_6_ro = HYTECH_aero_channel_6_ro_toS(_aero_data.aero_pressure_sensor_1.aero_channel_6);
    aero12_msg.aero_channel_7_ro = HYTECH_aero_channel_7_ro_toS(_aero_data.aero_pressure_sensor_1.aero_channel_7);

    CAN_util::enqueue_msg(&aero12_msg, &Pack_AERO_PRESSURE_SENSOR_12_hytech, VCRCANInterfaceImpl::CAN1_txBuffer);
 
}

void DrivebrainInterface::handle_enqueue_aero21_CAN_data() {
    AERO_PRESSURE_SENSOR_21_t aero21_msg;

    aero21_msg.aero_channel_0_ro = HYTECH_aero_channel_0_ro_toS(_aero_data.aero_pressure_sensor_2.aero_channel_0);
    aero21_msg.aero_channel_1_ro = HYTECH_aero_channel_1_ro_toS(_aero_data.aero_pressure_sensor_2.aero_channel_1);
    aero21_msg.aero_channel_2_ro = HYTECH_aero_channel_2_ro_toS(_aero_data.aero_pressure_sensor_2.aero_channel_2);
    aero21_msg.aero_channel_3_ro = HYTECH_aero_channel_3_ro_toS(_aero_data.aero_pressure_sensor_2.aero_channel_3);

    CAN_util::enqueue_msg(&aero21_msg, &Pack_AERO_PRESSURE_SENSOR_21_hytech, VCRCANInterfaceImpl::CAN1_txBuffer);
 
}

void DrivebrainInterface::handle_enqueue_aero22_CAN_data() {
    AERO_PRESSURE_SENSOR_22_t aero22_msg;

    aero22_msg.aero_channel_4_ro = HYTECH_aero_channel_4_ro_toS(_aero_data.aero_pressure_sensor_2.aero_channel_4);
    aero22_msg.aero_channel_5_ro = HYTECH_aero_channel_5_ro_toS(_aero_data.aero_pressure_sensor_2.aero_channel_5);
    aero22_msg.aero_channel_6_ro = HYTECH_aero_channel_6_ro_toS(_aero_data.aero_pressure_sensor_2.aero_channel_6);
    aero22_msg.aero_channel_7_ro = HYTECH_aero_channel_7_ro_toS(_aero_data.aero_pressure_sensor_2.aero_channel_7);

    CAN_util::enqueue_msg(&aero22_msg, &Pack_AERO_PRESSURE_SENSOR_22_hytech, VCRCANInterfaceImpl::CAN1_txBuffer);
 
}

void DrivebrainInterface::handle_enqueue_aero31_CAN_data() {
    AERO_PRESSURE_SENSOR_31_t aero31_msg;

    aero31_msg.aero_channel_0_ro = HYTECH_aero_channel_0_ro_toS(_aero_data.aero_pressure_sensor_3.aero_channel_0);
    aero31_msg.aero_channel_1_ro = HYTECH_aero_channel_1_ro_toS(_aero_data.aero_pressure_sensor_3.aero_channel_1);
    aero31_msg.aero_channel_2_ro = HYTECH_aero_channel_2_ro_toS(_aero_data.aero_pressure_sensor_3.aero_channel_2);
    aero31_msg.aero_channel_3_ro = HYTECH_aero_channel_3_ro_toS(_aero_data.aero_pressure_sensor_3.aero_channel_3);

    CAN_util::enqueue_msg(&aero31_msg, &Pack_AERO_PRESSURE_SENSOR_31_hytech, VCRCANInterfaceImpl::CAN1_txBuffer);
 
}

void DrivebrainInterface::handle_enqueue_aero32_CAN_data() {
    AERO_PRESSURE_SENSOR_32_t aero32_msg;

    aero32_msg.aero_channel_4_ro = HYTECH_aero_channel_4_ro_toS(_aero_data.aero_pressure_sensor_3.aero_channel_4);
    aero32_msg.aero_channel_5_ro = HYTECH_aero_channel_5_ro_toS(_aero_data.aero_pressure_sensor_3.aero_channel_5);
    aero32_msg.aero_channel_6_ro = HYTECH_aero_channel_6_ro_toS(_aero_data.aero_pressure_sensor_3.aero_channel_6);
    aero32_msg.aero_channel_7_ro = HYTECH_aero_channel_7_ro_toS(_aero_data.aero_pressure_sensor_3.aero_channel_7);

    CAN_util::enqueue_msg(&aero32_msg, &Pack_AERO_PRESSURE_SENSOR_32_hytech, VCRCANInterfaceImpl::CAN1_txBuffer);
 
}

void DrivebrainInterface::handle_enqueue_aero41_CAN_data() {
    AERO_PRESSURE_SENSOR_41_t aero41_msg;

    aero41_msg.aero_channel_0_ro = HYTECH_aero_channel_0_ro_toS(_aero_data.aero_pressure_sensor_4.aero_channel_0);
    aero41_msg.aero_channel_1_ro = HYTECH_aero_channel_1_ro_toS(_aero_data.aero_pressure_sensor_4.aero_channel_1);
    aero41_msg.aero_channel_2_ro = HYTECH_aero_channel_2_ro_toS(_aero_data.aero_pressure_sensor_4.aero_channel_2);
    aero41_msg.aero_channel_3_ro = HYTECH_aero_channel_3_ro_toS(_aero_data.aero_pressure_sensor_4.aero_channel_3);
    CAN_util::enqueue_msg(&aero41_msg, &Pack_AERO_PRESSURE_SENSOR_41_hytech, VCRCANInterfaceImpl::CAN1_txBuffer);
 
}

void DrivebrainInterface::handle_enqueue_aero42_CAN_data() {
    AERO_PRESSURE_SENSOR_42_t aero42_msg;

    aero42_msg.aero_channel_4_ro = HYTECH_aero_channel_4_ro_toS(_aero_data.aero_pressure_sensor_4.aero_channel_4);
    aero42_msg.aero_channel_5_ro = HYTECH_aero_channel_5_ro_toS(_aero_data.aero_pressure_sensor_4.aero_channel_5);
    aero42_msg.aero_channel_6_ro = HYTECH_aero_channel_6_ro_toS(_aero_data.aero_pressure_sensor_4.aero_channel_6);
    aero42_msg.aero_channel_7_ro = HYTECH_aero_channel_7_ro_toS(_aero_data.aero_pressure_sensor_4.aero_channel_7);

    CAN_util::enqueue_msg(&aero42_msg, &Pack_AERO_PRESSURE_SENSOR_42_hytech, VCRCANInterfaceImpl::CAN1_txBuffer);
 
}


void DrivebrainInterface::handle_send_ethernet_data(const hytech_msgs_VCRData_s &data) {
    handle_ethernet_socket_send_pb<hytech_msgs_VCRData_s_size>(_drivebrain_ip, _vcr_data_port, _udp_socket, data,
                                   hytech_msgs_VCRData_s_fields);
}
