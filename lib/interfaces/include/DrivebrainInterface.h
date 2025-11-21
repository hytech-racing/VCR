#ifndef DRIVEBRAININTERFACE_H
#define DRIVEBRAININTERFACE_H

#include "IPAddress.h"
#include "etl/singleton.h"

#include "SharedFirmwareTypes.h"

#include "FlexCAN_T4.h"

#include "hytech_msgs.pb.h"

#include "ProtobufMsgInterface.h"
#include <QNEthernet.h>


class DrivebrainInterface {
  public:

    DrivebrainInterface(const RearLoadCellData_s &rear_load_cell_data,
                        const RearSusPotData_s &rear_suspot_data,
                        const ThermistorData_s &coolant_temperature_data_0,
                        const ThermistorData_s &coolant_temperature_data_1,
                        // add aero data
                        IPAddress drivebrain_ip,
                        uint16_t vcr_data_port, qindesign::network::EthernetUDP *udp_socket);
    void receive_drivebrain_speed_command(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_torque_lim_command(const CAN_message_t &msg, unsigned long curr_millis);
    
    void receive_drivebrain_aero11_CAN_data(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_aero12_CAN_data(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_aero21_CAN_data(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_aero22_CAN_data(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_aero31_CAN_data(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_aero32_CAN_data(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_aero41_CAN_data(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_aero42_CAN_data(const CAN_message_t &msg, unsigned long curr_millis);

    void handle_enqueue_suspension_CAN_data();

    void handle_enqueue_coolant_temp_CAN_data();

    void handle_enqueue_aero11_CAN_data();
    void handle_enqueue_aero12_CAN_data();
    void handle_enqueue_aero21_CAN_data();
    void handle_enqueue_aero22_CAN_data();
    void handle_enqueue_aero31_CAN_data();
    void handle_enqueue_aero32_CAN_data();
    void handle_enqueue_aero41_CAN_data();
    void handle_enqueue_aero42_CAN_data();




    void handle_send_ethernet_data(const hytech_msgs_VCRData_s &data);
    StampedDrivetrainCommand_s get_latest_data();

  private:
    struct {
        const RearLoadCellData_s &rear_load_cell_data;
        const RearSusPotData_s &rear_suspot_data;
    } _suspension_data;

    struct {
        const ThermistorData_s &coolant_temperature_0_data;
        const ThermistorData_s &coolant_temperature_1_data;
    } _thermistor_data;

    struct AeroData_s {
        uint16_t aero_channel_0;
        uint16_t aero_channel_1;
        uint16_t aero_channel_2;
        uint16_t aero_channel_3;
        uint16_t aero_channel_4;
        uint16_t aero_channel_5;
        uint16_t aero_channel_6;
        uint16_t aero_channel_7;
    };
    struct {
        AeroData_s aero_pressure_sensor_1;
        AeroData_s aero_pressure_sensor_2;
        AeroData_s aero_pressure_sensor_3;
        AeroData_s aero_pressure_sensor_4;
    } _aero_data;

    IPAddress _drivebrain_ip;
    uint16_t _vcr_data_port;
    qindesign::network::EthernetUDP *_udp_socket;
    StampedDrivetrainCommand_s _latest_drivebrain_command = {};
};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;
#endif // DRIVEBRAININTERFACE_H
