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
                        const RearSusPotData_s &rear_suspot_data, IPAddress drivebrain_ip,
                        uint16_t vcr_data_port, qindesign::network::EthernetUDP *udp_socket);
    void receive_drivebrain_speed_command(const CAN_message_t &msg, unsigned long curr_millis);
    void receive_drivebrain_torque_lim_command(const CAN_message_t &msg, unsigned long curr_millis);

    void handle_enqueue_suspension_CAN_data();

    void handle_send_ethernet_data(const hytech_msgs_VCRData_s &data);
    StampedDrivetrainCommand_s get_latest_data();

  private:
    struct {
        const RearLoadCellData_s &rear_load_cell_data;
        const RearSusPotData_s &rear_suspot_data;
    } _suspension_data;

    IPAddress _drivebrain_ip;
    uint16_t _vcr_data_port;
    qindesign::network::EthernetUDP *_udp_socket;
    StampedDrivetrainCommand_s _latest_drivebrain_command = {};
};

using DrivebrainInterfaceInstance = etl::singleton<DrivebrainInterface>;
#endif // DRIVEBRAININTERFACE_H