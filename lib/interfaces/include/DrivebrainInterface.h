#ifndef __DRIVEBRAININTERFACE_H__
#define __DRIVEBRAININTERFACE_H__

#include "SharedFirmwareTypes.h"

#include "FlexCAN_T4.h"
class DrivebrainInterface {
  public:
    DrivebrainInterface() = default;
    void receive_drivebrain_speed_command(const CAN_message_t &msg,
                                          unsigned long curr_millis);
    void receive_drivebrain_torque_lim_command(const CAN_message_t &msg,
                                               unsigned long curr_millis);

    StampedDrivetrainCommand_s get_latest_data();

  private:
    StampedDrivetrainCommand_s _latest_drivebrain_command = {};
};

#endif // __DRIVEBRAININTERFACE_H__