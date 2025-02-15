#include "DrivebrainInterface.h"
#include "hytech.h"

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

    _latest_drivebrain_command.desired_speeds.veh_vec_data = {
        static_cast<float>(
            HYTECH_drivebrain_torque_fl_ro_fromS(drivebrain_msg.drivebrain_torque_fl_ro)),
        static_cast<float>(
            HYTECH_drivebrain_torque_fr_ro_fromS(drivebrain_msg.drivebrain_torque_fr_ro)),
        static_cast<float>(
            HYTECH_drivebrain_torque_rl_ro_fromS(drivebrain_msg.drivebrain_torque_rl_ro)),
        static_cast<float>(
            HYTECH_drivebrain_torque_rr_ro_fromS(drivebrain_msg.drivebrain_torque_rr_ro))};
}
