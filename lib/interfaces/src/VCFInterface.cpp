#include "VCFInterface.h"
#include "hytech.h"

void VCFInterface::receive_pedals_message(const CAN_message_t& msg, unsigned long curr_millis)
{
    VCR_PEDALS_t vcr_pedals_msg;
    Unpack_VCR_PEDALS_hytech(&vcr_pedals_msg, &msg.buf[0], msg.len);
    _curr_data.pedals_data.accel = HYTECH_accel_pedal_ro_fromS(static_cast<float>(vcr_pedals_msg.accel_pedal_ro));
    _curr_data.pedals_data.brake = HYTECH_brake_pedal_ro_fromS(static_cast<float>(vcr_pedals_msg.brake_pedal_ro));
    _curr_data.pedals_data.last_recv_millis = curr_millis;
    // TODO need to ensure that the pedals data timestamp is within a tollerance within the state machine
}
    