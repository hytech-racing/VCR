#include "VCFInterface.h"
#include "hytech.h"

void VCFInterface::receive_pedals_message(const CAN_message_t& msg, unsigned long curr_millis)
{
    PEDALS_SYSTEM_DATA_t pedals_msg;
    Unpack_PEDALS_SYSTEM_DATA_hytech(&pedals_msg, &msg.buf[0], msg.len);
    _curr_data.pedals_data.recvd_data.implausibility_has_exceeded_max_duration = pedals_msg.implaus_exceeded_max_duration;
    
    _curr_data.pedals_data.recvd_data.brake_and_accel_pressed_implausibility_high = pedals_msg.brake_accel_implausibility;
    
    _curr_data.pedals_data.recvd_data.accel_is_implausible = pedals_msg.accel_implausible;
    _curr_data.pedals_data.recvd_data.brake_is_implausible = pedals_msg.brake_implausible;
    
    _curr_data.pedals_data.recvd_data.mech_brake_is_active = pedals_msg.mechanical_brake_active;
    _curr_data.pedals_data.recvd_data.brake_is_pressed = pedals_msg.brake_pedal_active;
    _curr_data.pedals_data.recvd_data.accel_is_pressed = pedals_msg.accel_pedal_active;

    _curr_data.pedals_data.recvd_data.accel_percent = HYTECH_accel_pedal_ro_fromS(static_cast<float>(pedals_msg.accel_pedal_ro));
    _curr_data.pedals_data.recvd_data.brake_percent = HYTECH_brake_pedal_ro_fromS(static_cast<float>(pedals_msg.brake_pedal_ro));
    _curr_data.pedals_data.last_recv_millis = curr_millis;
    
    // TODO need to ensure that the pedals data timestamp is within a tollerance within the state machine
}
    