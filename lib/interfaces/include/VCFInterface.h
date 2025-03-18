#ifndef __VCFINTERFACE_H__
#define __VCFINTERFACE_H__

#include "FlexCAN_T4.h"

#include "shared_types.h"
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

struct VCFCANInterfaceData_s {
    StampedPedalsSystemData_s stamped_pedals;
    DashInputState_s dash_input_state;
};

class VCFInterface {
public:

    VCFInterface() = delete;

    VCFInterface(unsigned long init_millis, unsigned long max_heartbeat_interval_ms) : _max_heartbeat_interval_ms(max_heartbeat_interval_ms)
    {
        _curr_data.stamped_pedals.last_recv_millis = 0;
        _curr_data.stamped_pedals.heartbeat_ok = false; // start out false
    };

    bool is_start_button_pressed() { return _curr_data.dash_input_state.start_btn_is_pressed; }
    bool is_brake_pressed() {return _curr_data.stamped_pedals.pedals_data.brake_is_pressed; }
    bool is_pedals_heartbeat_ok() {return _curr_data.stamped_pedals.heartbeat_ok; }
    void reset_pedals_heartbeat();
    
    void receive_pedals_message(const CAN_message_t& msg, unsigned long curr_millis);
    
    VCFCANInterfaceData_s get_latest_data();

    void send_buzzer_start_message();

private:

    VCFCANInterfaceData_s _curr_data;

    unsigned long _max_heartbeat_interval_ms;
    
};

using VCFInterfaceInstance = etl::singleton<VCFInterface>;

#endif // __VCFINTERFACE_H__