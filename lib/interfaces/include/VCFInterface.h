#ifndef __VCFINTERFACE_H__
#define __VCFINTERFACE_H__

#include "FlexCAN_T4.h"

#include "shared_types.h"
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

struct VCFCANInterfaceData_s {
    StampedPedalsSystemData_s stamped_pedals;
};

class VCFInterface {
public:

    VCFInterface() = delete;

    VCFInterface(unsigned long init_millis, unsigned long max_heartbeat_interval_ms)
    {
        _curr_data.stamped_pedals.last_heartbeat_time = init_millis;
        _max_heartbeat_interval_ms = max_heartbeat_interval_ms;
    };

    void receive_pedals_message(const CAN_message_t& msg, unsigned long curr_millis);
    
    VCFCANInterfaceData_s get_latest_data();

private:

    VCFCANInterfaceData_s _curr_data;

    unsigned long _max_heartbeat_interval_ms;
    
};

using VCFInterfaceInstance = etl::singleton<VCFInterface>;

#endif // __VCFINTERFACE_H__