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
    VCFInterface() = default;

    void receive_pedals_message(const CAN_message_t& msg, unsigned long curr_millis);
    
    VCFCANInterfaceData_s get_latest_data();

private:

    VCFCANInterfaceData_s _curr_data;
    
};

using VCFInterfaceInstance = etl::singleton<VCFInterface>;

#endif // __VCFINTERFACE_H__