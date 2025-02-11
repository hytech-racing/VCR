#ifndef __VCFINTERFACE_H__
#define __VCFINTERFACE_H__

#include "FlexCAN_T4.h"

#include "shared_types.h"
#include "SharedFirmwareTypes.h"
// this struct just contains the data we need from pedals 
// within VCR. the implaus check is done in the state machine.
struct PedalsStampedData_s : TimestampedData_s
{

    // float brake; // float between 0 and 1
    // float accel; // float between 0 and 1
    PedalsSystemData_s recvd_data;
};

struct LoadCellData_s : TimestampedData_s
{
    
};

struct VCFCANInterfaceData_s {
    PedalsStampedData_s pedals_data;
};

class VCFInterface {
public:
    VCFInterface() = default;

    void receive_pedals_message(const CAN_message_t& msg, unsigned long curr_millis);
    
    VCFCANInterfaceData_s get_latest_data();

private:

    VCFCANInterfaceData_s _curr_data;
    
};
#endif // __VCFINTERFACE_H__