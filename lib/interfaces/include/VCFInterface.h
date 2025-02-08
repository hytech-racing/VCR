#ifndef __VCFINTERFACE_H__
#define __VCFINTERFACE_H__

#include "FlexCAN_T4.h"

#include "shared_types.h"

// this struct just contains the data we need from pedals 
// within VCR. the implaus check is done in the state machine.
struct PedalsData_s : TimestampedData_s
{
    bool implausibility_exceeded_duration;
    float brake; // float between 0 and 1
    float accel; // float between 0 and 1
};

struct LoadCellData_s : TimestampedData_s
{
    
};

struct VCFInterfaceData_s {
    PedalsData_s pedals_data;
};

class VCFInterface {
public:
    VCFInterface() = default;

    void receive_pedals_message(const CAN_message_t& msg, unsigned long curr_millis);
    
    VCFInterfaceData_s get_latest_data();

private:

    VCFInterfaceData_s _curr_data;
    
};
#endif // __VCFINTERFACE_H__