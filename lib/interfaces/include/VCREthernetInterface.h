#ifndef __VCR_INTERFACE_H__
#define __VCR_INTERFACE_H__

#include "hytech_msgs.pb.h"
#include "SharedFirmwareTypes.h"

namespace VCREthernetInterface 
{
    hytech_msgs_VCRData_s make_vcr_data_msg(VCRData_s &shared_state);

    void receive_pb_msg_acu_core_data(const hytech_msgs_ACUCoreData_s &msg_in, VCRData_s &shared_state);
    void receive_pb_msg_acu_all_data(const hytech_msgs_ACUAllData_s &msg_in, VCRData_s &shared_state);
    void receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state);
    void receive_pb_msg_vcf(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state);

    void copy_inverter_data(InverterData_s &original, hytech_msgs_InverterData_s &destination);
}

#endif //__VCR_INTERFACE_H__
	


