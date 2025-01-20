#ifndef __VCR_INTERFACE_H__
#define __VCR_INTERFACE_H__

#include "hytech_msgs.pb.h"

// #include equivalent of "DrivebrainData.h"
#include "SharedFirmwareTypes.h"

class VCREthernetInterface 
{
public:
	VCREthernetInterface() {
		//_latest_data.last_receive_time_millis = -1;
        
	//_latest_data.DB_prev>VCR_recv_millis
	};

//void VCREthernetInterface::receive_pb_msg(const hytech_msgs_VCRSystemData_s &msg_in, unsigned long curr_millis);
hytech_msgs_VCRSystemData_s VCREthernetInterface::make_vcrsystemdata_msg(const VCRSystemData_s &shared_state);
//hytech_msgs_VCRSystemData_s make_vcr_msg(const VCRSystemData_s &shared_state);
hytech_msgs_VCRInterfaceData_s VCREthernetInterface::make_vcrinterfacedata_msg(const VCRInterfaceData_s &shared_state);
void VCREthernetInterface::receive_pb_msg_acu(const hytech_msgs_BMSData &msg_in);
void VCREthernetInterface::receive_pb_msg_db(const hytech_msgs_BMSData &msg_in);
void VCREthernetInterface::receive_pb_msg_vcf(const hytech_msgs_BMSData &msg_in);
VCRData_s get_latest_data() { return _latest_data; }



private:
    VCRData_s _latest_data = {};



}

#endif //__VCR_INTERFACE_H__
	


