#ifndef __VCR_INTERFACE_H__
#define __VCR_INTERFACE_H__

#include "hytech_msgs.pb.h"

#include "SharedFirmwareTypes.h"

//make variables references to structs

struct DrivebrainData_VCR
{
	veh_vec<float> desired_rpms;
    veh_vec<float> torque_limit_nm;
    int64 prev_MCU_recv_millis;
};

struct ACUData_VCR
{
    std::array<std::array<std::optional<volt>, 12>, num_chips> voltages;
    std::array<celcius, 4 * num_chips> cell_temperatures;
    std::array<float, num_humidity_sensors> humidity;
    std::array<float, num_board_thermistors> board_temperatures;

    float min_voltage;
    float max_voltage;
    size_t min_voltage_cell_id;              // 0 - 125
    size_t max_voltage_cell_id;              // 0 - 125
    size_t max_board_temperature_segment_id; // 0 - 5
    size_t max_humidity_segment_id;          // 0 - 5
    size_t max_cell_temperature_cell_id;     // 0 - 47
    float total_voltage;
    float average_cell_temperature;
};

struct VCFData_VCR
{	
	PedalsSystemData_s pedals_system_data;
    FrontLoadCellsFiltered_s front_loadcells_filtered;
    FrontSusPotsFiltered_s front_suspots_filtered;
    SteeringFiltered_s steering_filtered;
    DashDisplayState_s dash_display;
}

class VCREthernetInterface 
{
public:
	VCREthernetInterface(DrivebrainData_VCR &drivebraindata, VCFData_VCR &vcfdata, ACUData_VCR &acudata): 
	_latest_db_data(drivebraindata), 
	_latest_acu_data(acudata),
	_latest_vcf_data(vcfdata) {
		//_latest_data.last_receive_time_millis = -1;
        
	//_latest_data.DB_prev>VCR_recv_millis
	};

hytech_msgs_VCRSystemData_s VCREthernetInterface::make_vcrsystemdata_msg(const VCRSystemData_s &shared_state);
hytech_msgs_VCRInterfaceData_s VCREthernetInterface::make_vcrinterfacedata_msg(const VCRInterfaceData_s &shared_state);
void VCREthernetInterface::receive_pb_msg_acu(const hytech_msgs_BMSData &msg_in);
void VCREthernetInterface::receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in);
void VCREthernetInterface::receive_pb_msg_vcf(const hytech_msgs_VCFSystemData &msg_in);

//VCRData_s get_latest_data() { return _latest_data; }
DrivebrainData_VCR get_latest_db_data() { return _latest_db_data; }
ACUData_VCR get_latest_acu_data() { return _latest_acu_data; }
VCFData_VCR get_latest_vcf_data() { return _latest_vcf_data; }



private:
    DrivebrainData_VCR &_latest_db_data = {};
	ACUData_VCR &_latest_acu_data = {};
	VCFData_VCR &_latest_vcf_data = {};
}

#endif //__VCR_INTERFACE_H__
	


