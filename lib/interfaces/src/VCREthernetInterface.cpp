#include "VCR_Ethernet_Interface.h"
#include "SharedFirmwareTypes.h"
#include <Arduino.h>

hytech_msgs_VCRSystemData_s VCR_Ethernet_Interface::make_vcr_msg(const VCRSystemData_s &shared_state)
{
	hytech_msgs_VCRSystemData_s out;

    //RearLoadCellsFiltered_s
    out.rear_loadcells_filtered.RL_loadcell_filtered_pounds;
    out.rear_loadcells_filtered.RR_loadcell_filtered_pounds;
    out.rear_loadcells_filtered.rear_loadcell_FIR_is_saturated;

    out.rear_loadcells_filtered = {shared_state.rear_loadcells_filtered.RL_loadcell_filtered_pounds,
                                   shared_state.rear_loadcells_filtered.RR_loadcell_filtered_pounds,
                                   shared_state.rear_loadcells_filtered.rear_loadcell_FIR_is_saturated};

    
    //FrontLoadCellsFiltered_s
    out.front_loadcells_filtered.FL_loadcell_filtered_pounds;
    out.front_loadcells_filtered.FR_loadcell_filtered_pounds;
    out.front_loadcells_filtered.front_loadcell_FIR_is_saturated;

    out.front_loadcells_filtered = {shared_state.front_loadcells_filtered.FL_loadcell_filtered_pounds,
                                    shared_state.front_loadcells_filtered.FR_loadcell_filtered_pounds,
                                    shared_state.front_loadcells_filtered.front_loadcell_FIR_is_saturated};

    

    //FrontSusPotsFiltered_s
    out.front_suspots_filtered.FL_sus_pot_filtered_analog;
    out.front_suspots_filtered.FR_sus_pot_filtered_analog;
    out.front_suspots_filtered.front_loadcell_FIR_is_saturated;

    out.front_suspots_filtered = {shared_state.front_suspots_filtered.FL_sus_pot_filtered_analog,
                                  shared_state.front_suspots_filtered.FR_sus_pot_filtered_analog,
                                  shared_state.front_suspots_filtered.front_loadcell_FIR_is_saturated};


    //SteeringFiltered_s
    out.steering_filtered.steering_filtered_degrees;
    out.steering_filtered.steering_FIR_is_saturated;

    out.steering_filtered = {shared_state.steering_filtered.steering_filtered_degrees,
                             shared_state.steering_filtered.steering_FIR_is_saturated};

    
    //DashDisplayState_s
    out.dash_display.dash_data;
    out.dash_display = {shared_state.dash_display.dash_data};

    return out;

}

void VCREthernetInterface::receive_pb_msg(const hytech_msgs_VCRSystemData_s &msg_in, unsigned long curr_millis)
{

}
	



