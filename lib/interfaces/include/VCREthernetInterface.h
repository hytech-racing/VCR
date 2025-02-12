#ifndef VCR_ETHERNET_INTERFACE_H
#define VCR_ETHERNET_INTERFACE_H

#include "hytech_msgs.pb.h"
#include "SharedFirmwareTypes.h"

namespace VCREthernetInterface 
{
    /**
     * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_VCRData_s.
     * 
     * @param shared_state The current VCR state, which includes both interface and system data.
     * @return A populated instance of the outgoing protoc struct.
     */
    hytech_msgs_VCRData_s make_vcr_data_msg(VCRData_s &shared_state);

    /**
     * Function to take a populated protoc struct from ACU and update the VCR state.
     * 
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     * 
     * @post After this function completes, shared_state will have updated contents of ACUCoreData.
     */
    void receive_pb_msg_acu_core_data(const hytech_msgs_ACUCoreData_s &msg_in, VCRData_s &shared_state);

    /**
     * Function to take a populated protoc struct from ACU and update the VCR state.
     * 
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     * 
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receive_pb_msg_acu_all_data(const hytech_msgs_ACUAllData_s &msg_in, VCRData_s &shared_state);

    /**
     * Function to take a populated protoc struct from the drivebrain and update the VCR state.
     * 
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     * 
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state);

    /**
     * Function to take a populated protoc struct from VCF and update the VCR state.
     * 
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     * 
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receive_pb_msg_vcf(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state);

    /**
     * Helper function to copy an instance of InverterData_s to the protoc struct hytech_msgs_InverterData_s.
     * @param original A populated instance of the InverterData_s defined in shared firmware types.
     * @param destination The destination protoc struct.
     * @post The destination struct will be populated with the data from original.s
     */
    void copy_inverter_data(InverterData_s &original, hytech_msgs_InverterData_s &destination);
}

#endif /* VCR_ETHERNET_INTERFACE_H */