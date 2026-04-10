#ifndef VCR_ETHERNET_INTERFACE_H
#define VCR_ETHERNET_INTERFACE_H

#include "hytech_msgs.pb.h"
#include "SharedFirmwareTypes.h"
#include "ADCInterface.h"
#include "ProtobufMsgInterface.h"

namespace VCREthernetInterface
{
    /**
     * Function to transform our struct from shared_data_types into the protoc struct hytech_msgs_VCRData_s.
     *
     * @param
     * @return A populated instance of the outgoing protoc struct.
     */
    hytech_msgs_VCRData_s makeVCRDataMsg(
        const ADCInterface &ADCInterfaceInstance,
        DrivetrainDynamicReport_s &DrivetrainData,
        VCFHeartbeatData_s &VCF_Heartbeat_Data,
        VehicleState_e &vehicle_state_machine_state,
        DrivetrainState_e drivetrain_state_machine_state,
        veh_vec<InverterData_s> &InverterData,
        DrivebrainControllerStatus_s &DB_Controller_Status,
        TorqueControllerMuxStatus_s &tc_mux_status,
        CurrentSensorData_s &current_sensor_data);

    /**
     * Function to take a populated protoc struct from the drivebrain and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receivePbMsgDB(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    /**
     * Function to take a populated protoc struct from VCF and update the VCR state.
     *
     * @param msg_in A reference to a populated protoc struct.
     * @param shared_state A reference to the VCR state.
     *
     * @post After this function completes, shared_state will have updated contents of ACUAllData.
     */
    void receivePbMsgVCF(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis);

    /**
     * Helper function to copy an instance of InverterData_s to the protoc struct hytech_msgs_InverterData_s.
     * @param original A populated instance of the InverterData_s defined in shared firmware types.
     * @param destination The destination protoc struct.
     * @post The destination struct will be populated with the data from original.
     */
    void copyInverterData(const InverterData_s &original, hytech_msgs_InverterData_s &destination);

    /**
     * Helper function to copy veh_vec data.
     *
     * @param original A populated instance of a veh_vec.
     * @param destination A reference to an unpopulated instance of veh_vec.
     * @post The destination veh_vec will be populated with the data from the original.
     */
    template <typename from_T, typename to_T>
    void copyVehVecMembers(const from_T& from, to_T& to)
    {
      to.FL = from.FL;
      to.FR = from.FR;
      to.RL = from.RL;
      to.RR = from.RR;
    }
}

#endif /* VCR_ETHERNET_INTERFACE_H */