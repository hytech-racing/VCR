#ifdef ARDUINO

#include <Arduino.h>
#endif

 // NOLINT for TaskScheduler

/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* Arduino specific upstream Libraries */
#include "QNEthernet.h"

#define _TASK_MICRO_RES // NOLINT

/* Local includes */
#include "TorqueControllerMux.hpp"
#include "VCFInterface.h"
#include "VCREthernetInterface.h"

#include "FlexCAN_T4.h"
#include "VCRCANInterfaceImpl.h"

#include "etl/singleton.h"

#include "DrivebrainInterface.h"
#include "InverterInterface.h"
#include "DrivetrainSystem.h"
#include "VCR_SystemTasks.h"


// has to be included here as the define is only defined for source files in the implementation
// not in the library folder (makes sense)
#include "device_fw_version.h"  // from pio-git-hash


#include "EthernetAddressDefs.h"
#include "VehicleStateMachine.h"


FlexCAN_Type<CAN3> TELEM_CAN;
FlexCAN_Type<CAN2> INVERTER_CAN;

/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP vcr_data_socket;
qindesign::network::EthernetUDP vcf_data_socket;

VCRData_s vcr_data = {};

void setup() {
    Serial.begin(115200);

    EthernetIPDefsInstance::create();
    uint8_t mac[6];
    qindesign::network::Ethernet.macAddress(mac);
    qindesign::network::Ethernet.begin(mac, EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);

    vcr_data_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    vcf_data_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
}

void loop() { 
    if (millis() % 1000 < 500) {
        etl::optional<hytech_msgs_VCFData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCFData_s_size, hytech_msgs_VCFData_s>(&vcf_data_socket, &hytech_msgs_VCFData_s_msg);
        if (protoc_struct) {
            VCREthernetInterface::receive_pb_msg_vcf(protoc_struct.value(), vcr_data, millis());
            
            Serial.println("Received bytes: (protoc struct value)");
            char *string_ptr = (char*) &(protoc_struct.value());
            uint32_t kk = sizeof(protoc_struct.value());
            while(kk--)
                Serial.printf("%02X ", *string_ptr++);
            Serial.println("");

            Serial.println(protoc_struct.value().pedals_system_data.regen_percent);
            Serial.println(vcr_data.interface_data.recvd_pedals_data.pedals_data.regen_percent);
        } else {
            Serial.printf("Did not receive VCF message!\n");
        }

        vcr_data.interface_data.rear_loadcell_data.RL_loadcell_analog += 1;
        hytech_msgs_VCRData_s send_protoc_struct = VCREthernetInterface::make_vcr_data_msg(vcr_data);

        Serial.println("Send bytes: (protoc struct value)");
        char *string_ptr = (char*) &(send_protoc_struct);
        uint32_t kk = sizeof(send_protoc_struct);
        while(kk--)
            Serial.printf("%02X ", *string_ptr++);
        Serial.println("");

        handle_ethernet_socket_send_pb<hytech_msgs_VCRData_s_size>(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().VCRData_port,
            &vcr_data_socket, send_protoc_struct, hytech_msgs_VCRData_s_fields);

        delay(500);
    }
}
