#include <Arduino.h>
#include <cstdint>
#include <QNEthernet.h>
#include "VCREthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "hytech_msgs.pb.h"

#include <array>
#include <cstring>

#include "ProtobufMsgInterface.h"




//combine recv and receive
//call ethernet socket receive --> returns protobuf c struct



using namespace qindesign::network;
EthernetUDP socket; 
//EthernetUDP recv_socket; 


const IPAddress default_VCR_ip(192, 168, 1, 30); //(for now) sender
const IPAddress receive_ip(192, 168, 1, 31); // receiver
const IPAddress default_dns(192, 168, 1, 1);
const IPAddress default_gateway(192, 168, 1, 1);
const IPAddress car_subnet(255, 255, 255, 0);
uint16_t port1 = 4444;
uint16_t port2 = 5555;
hytech_msgs_VCRData_s msg;
VCRData_s vcr_state;

// uint8_t default_MCU_MAC_address[6] = 
//     {0x04, 0xe9, 0xe5, 0x10, 0x1f, 0x22};

void init_ethernet_device()
{
    Ethernet.begin(default_VCR_ip, default_dns, default_gateway, car_subnet);
    socket.begin(4444);
    //recv_socket.begin(5555);
}

void test_send()
{
    //send VCRData_s
    VCREthernetInterface::make_vcr_data_msg(vcr_state);
    if (handle_ethernet_socket_send_pb<hytech_msgs_VCRData_s, 4096>(receive_ip, 5555, &socket, &msg, hytech_msgs_VCRData_s_msg)   {
        Serial.println("Sent");
    } else {
        Serial.println("Failed");
    }

}

void setup()
{
    init_ethernet_device();
}

void loop()
{
    test_send();
    // Serial.println("loopin");
}