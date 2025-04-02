#include "VCREthernetTCPClient.h"
#include "EthernetAddressDefs.h"

VCREthernetTCPClient::VCREthernetTCPClient() {
}

void VCREthernetTCPClient::begin() {
    Serial.begin(BAUD_RATE);
    while (!Serial) {}

    EthernetIPDefs_s& ipDeps = EthernetIPDefsInstance::instance();

    Ethernet.begin(ipDeps.vcr_ip, ipDeps.car_subnet, ipDeps.default_gateway, ipDeps.default_gateway);

    Serial.print("IP address: ");
    Serial.println(Ethernet.localIP());
}

void VCREthernetTCPClient::sendAndReceive() {
    EthernetIPDefs_s& ipDeps = EthernetIPDefsInstance::instance();

    if (!client.connected()) {
        Serial.print("Connecting to server at ");
        Serial.print(ipDeps.acu_ip);
        Serial.print(":");
        Serial.println(ipDeps.ACUAllData_port);
        if (client.connect(ipDeps.acu_ip, ipDeps.ACUAllData_port)) {
            Serial.println("Connected to server");
        } else {
            Serial.println("Connection failed");
        }
    }

    client.print("Message from client (");
    client.print(Ethernet.localIP());
    client.println(")");
    Serial.print("Message sent from client at ");
    Serial.println(Ethernet.localIP());

    if (client.available()) {
        String response = client.readStringUntil('\n');
        Serial.print("Response from server: ");
        Serial.println(response);
    }
}