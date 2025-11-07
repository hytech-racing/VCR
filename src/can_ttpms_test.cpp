#include <Arduino.h>

// Core system interfaces for CAN communication and sensor data
#include "CANInterface.h"
#include "TTPMSInterface.h"
#include "InverterInterface.h"
#include "VCFInterface.h"
#include "ACUInterface.h"
#include "DrivebrainInterface.h"
#include "hytech.h"
#include "VCRCANInterfaceImpl.h"
#include "etl/singleton.h"
#include "VCR_Constants.h"


FlexCAN_T4<CAN2> MAIN_CAN;

// Initialize inverter interfaces for each wheel with their CAN IDs
InverterInterface fl_inverter_int(INV1_CONTROL_WORD_CANID, INV1_CONTROL_INPUT_CANID, INV1_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE}); //NOLINT
InverterInterface fr_inverter_int(INV2_CONTROL_WORD_CANID, INV2_CONTROL_INPUT_CANID, INV2_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE}); //NOLINT
InverterInterface rl_inverter_int(INV3_CONTROL_WORD_CANID, INV3_CONTROL_INPUT_CANID, INV3_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE}); //NOLINT
InverterInterface rr_inverter_int(INV4_CONTROL_WORD_CANID, INV4_CONTROL_INPUT_CANID, INV4_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE}); //NOLINT

// Create TTPMS sensor interfaces for each wheel
TTPMSInterface fl_ttpms_int;
TTPMSInterface fr_ttpms_int;
TTPMSInterface rl_ttpms_int;
TTPMSInterface rr_ttpms_int;


void on_recv(const CAN_message_t &msg)
{
    // Forward message to CAN interface implementation
    VCRCANInterfaceImpl::vcr_CAN_recv(CANInterfacesInstance::instance(), msg, msg.timestamp);

    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX); Serial.print(" ");
    Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    Serial.print("  LEN: "); Serial.print(msg.len); Serial.print(" ");
    Serial.print("  DATA: ");    
        for ( uint8_t i = 0; i < 8; i++ ) {
            Serial.print(msg.buf[i], HEX); Serial.print(" ");
        }        
    Serial.println();

    TTPMSSensorData_s data = rl_ttpms_int.getLatestSensorData();
    Serial.println("=== TTPMSSensorData ===");
    Serial.print("Serial #: "); Serial.print(data.serial_number);
    Serial.print(" Battery (mV): "); Serial.print(data.battery_voltage);
    Serial.print(" Pressure: "); Serial.print(data.pressure);
    Serial.print(" Gauge Pressure: "); Serial.println(data.gauge_pressure);

    Serial.println("Infrared Temps: ");
    for (int i = 0; i < 16; i++) {
        Serial.print("  ["); Serial.print(i); Serial.print("] = ");
        Serial.print(data.infrared_temp[i]);
    }

    Serial.println();
    Serial.print("Transmission Count: "); Serial.print(data.transmission_count);
    Serial.print(" RSSI: "); Serial.print(data.rssi);
    Serial.print(" Sensor Temp: "); Serial.print(data.sensor_temperature);
    Serial.print(" Node ID: "); Serial.println(data.sensor_node_id);
    Serial.println("=======================");

    Serial.print("  TS: "); Serial.println(msg.timestamp);
}
    

void setup()
{
    Serial.begin(115200);  
    handle_CAN_setup(MAIN_CAN, 500000, &on_recv);  //500 kbps baudrate

    // Create singleton instance with all vehicle interfaces
    CANInterfacesInstance::create(
    VCFInterfaceInstance::instance(),
    ACUInterfaceInstance::instance(),
    DrivebrainInterfaceInstance::instance(), 
    fl_inverter_int,
    fr_inverter_int,
    rl_inverter_int,
    rr_inverter_int,
    fl_ttpms_int,
    fr_ttpms_int,
    rl_ttpms_int,
    rr_ttpms_int
    );
}

// Main loop - CAN messages are handled by interrupt in on_recv
void loop()
{    
    delay(1000);  
}
