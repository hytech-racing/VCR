#include "SystemTimeInterface.h"
#include "VCR_Tasks.h"


/* From shared-systems-lib */
#include "Logger.h"

/* From shared_firmware_types library */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCR_Constants.h"
#include "BuzzerController.h"
#include "VCR_Globals.h"
#include "VehicleStateMachine.h"

bool init_read_adc0_task()
{
    adc_0.init();
    adc_0.setChannelScaleAndOffset(GLV_SENSE_CHANNEL, GLV_SENSE_SCALE, GLV_SENSE_OFFSET);
    adc_0.setChannelScaleAndOffset(CURRENT_SENSE_CHANNEL, CURRENT_SENSE_SCALE, CURRENT_SENSE_OFFSET);
    adc_0.setChannelScaleAndOffset(REFERENCE_SENSE_CHANNEL, REFERENCE_SENSE_SCALE, REFERENCE_SENSE_OFFSET);
    adc_0.setChannelScaleAndOffset(RL_LOADCELL_CHANNEL, RL_LOADCELL_SCALE, RL_LOADCELL_OFFSET);
    adc_0.setChannelScaleAndOffset(RR_LOADCELL_CHANNEL, RL_LOADCELL_SCALE, RL_LOADCELL_OFFSET);
    adc_0.setChannelScaleAndOffset(RL_SUS_POT_CHANNEL, RL_SUS_POT_SCALE, RL_SUS_POT_OFFSET);
    adc_0.setChannelScaleAndOffset(RR_SUS_POT_CHANNEL, RR_SUS_POT_SCALE, RR_SUS_POT_OFFSET);
    
    hal_printf("Initialized ADC0 at %d (millis)\n", sys_time::hal_millis()); // NOLINT
    return true;
}

void run_read_adc0_task()
{

    adc_0.sample(); // Samples all eight channels.
    adc_0.convert(); // Converts all eight channels.

    vcr_data.interface_data.current_sensor_data.twentyfour_volt_sensor = adc_0.data.conversions[GLV_SENSE_CHANNEL].conversion;
    vcr_data.interface_data.current_sensor_data.current_sensor_unfiltered = adc_0.data.conversions[CURRENT_SENSE_CHANNEL].conversion;
    vcr_data.interface_data.current_sensor_data.current_refererence_unfiltered = adc_0.data.conversions[REFERENCE_SENSE_CHANNEL].conversion;
    vcr_data.interface_data.rear_loadcell_data.RL_loadcell_analog = adc_0.data.conversions[RL_LOADCELL_CHANNEL].conversion;
    vcr_data.interface_data.rear_loadcell_data.RR_loadcell_analog = adc_0.data.conversions[RR_LOADCELL_CHANNEL].conversion;
    vcr_data.interface_data.rear_suspot_data.RL_sus_pot_analog = adc_0.data.conversions[RL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    vcr_data.interface_data.rear_suspot_data.RR_sus_pot_analog = adc_0.data.conversions[RR_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    // Serial.println("yo");
    hal_printf("ADC0 reading 0 %d\n", adc_0.data.conversions[0].raw); // NOLINT
}

bool init_read_adc1_task()
{
    /* NOLINTBEGIN */ // Thermistor channels are for testing purposes only, the pin numbers 0-7 are acceptable "magic numbers".
    // Initialize all eight channels to scale = 1, offset = 0
    adc_0.init();
    adc_1.setChannelScaleAndOffset(0, 1, 0);
    adc_1.setChannelScaleAndOffset(1, 1, 0);
    adc_1.setChannelScaleAndOffset(2, 1, 0);
    adc_1.setChannelScaleAndOffset(3, 1, 0);
    adc_1.setChannelScaleAndOffset(4, 1, 0);
    adc_1.setChannelScaleAndOffset(5, 1, 0);
    adc_1.setChannelScaleAndOffset(6, 1, 0);
    adc_1.setChannelScaleAndOffset(7, 1, 0);
    /* NOLINTEND */

    hal_printf("Initialized ADC1 at %d (millis)\n", sys_time::hal_millis()); // NOLINT
    return true;
}

void run_read_adc1_task()
{

    adc_1.sample(); // Samples all eight channels.
    adc_1.convert(); // Converts all eight channels.
    hal_printf("ADC1 reading 0 %d\n", adc_1.data.conversions[0].raw); // NOLINT
}

void run_update_buzzer_controller_task()
{
    vcr_data.system_data.buzzer_is_active = BuzzerController::getInstance().buzzer_is_active(sys_time::hal_millis()); //NOLINT

}

void create_watchdog()
{
    WatchdogInstance::create(default_system_params::KICK_INTERVAL_MS); // this has issues for some reason with clang-tidy // NOLINT
}

void run_kick_watchdog()
{
    digitalWrite(WATCHDOG_PIN, WatchdogInstance::instance().get_watchdog_state(sys_time::hal_millis()));
}


bool init_I2C(){
    mcp.init();
    mcp.portMode(MCP23017Port::A, 0b01111111);
    mcp.portMode(MCP23017Port::B, 0b01111111);

    mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
    mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

    // GPIO_B reflects the same logic as the input pins state
    mcp.writeRegister(MCP23017Register::IPOL_A, 0x00);
    mcp.writeRegister(MCP23017Register::IPOL_B, 0x00);

    return true;
}

bool get_bit(u_int16_t data, int bit){
    return (data>>bit)&1;
}

void run_I2C(){
    
    uint16_t temp = mcp.read();
    //GPA
    vcr_data.interface_data.shutdown_sensing_data.bspd_is_ok = get_bit(temp, 0);
    vcr_data.interface_data.shutdown_sensing_data.k_watchdog_relay = get_bit(temp, 1);
    vcr_data.interface_data.shutdown_sensing_data.watchdog_is_ok = get_bit(temp, 2);
    vcr_data.interface_data.shutdown_sensing_data.l_bms_relay = get_bit(temp, 3);
    vcr_data.interface_data.shutdown_sensing_data.bms_is_ok = get_bit(temp, 4);
    vcr_data.interface_data.shutdown_sensing_data.m_imd_relay = get_bit(temp, 5);
    vcr_data.interface_data.shutdown_sensing_data.imd_is_ok = get_bit(temp, 6);
    // vcr_data.shutdown_sensing_data.i_shutdown_in = get_bit(temp, 7); //GPA7 unusable for input

    //GPB
    vcr_data.interface_data.ethernet_is_linked.acu_link = get_bit(temp, 8);
    vcr_data.interface_data.ethernet_is_linked.drivebrain_link = get_bit(temp, 9);
    vcr_data.interface_data.ethernet_is_linked.vcf_link = get_bit(temp, 10);
    vcr_data.interface_data.ethernet_is_linked.teensy_link = get_bit(temp, 11);
    vcr_data.interface_data.ethernet_is_linked.debug_link = get_bit(temp, 12);
    vcr_data.interface_data.ethernet_is_linked.ubiquiti_link = get_bit(temp, 13);
    // vcr_data.shutdown_sensing_data.j_bspd_relay = get_bit(temp, 15); //GPB7 unusable for input
}
