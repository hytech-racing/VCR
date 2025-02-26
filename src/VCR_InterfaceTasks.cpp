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
#include "IOExpander.h"

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
    adc_1.init();
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

void create_ioexpander(){
    IOExpanderInstance::create();
}

void read_ioexpander(){
    IOExpanderInstance::instance().read();

    vcr_data.interface_data.shutdown_sensing_data.bspd_is_ok = IOExpanderInstance::instance().getBit(0, 0);
    vcr_data.interface_data.shutdown_sensing_data.k_watchdog_relay = IOExpanderInstance::instance().getBit(0,1);
    vcr_data.interface_data.shutdown_sensing_data.watchdog_is_ok = IOExpanderInstance::instance().getBit(0,2);
    vcr_data.interface_data.shutdown_sensing_data.l_bms_relay = IOExpanderInstance::instance().getBit(0,3);
    vcr_data.interface_data.shutdown_sensing_data.bms_is_ok = IOExpanderInstance::instance().getBit(0,4);
    vcr_data.interface_data.shutdown_sensing_data.m_imd_relay = IOExpanderInstance::instance().getBit(0,5);
    vcr_data.interface_data.shutdown_sensing_data.imd_is_ok = IOExpanderInstance::instance().getBit(0,6);

    vcr_data.interface_data.ethernet_is_linked.acu_link = IOExpanderInstance::instance().getBit(1, 0);
    vcr_data.interface_data.ethernet_is_linked.drivebrain_link = IOExpanderInstance::instance().getBit(1, 1);
    vcr_data.interface_data.ethernet_is_linked.vcf_link = IOExpanderInstance::instance().getBit(1, 2);
    vcr_data.interface_data.ethernet_is_linked.teensy_link = IOExpanderInstance::instance().getBit(1, 3);
    vcr_data.interface_data.ethernet_is_linked.debug_link = IOExpanderInstance::instance().getBit(1, 4);
    vcr_data.interface_data.ethernet_is_linked.ubiquiti_link = IOExpanderInstance::instance().getBit(1, 5);
}
