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
#include "AMSSystem.h"


bool init_bundle(){

    float adc0_scales[channels_within_mcp_adc], adc0_offsets[channels_within_mcp_adc], adc1_scales[channels_within_mcp_adc], adc1_offsets[channels_within_mcp_adc]; 
    adc0_scales[GLV_SENSE_CHANNEL] = GLV_SENSE_SCALE;
    adc0_offsets[GLV_SENSE_CHANNEL] = GLV_SENSE_OFFSET;
    adc0_scales[CURRENT_SENSE_CHANNEL] = CURRENT_SENSE_SCALE;
    adc0_offsets[CURRENT_SENSE_CHANNEL] = CURRENT_SENSE_OFFSET;
    adc0_scales[REFERENCE_SENSE_CHANNEL] = REFERENCE_SENSE_SCALE;
    adc0_offsets[REFERENCE_SENSE_CHANNEL] = REFERENCE_SENSE_OFFSET;
    adc0_scales[RL_LOADCELL_CHANNEL] = RL_LOADCELL_SCALE;
    adc0_offsets[RL_LOADCELL_CHANNEL] = RL_LOADCELL_OFFSET;
    adc0_scales[RR_LOADCELL_CHANNEL] = RR_LOADCELL_SCALE;
    adc0_offsets[RR_LOADCELL_CHANNEL] = RR_LOADCELL_OFFSET;
    adc0_scales[RL_SUS_POT_CHANNEL] = RL_SUS_POT_SCALE;
    adc0_offsets[RL_SUS_POT_CHANNEL] = RL_SUS_POT_OFFSET;
    adc0_scales[RR_SUS_POT_CHANNEL] = RR_SUS_POT_SCALE;
    adc0_offsets[RR_SUS_POT_CHANNEL] = RR_SUS_POT_OFFSET;

    adc1_scales[THERMISTOR_0] = THERMISTOR_0_SCALE;
    adc1_offsets[THERMISTOR_0] = THERMISTOR_0_OFFSET;
    adc1_scales[THERMISTOR_1] = THERMISTOR_1_SCALE;
    adc1_offsets[THERMISTOR_1] = THERMISTOR_1_OFFSET;
    adc1_scales[THERMISTOR_2] = THERMISTOR_2_SCALE;
    adc1_offsets[THERMISTOR_2] = THERMISTOR_2_OFFSET;
    adc1_scales[THERMISTOR_3] = THERMISTOR_3_SCALE;
    adc1_offsets[THERMISTOR_3] = THERMISTOR_3_OFFSET;
    adc1_scales[THERMISTOR_4] = THERMISTOR_4_SCALE;
    adc1_offsets[THERMISTOR_4] = THERMISTOR_4_OFFSET;
    adc1_scales[THERMISTOR_5] = THERMISTOR_5_SCALE;
    adc1_offsets[THERMISTOR_5] = THERMISTOR_5_OFFSET;
    adc1_scales[THERMISTOR_6] = THERMISTOR_6_SCALE;
    adc1_offsets[THERMISTOR_6] = THERMISTOR_6_OFFSET;
    adc1_scales[THERMISTOR_7] = THERMISTOR_7_SCALE;
    adc1_offsets[THERMISTOR_7] = THERMISTOR_7_OFFSET;

    ADCSingletonInstance::create(adc0_scales, adc0_offsets, adc1_scales, adc1_offsets);

    return true;
}

void run_read_adc0_task()
{

    ADCSingletonInstance::instance().adc0.tick();

    vcr_data.interface_data.current_sensor_data.twentyfour_volt_sensor = 
        ADCSingletonInstance::instance().adc0.data.conversions[GLV_SENSE_CHANNEL].conversion;

    vcr_data.interface_data.current_sensor_data.current_sensor_unfiltered = 
        ADCSingletonInstance::instance().adc0.data.conversions[CURRENT_SENSE_CHANNEL].conversion;

    vcr_data.interface_data.current_sensor_data.current_refererence_unfiltered = 
        ADCSingletonInstance::instance().adc0.data.conversions[REFERENCE_SENSE_CHANNEL].conversion;

    vcr_data.interface_data.rear_loadcell_data.RL_loadcell_analog = 
        ADCSingletonInstance::instance().adc0.data.conversions[RL_LOADCELL_CHANNEL].conversion;

    vcr_data.interface_data.rear_loadcell_data.RR_loadcell_analog = 
        ADCSingletonInstance::instance().adc0.data.conversions[RR_LOADCELL_CHANNEL].conversion;

    vcr_data.interface_data.rear_suspot_data.RL_sus_pot_analog = 
        ADCSingletonInstance::instance().adc0.data.conversions[RL_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    vcr_data.interface_data.rear_suspot_data.RR_sus_pot_analog = 
        ADCSingletonInstance::instance().adc0.data.conversions[RR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    // Serial.println("yo");
    hal_printf("ADC0 reading 0 %d\n", 
        ADCSingletonInstance::instance().adc0.data.conversions[0].raw); // NOLINT
}


void run_read_adc1_task()
{

    ADCSingletonInstance::instance().adc1.tick();

    hal_printf("ADC1 reading 0 %d\n", ADCSingletonInstance::instance().adc1.data.conversions[0].raw); // NOLINT
}

void run_update_buzzer_controller_task()
{
    vcr_data.system_data.buzzer_is_active = BuzzerController::getInstance().buzzer_is_active(sys_time::hal_millis()); //NOLINT
}



bool init_ams_system_task()
{
    AMSSystemInstance::create(HEARTBEAT_INTERVAL_MS); // NOLINT 
    pinMode(SOFTWARE_OK_PIN, OUTPUT);
    return true;
}

bool run_ams_system_task()
{
    AMSSystemInstance::instance().update_ams_system(sys_time::hal_millis(), vcr_data);
    digitalWrite(SOFTWARE_OK_PIN, vcr_data.system_data.ams_data.ams_ok);
    return true;
}



void create_watchdog()
{
    WatchdogInstance::create(default_system_params::KICK_INTERVAL_MS); // this has issues for some reason with clang-tidy // NOLINT
    pinMode(WATCHDOG_PIN, OUTPUT);
}

void run_kick_watchdog()
{
    digitalWrite(WATCHDOG_PIN, WatchdogInstance::instance().get_watchdog_state(sys_time::hal_millis()));
}
