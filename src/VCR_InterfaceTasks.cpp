#include "CANInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "SystemTimeInterface.h"
#include "VCR_InterfaceTasks.h"


/* From shared-systems-lib */
#include "Logger.h"

/* From shared_firmware_types library */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCR_Constants.h"
#include "BuzzerController.h"
#include "VCR_Globals.h"

// #include "VehicleStateMachine.h"

#include "DrivebrainInterface.h"

bool init_read_adc0_task()
{
    float scales[channels_within_mcp_adc] = {GLV_SENSE_SCALE, CURRENT_SENSE_SCALE, REFERENCE_SENSE_SCALE, RL_LOADCELL_SCALE, RR_LOADCELL_SCALE, RL_SUS_POT_SCALE, RR_SUS_POT_SCALE, 1}; 
    float offsets[channels_within_mcp_adc] = {GLV_SENSE_OFFSET, CURRENT_SENSE_OFFSET, REFERENCE_SENSE_OFFSET, RL_LOADCELL_OFFSET, RR_LOADCELL_OFFSET, RL_SUS_POT_OFFSET, RR_SUS_POT_OFFSET, 0};

    ADC0Instance::create(ADC0_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, scales, offsets);
     
    hal_printf("Initialized ADC0 at %d (millis)\n", sys_time::hal_millis());
    return true;
}

void run_read_adc0_task()
{

    ADC0Instance::instance().tick(); // why is it that the sample and convert functions are public in the mcp_adc interface? 
    vcr_data.interface_data.current_sensor_data.twentyfour_volt_sensor = ADC0Instance::instance().data.conversions[GLV_SENSE_CHANNEL].conversion;
    vcr_data.interface_data.current_sensor_data.current_sensor_unfiltered = ADC0Instance::instance().data.conversions[CURRENT_SENSE_CHANNEL].conversion;
    vcr_data.interface_data.current_sensor_data.current_refererence_unfiltered = ADC0Instance::instance().data.conversions[REFERENCE_SENSE_CHANNEL].conversion;
    vcr_data.interface_data.rear_loadcell_data.RL_loadcell_analog = ADC0Instance::instance().data.conversions[RL_LOADCELL_CHANNEL].conversion;
    vcr_data.interface_data.rear_loadcell_data.RR_loadcell_analog = ADC0Instance::instance().data.conversions[RR_LOADCELL_CHANNEL].conversion;
    vcr_data.interface_data.rear_suspot_data.RL_sus_pot_analog = ADC0Instance::instance().data.conversions[RL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    vcr_data.interface_data.rear_suspot_data.RR_sus_pot_analog = ADC0Instance::instance().data.conversions[RR_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    
    hal_printf("ADC0 reading 0 %d\n", ADC0Instance::instance().data.conversions[0].raw); // NOLINT
}

bool init_read_adc1_task()
{
    /* NOLINTBEGIN */ // Thermistor channels are for testing purposes only, the pin numbers 0-7 are acceptable "magic numbers".
    // Initialize all eight channels to scale = 1, offset = 0
    float scales[channels_within_mcp_adc] = {1, 1, 1, 1, 1, 1, 1, 1};
    float offsets[channels_within_mcp_adc] = {0, 0, 0, 0, 0, 0, 0, 0};
    /* NOLINTEND */

    ADC1Instance::create(ADC1_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, scales, offsets);

    hal_printf("Initialized ADC1 at %d (millis)\n", sys_time::hal_millis());
    return true;
}

void run_read_adc1_task()
{

    ADC1Instance::instance().tick();
    hal_printf("ADC1 reading 0 %d\n", ADC1Instance::instance().data.conversions[0].raw);
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

// CAN send tasks

// adds rear suspension and vcr status CAN messages to the sent on next mega loop run 
void handle_enqueue_suspension_CAN_data()
{
    DrivebrainInterfaceInstance::instance().handle_enqueue_suspension_CAN_data();
}

void handle_send_all_data()
{
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceImpl::telem_can_tx_buffer, &VCRCANInterfaceImpl::TELEM_CAN);
}