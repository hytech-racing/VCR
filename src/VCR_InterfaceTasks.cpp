#include "CANInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "SystemTimeInterface.h"
#include "VCR_InterfaceTasks.h"
 
 
/* From shared-systems-lib */
#include "Logger.h"
 
/* From shared_firmware_types library */
#include "SharedFirmwareTypes.h"
 
/* Local includes */
#include "VCREthernetInterface.h"
#include "VCR_Constants.h"
#include "BuzzerController.h"
#include "VCR_Globals.h"
 
#include "AMSSystem.h"
#include "DrivebrainInterface.h"
 
bool init_read_adc0_task()
{
    SPI.begin();
    float scales[channels_within_mcp_adc] = {1, 1, 1, 1, 1, 1, 1, 1};
    float offsets[channels_within_mcp_adc] = {0, 0, 0, 0, 0, 0, 0, 0};
    // ADC2Instance::create(ADC2_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, scales, offsets);
    ADC1Instance::create(ADC0_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, scales, offsets);
     
    hal_printf("Initialized ADC0 at %d (millis)\n", sys_time::hal_millis()); // NOLINT
    return true;
}
 
void run_read_adc0_task()
{
    ADC1Instance::instance().tick();
    hal_printf("ADC0 reading 0 %d\n", ADC1Instance::instance().data.conversions[0].raw);
    hal_printf("ADC0 reading 1 %d\n", ADC1Instance::instance().data.conversions[1].raw);
    hal_printf("ADC0 reading 2 %d\n", ADC1Instance::instance().data.conversions[2].raw);
    hal_printf("ADC0 reading 3 %d\n", ADC1Instance::instance().data.conversions[3].raw);
    hal_printf("ADC0 reading 4 %d\n", ADC1Instance::instance().data.conversions[4].raw);
    hal_printf("ADC0 reading 5 %d\n", ADC1Instance::instance().data.conversions[5].raw);
 
    hal_printf("\n\n");
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
 
 
 
bool init_ams_system_task()
{
    AMSSystemInstance::create(HEARTBEAT_INTERVAL_MS); // NOLINT
    pinMode(SOFTWARE_OK_PIN, OUTPUT);
    return true;
}
 
void run_ams_system_task()
{
    AMSSystemInstance::instance().update_ams_system(sys_time::hal_millis(), vcr_data);
    digitalWrite(SOFTWARE_OK_PIN, vcr_data.system_data.ams_data.ams_ok);
}
 
 
 
bool create_watchdog()
{
    WatchdogInstance::create(default_system_params::KICK_INTERVAL_MS); // this has issues for some reason with clang-tidy // NOLINT
    pinMode(WATCHDOG_PIN, OUTPUT);
    return true;
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
 
void handle_send_VCR_ethernet_data()
{
    DrivebrainInterfaceInstance::instance().handle_send_ethernet_data(VCREthernetInterface::make_vcr_data_msg(vcr_data));
}
 
void handle_send_all_data()
{
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceImpl::telem_can_tx_buffer, &VCRCANInterfaceImpl::TELEM_CAN);
}