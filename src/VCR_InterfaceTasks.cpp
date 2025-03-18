#include "CANInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "SystemTimeInterface.h"
#include "VCR_InterfaceTasks.h"
#include "ht_task.hpp"


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
#include "IOExpander.h"
#include "IOExpanderUtils.h"


bool init_adc_bundle()
{

    float adc0_scales[channels_within_mcp_adc], adc0_offsets[channels_within_mcp_adc], adc1_scales[channels_within_mcp_adc], adc1_offsets[channels_within_mcp_adc];  // NOLINT (C-style arrays)
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

bool run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
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

    return true;
}


bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    ADCSingletonInstance::instance().adc1.tick();

    return true;
}

bool run_update_buzzer_controller_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    vcr_data.system_data.buzzer_is_active = BuzzerController::getInstance().buzzer_is_active(sys_time::hal_millis()); //NOLINT
    return true;
}



bool init_ams_system_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    AMSSystemInstance::create(HEARTBEAT_INTERVAL_MS); // NOLINT 
    pinMode(SOFTWARE_OK_PIN, OUTPUT);
    return true;
}

bool run_ams_system_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    AMSSystemInstance::instance().update_ams_system(sys_time::hal_millis(), vcr_data);
    // digitalWrite(SOFTWARE_OK_PIN, vcr_data.system_data.ams_data.ams_ok);
    digitalWrite(SOFTWARE_OK_PIN, true);
    return true;
}



bool init_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInstance::create(WATCHDOG_KICK_INTERVAL_MS); // NOLINT
    pinMode(WATCHDOG_PIN, OUTPUT);
    return true;
}

bool run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    digitalWrite(WATCHDOG_PIN, WatchdogInstance::instance().get_watchdog_state(sys_time::hal_millis()));
    return true;
}

// CAN send tasks

// adds rear suspension and vcr status CAN messages to the sent on next mega loop run 
bool enqueue_suspension_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DrivebrainInterfaceInstance::instance().handle_enqueue_suspension_CAN_data();
    return true;
}

bool handle_send_VCR_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DrivebrainInterfaceInstance::instance().handle_send_ethernet_data(VCREthernetInterface::make_vcr_data_msg(vcr_data));
    return true;
}

bool handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceImpl::inverter_can_tx_buffer, &INVERTER_CAN);
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceImpl::telem_can_tx_buffer, &TELEM_CAN);
    return true;
}


bool enqueue_inverter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CANInterfacesInstance::instance().fl_inverter_interface.send_INV_CONTROL_WORD();
    CANInterfacesInstance::instance().fl_inverter_interface.send_INV_SETPOINT_COMMAND();

    CANInterfacesInstance::instance().fr_inverter_interface.send_INV_CONTROL_WORD();
    CANInterfacesInstance::instance().fr_inverter_interface.send_INV_SETPOINT_COMMAND();

    CANInterfacesInstance::instance().rl_inverter_interface.send_INV_CONTROL_WORD();
    CANInterfacesInstance::instance().rl_inverter_interface.send_INV_SETPOINT_COMMAND();

    CANInterfacesInstance::instance().rr_inverter_interface.send_INV_CONTROL_WORD();
    CANInterfacesInstance::instance().rr_inverter_interface.send_INV_SETPOINT_COMMAND();
    // VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceImpl::telem_can_tx_buffer, &TELEM_CAN);
    return true;
}

bool init_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    IOExpanderInstance::create(0x20);
    return true;
}

bool read_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // NOLINTBEGIN
    uint16_t data = 0;
    data = IOExpanderInstance::instance().read();
    vcr_data.interface_data.shutdown_sensing_data.bspd_is_ok = IOExpanderUtils::getBit(data, 0, 0);
    vcr_data.interface_data.shutdown_sensing_data.k_watchdog_relay = IOExpanderUtils::getBit(data, 0, 1);
    vcr_data.interface_data.shutdown_sensing_data.watchdog_is_ok = IOExpanderUtils::getBit(data, 0, 2);
    vcr_data.interface_data.shutdown_sensing_data.l_bms_relay = IOExpanderUtils::getBit(data, 0, 3);
    vcr_data.interface_data.shutdown_sensing_data.bms_is_ok = IOExpanderUtils::getBit(data, 0, 4);
    vcr_data.interface_data.shutdown_sensing_data.m_imd_relay = IOExpanderUtils::getBit(data, 0, 5);
    vcr_data.interface_data.shutdown_sensing_data.imd_is_ok = IOExpanderUtils::getBit(data, 0, 6);

    vcr_data.interface_data.ethernet_is_linked.acu_link = IOExpanderUtils::getBit(data, 1, 0);
    vcr_data.interface_data.ethernet_is_linked.drivebrain_link = IOExpanderUtils::getBit(data, 1, 1);
    vcr_data.interface_data.ethernet_is_linked.vcf_link = IOExpanderUtils::getBit(data, 1, 2);
    vcr_data.interface_data.ethernet_is_linked.teensy_link = IOExpanderUtils::getBit(data, 1, 3);
    vcr_data.interface_data.ethernet_is_linked.debug_link = IOExpanderUtils::getBit(data, 1, 4);
    vcr_data.interface_data.ethernet_is_linked.ubiquiti_link = IOExpanderUtils::getBit(data, 1, 5);

    return true;
    // NOLINTEND
}