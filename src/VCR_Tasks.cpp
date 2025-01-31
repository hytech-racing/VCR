#include "VCR_Tasks.h"

/* From HT_SCHED library */
#include "ht_sched.hpp"

/* From shared-systems-lib */
#include "Logger.h"

/* From shared_firmware_types library */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCR_Constants.h"
#include "Buzzer.h"
#include "VCR_Globals.h"
#include "VehicleStateMachine.h"

bool init_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    adc_0.setChannelScaleAndOffset(GLV_SENSE_CHANNEL, GLV_SENSE_SCALE, GLV_SENSE_OFFSET);
    adc_0.setChannelScaleAndOffset(CURRENT_SENSE_CHANNEL, CURRENT_SENSE_SCALE, CURRENT_SENSE_OFFSET);
    adc_0.setChannelScaleAndOffset(REFERENCE_SENSE_CHANNEL, REFERENCE_SENSE_SCALE, REFERENCE_SENSE_OFFSET);
    adc_0.setChannelScaleAndOffset(RL_LOADCELL_CHANNEL, RL_LOADCELL_SCALE, RL_LOADCELL_OFFSET);
    adc_0.setChannelScaleAndOffset(RR_LOADCELL_CHANNEL, RL_LOADCELL_SCALE, RL_LOADCELL_OFFSET);
    adc_0.setChannelScaleAndOffset(RL_SUS_POT_CHANNEL, RL_SUS_POT_SCALE, RL_SUS_POT_OFFSET);
    adc_0.setChannelScaleAndOffset(RR_SUS_POT_CHANNEL, RR_SUS_POT_SCALE, RR_SUS_POT_OFFSET);

    hal_printf("Initialized ADC0 at %d (micros)\n", sysMicros);

    return true;
}

bool run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    adc_0.sample(); // Samples all eight channels.
    adc_0.convert(); // Converts all eight channels.

    interface_data.current_sensor_data.twentyfour_volt_sensor = adc_0.data.conversions[GLV_SENSE_CHANNEL].conversion;
    interface_data.current_sensor_data.current_sensor_unfiltered = adc_0.data.conversions[CURRENT_SENSE_CHANNEL].conversion;
    interface_data.current_sensor_data.current_refererence_unfiltered = adc_0.data.conversions[REFERENCE_SENSE_CHANNEL].conversion;
    interface_data.rear_loadcell_data.RL_loadcell_analog = adc_0.data.conversions[RL_LOADCELL_CHANNEL].conversion;
    interface_data.rear_loadcell_data.RR_loadcell_analog = adc_0.data.conversions[RR_LOADCELL_CHANNEL].conversion;
    interface_data.rear_suspot_data.RL_sus_pot_analog = adc_0.data.conversions[RL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    interface_data.rear_suspot_data.RR_sus_pot_analog = adc_0.data.conversions[RR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    return true;
}

HT_TASK::Task read_adc0_task = HT_TASK::Task(init_read_adc0_task, run_read_adc0_task, 10, 1000UL); // 1000us is 1kHz



bool run_tick_state_machine_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VehicleStateMachine::getInstance().tick_state_machine(sysMicros / 1000); // tick function requires millis
    return true;
}

HT_TASK::Task tick_state_machine_task = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, run_tick_state_machine_task, 2, 10000UL); // 10,000us is 100hz



bool init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    // Initialize all eight channels to scale = 1, offset = 0
    adc_1.setChannelScaleAndOffset(0, 1, 0);
    adc_1.setChannelScaleAndOffset(1, 1, 0);
    adc_1.setChannelScaleAndOffset(2, 1, 0);
    adc_1.setChannelScaleAndOffset(3, 1, 0);
    adc_1.setChannelScaleAndOffset(4, 1, 0);
    adc_1.setChannelScaleAndOffset(5, 1, 0);
    adc_1.setChannelScaleAndOffset(6, 1, 0);
    adc_1.setChannelScaleAndOffset(7, 1, 0);

    hal_printf("Initialized ADC0 at %d (micros)\n", sysMicros);

    return true;
}

bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    adc_1.sample(); // Samples all eight channels.
    adc_1.convert(); // Converts all eight channels.

    return true;
}

HT_TASK::Task read_adc1_task = HT_TASK::Task(init_read_adc1_task, run_read_adc1_task, 10, 40000UL); // 20000us is 25Hz



bool run_update_buzzer_controller_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{

    system_data.buzzer_is_active = BuzzerController::getInstance().buzzer_is_active(sysMicros / 1000);

    return true;
}

HT_TASK::Task update_buzzer_controller_task = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, run_update_buzzer_controller_task, 10, 20000UL); // 20000us is 50hz



bool run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    digitalWrite(watchdog_pin, WatchdogSystem::getInstance().get_watchdog_state(sysMicros / 1000));
    return true;
}
HT_TASK::Task kick_watchdog_task = HT_TASK::Task(HT_TASK::DUMMY_FUNCTION, run_kick_watchdog, 3, 2000UL); // 2000us is 500hz