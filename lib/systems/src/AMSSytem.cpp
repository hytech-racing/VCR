#include "AMSSystem.h"

AMSSystemData_s AMSSystem::init(unsigned long curr_millis)
{
    _last_heartbeat_time_ms = curr_millis;

    AMSSystemData_s init_system_data;
    init_system_data.min_cell_voltage = DEFAULT_INIT_VOLTAGE;
    init_system_data.average_cell_voltage = DEFAULT_INIT_VOLTAGE;
    init_system_data.max_cell_voltage = DEFAULT_INIT_VOLTAGE;
    init_system_data.min_temp_celsius = DEFAULT_INIT_TEMP;
    init_system_data.average_temp_celsius = DEFAULT_INIT_TEMP;
    init_system_data.max_temp_celsius = DEFAULT_INIT_TEMP;
    init_system_data.total_pack_voltage = DEFAULT_INIT_VOLTAGE * 126; // 126 cells in pack //NOLINT
    init_system_data.ams_ok = true;

    return init_system_data;
}

AMSSystemData_s AMSSystem::update_ams_system(unsigned long curr_millis, VCRData_s &vcr_data)
{
    // Declare struct to return
    AMSSystemData_s ret;

    ret.total_pack_voltage = vcr_data.interface_data.acu_core_data.pack_voltage;

    // Apply IIR filters to cell voltages & temps
    ret.min_cell_voltage = vcr_data.system_data.ams_data.min_cell_voltage * _cell_voltage_alpha + vcr_data.interface_data.acu_core_data.min_cell_voltage * (1.0f - _cell_voltage_alpha);
    ret.average_cell_voltage = vcr_data.system_data.ams_data.average_cell_voltage * _cell_voltage_alpha + vcr_data.interface_data.acu_core_data.avg_cell_voltage * (1.0f - _cell_temp_alpha);
    ret.max_cell_voltage = -1.0f; // We're only using min and average cell voltage, so this is -1.0 for now.

    ret.min_temp_celsius = -1.0f; // We're only using max cell temps, so this is -1.0 for now.
    ret.average_temp_celsius = -1.0f; // We're only using max cell temps, so this is -1.0 for now.
    ret.max_temp_celsius = vcr_data.system_data.ams_data.max_temp_celsius * _cell_temp_alpha + vcr_data.interface_data.acu_core_data.max_cell_temp * (1.0f - _cell_temp_alpha);

    // Check for three shutdown conditions
    bool heartbeat_ok = _check_heartbeat(curr_millis);
    bool lowest_cell_below_threshold = ret.min_cell_voltage < PACK_CHARGE_CRIT_LOWEST_CELL_THRESHOLD;
    bool pack_below_threshold = ret.total_pack_voltage < PACK_CHARGE_CRIT_TOTAL_THRESHOLD_VOLTS;

    return ret;
}

bool AMSSystem::_check_heartbeat(unsigned long curr_micros) {
    unsigned long dur_since_last = curr_micros - _last_heartbeat_time_ms;
    _last_heartbeat_time_ms = curr_micros;
    if (dur_since_last > HEARTBEAT_INTERVAL_MS) {
        return false;
    }
    return true;
}