#include "AMSSystem.h"

AMSSystemData_s AMSSystem::update_ams_system(unsigned long curr_millis, VCRData_s &vcr_data)
{
    // Declare struct to return
    AMSSystemData_s ret = {};

    ret.ams_ok = true;

    // If no message has been received, return immediately.
    if (!_has_received_one_message && vcr_data.interface_data.stamped_acu_core_data.last_recv_millis == 0)
    {
        return ret;
    }

    ret.total_pack_voltage = vcr_data.interface_data.stamped_acu_core_data.acu_data.pack_voltage;

    // Directly copy data from ACU Core Data
    ret.min_cell_voltage = vcr_data.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage;
    ret.average_cell_voltage = vcr_data.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage;
    ret.max_cell_voltage = -1.0f; // We're only using min and average cell voltage, so this is -1.0 for now.

    ret.min_temp_celsius = -1.0f; // We're only using max cell temps, so this is -1.0 for now.
    ret.average_temp_celsius = -1.0f; // We're only using max cell temps, so this is -1.0 for now.
    ret.max_temp_celsius = vcr_data.interface_data.stamped_acu_core_data.acu_data.max_cell_temp;

    // Check for three shutdown conditions
    bool heartbeat_ok = curr_millis - vcr_data.interface_data.stamped_acu_core_data.last_recv_millis < _heartbeat_interval_ms;

    ret.ams_ok = heartbeat_ok;

    return ret;
}