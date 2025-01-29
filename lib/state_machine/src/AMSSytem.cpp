#include "AMSSystem.h"

void AMSSystem::init(unsigned long curr_micros) {
    _last_heartbeat_time = curr_micros;
}

AMSSystemData_s AMSSystem::update_ams_system(unsigned long curr_micros, BMSData_s &bms_data) {
    //calculate all the vars and return
    //check for the 3 conditions
    AMSSystemData_s system_data;
    bms_container = bms_data;
    bool heartbeat_check = check_heartbeat(curr_micros); //process heartbeat method

    system_data.min_cell_voltage = _filtered_min_cell_voltage;
    system_data.average_cell_voltage = -1;
    system_data.max_cell_voltage = -1;

    system_data.min_temp = -1;
    system_data.average_temp = -1;
    system_data.max_temp = _filtered_max_cell_temp;

    system_data.ams_ok = heartbeat_check && check_voltage();

    /*
    float min_cell_voltage;
    float average_cell_voltage;
    float max_cell_voltage;
    float min_temp; // Degrees celsius
    float average_temp; // Degrees celsius
    float max_temp; // Degrees celsius
    */

}

float AMSSystem::get_filtered_max_cell_temp() {
    bms_high_temp = get_high_temp();
    _filtered_max_cell_temp = _filtered_max_cell_temp * _cell_temp_alpha + (1.0 - _cell_temp_alpha) * bms_high_temp;
    return _filtered_max_cell_temp;
}

float AMSSystem::get_filtered_min_cell_voltage() {
    bms_low_voltage = get_min_cell_voltage();
    _filtered_min_cell_voltage = _filtered_min_cell_voltage * _cell_temp_alpha + (1.0 - _cell_voltage_alpha) * bms_low_voltage;
    return _filtered_min_cell_voltage;
}

//checkers
bool AMSSystem::check_voltage() {
    float temp_arr[] = bms_container.voltages;
    float sum = 0;
    for (int i = 0; i < temp_arr.size(); i++) {
        if (temp_arr[i] < PACK_CHARGE_CRIT_LOWEST_CELL_THRESHOLD) {
            return false;
        }
        sum += temp_arr[i];
    }
    if (sum < PACK_CHARGE_CRIT_TOTAL_THRESHOLD_VOLTS) {
        return false;
    }
    return true;
}

bool AMSSystem::check_heartbeat(unsigned long curr_micros) {
    unsigned long dur_since_last = curr_micros - _last_heartbeat_time;
    _last_heartbeat_time = curr_micros;
    if (dur_since_last > HEARTBEAT_INTERVAL_MS) {
        return false;
    }
    return true;
}


//getters
float AMSSystem::get_high_temp() {
    float temp_arr[] = bms_container.temperatures;
    float max = temp_arr[0];
    for (int i = 1; i < temp_arr.size(); i++) {
        if (temp_arr[i] >= max) {
            max = temp_arr[i];
        }
    }
    return max;
}

float AMSSystem::get_min_cell_voltage() {
    float temp_arr[] = bms_container.voltages;
    float min = temp_arr[0];
    for (int i = 1; i < temp_arr.size(); i++) {
        if (temp_arr[i] <= min) {
            min = temp_arr[i];
        }
    }
    return min;
}
