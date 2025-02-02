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

    system_data.min_cell_voltage = get_filtered_min_cell_voltage();
    system_data.average_cell_voltage = get_filtered_average_cell_voltage();
    system_data.max_cell_voltage = get_filtered_max_cell_voltage();

    system_data.min_temp = get_filtered_min_cell_temp();
    system_data.average_temp = get_filtered_average_cell_temp();
    system_data.max_temp = get_filtered_max_cell_temp();

    system_data.ams_ok = heartbeat_check && check_voltage();
}



//checkers
bool AMSSystem::check_voltage() {
    float sum = 0;
    for (float num : bms_container.voltages) {
        if (num < PACK_CHARGE_CRIT_LOWEST_CELL_THRESHOLD) {
            return false;
        }
        sum += num;
    }
    sum = sum / 1000000;
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
float AMSSystem::get_filtered_max_cell_temp() {
    float bms_high_temp = bms_container.temperatures[0];
    for (float num : bms_container.temperatures) {
        if (num >= bms_high_temp) {
            bms_high_temp = num;
        }
    }
    _filtered_max_cell_temp = _filtered_max_cell_temp * _cell_temp_alpha + (1.0 - _cell_temp_alpha) * bms_high_temp;
    return _filtered_max_cell_temp;
}

float AMSSystem::get_filtered_min_cell_voltage() {
    float bms_low_voltage = bms_container.voltages[0];
    for (float num : bms_container.voltages) {
        if (num <= bms_low_voltage) {
            bms_low_voltage = num;
        }
    }
    _filtered_min_cell_voltage = _filtered_min_cell_voltage * _cell_voltage_alpha + (1.0 - _cell_voltage_alpha) * bms_low_voltage;
    return _filtered_min_cell_voltage;
}


//Extra filtered getters
float AMSSystem::get_filtered_average_cell_voltage() {
    float sum = 0;
    for (float num : bms_container.voltages) {
        sum += num;
    }
    float avg = sum / 126;
    _filtered_average_cell_voltage = _filtered_average_cell_voltage * _cell_voltage_alpha + (1.0 - _cell_voltage_alpha) * avg;
    return _filtered_average_cell_voltage;
}


float AMSSystem::get_filtered_average_cell_temp() {
    float sum = 0;
    for (float num : bms_container.temperatures) {
        sum += num;
    }
    float avg = sum / 12;
    _filtered_average_cell_temp = _filtered_average_cell_temp * _cell_temp_alpha + (1.0 - _cell_temp_alpha) * avg;
    return _filtered_average_cell_temp;
}

float AMSSystem::get_filtered_min_cell_temp() {
    float min = bms_container.temperatures[0];
    for (float num : bms_container.temperatures) {
        if (num <= min) {
            min = num;
        }
    }
    _filtered_min_cell_temp = _filtered_min_cell_temp * _cell_temp_alpha + (1.0 - _cell_temp_alpha) * min;
}

float AMSSystem::get_filtered_max_cell_voltage() {
    float max = bms_container.voltages[0];
    for (float num : bms_container.voltages) {
        if (num >= max) {
            max = num;
        }
    }
    _filtered_max_cell_voltage = _filtered_max_cell_voltage * _cell_voltage_alpha + (1.0 - _cell_voltage_alpha) * max;
}