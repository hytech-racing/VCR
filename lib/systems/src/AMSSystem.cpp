#include "AMSSystem.h"
<<<<<<< HEAD:lib/systems/src/AMSSystem.cpp
=======

// Singleton access method
AMSSystem& AMSSystem::getInstance(int sw_ok_pin) {
    if (instance_ == nullptr) {
        if (sw_ok_pin == -1) {
            // Handle invalid initialization attempt
            throw std::runtime_error("AMSSystem must be initialized with valid arguments!");
        }
        instance_ = new AMSSystem(sw_ok_pin);
    }
    return *instance_;
}
>>>>>>> 4c7842f (Updated amssystem.h):lib/state_machine/src/AMSInterface.cpp


void AMSSystem::init(unsigned long curr_micros) {
    
    // Set pin mode
    pinMode(_pin_software_ok, OUTPUT);

    set_heartbeat(curr_micros);

    // Initializes the bms_voltages_ member variable to an invalid state. This will
    // get overridden once retrieve_voltage_CAN() has been called at least once.
    // we will be using receive_voltage_Ethernet() instead.
    bms_voltages_.low_voltage_ro = 0xFFFFU;
    bms_voltages_.high_voltage_ro = 0x1111U;

}

void AMSSystem::set_start_state() {
    
    digitalWrite(_pin_software_ok, HIGH);
    
}

//SETTERS//
void AMSSystem::set_state_ok_high(bool ok_high) {
    if (ok_high)
        digitalWrite(_pin_software_ok, HIGH);
    else
        digitalWrite(_pin_software_ok, LOW);
}

<<<<<<< HEAD:lib/systems/src/AMSSystem.cpp
//recieve heartbeat
=======
>>>>>>> 4c7842f (Updated amssystem.h):lib/state_machine/src/AMSInterface.cpp
void AMSSystem::set_heartbeat(unsigned long curr_micros) {
    last_heartbeat_time_ = curr_micros;
}

bool AMSSystem::heartbeat_received(unsigned long curr_millis) {
    return ((curr_millis - last_heartbeat_time_) < HEARTBEAT_INTERVAL);
}

<<<<<<< HEAD:lib/systems/src/AMSSystem.cpp
//will receive from ethernet
=======
>>>>>>> 4c7842f (Updated amssystem.h):lib/state_machine/src/AMSInterface.cpp
bool AMSSystem::is_below_pack_charge_critical_low_thresh() {
    return (HYTECH_low_voltage_ro_fromS(bms_voltages_.low_voltage_ro) < PACK_CHARGE_CRIT_LOWEST_CELL_THRESHOLD);
}

<<<<<<< HEAD:lib/systems/src/AMSSystem.cpp
//will receive from ethernet
=======
>>>>>>> 4c7842f (Updated amssystem.h):lib/state_machine/src/AMSInterface.cpp
bool AMSSystem::is_below_pack_charge_critical_total_thresh() {
    return (HYTECH_total_voltage_ro_fromS(bms_voltages_.total_voltage_ro) < PACK_CHARGE_CRIT_TOTAL_THRESHOLD);
}

bool AMSSystem::pack_charge_is_critical() {
    return (is_below_pack_charge_critical_low_thresh() || is_below_pack_charge_critical_total_thresh());
}

//GETTERS//
float AMSSystem::get_filtered_max_cell_temp() {
    bms_high_temp = bms_temperatures_.get_high_temperature() / 100.0;
    filtered_max_cell_temp = filtered_max_cell_temp * cell_temp_alpha + (1.0 - cell_temp_alpha) * bms_high_temp;
    return filtered_max_cell_temp;
}

<<<<<<< HEAD:lib/systems/src/AMSSystem.cpp
//will receive from ethernet
=======
>>>>>>> 4c7842f (Updated amssystem.h):lib/state_machine/src/AMSInterface.cpp
float AMSSystem::get_filtered_min_cell_voltage() {
    bms_low_voltage = HYTECH_low_voltage_ro_fromS(bms_voltages_.low_voltage_ro);
    filtered_min_cell_voltage = filtered_min_cell_voltage * cell_temp_alpha + (1.0 - cell_voltage_alpha) * bms_low_voltage;
    return filtered_min_cell_voltage;
<<<<<<< HEAD:lib/systems/src/AMSSystem.cpp
}

void AMSSystem::calculate_acc_derate_factor() {
    float voltage_lim_factor = 1.0;
    float startDerateVoltage = 3.5;
    float endDerateVoltage = 3.2;
    float voltage_lim_max = 1;
    float voltage_lim_min = 0.2;

    float temp_lim_factor = 1.0;
    float startDerateTemp = 50;
    float stopDerateTemp = 58;
    float temp_lim_max = 1;
    float temp_lim_min = 0.2;

    float filtered_min_cell_voltage = get_filtered_min_cell_voltage();
    //float_map equivalient because new code is bad 
    voltage_lim_factor = (filtered_min_cell_voltage - startDerateVoltage) * (voltage_lim_min - voltage_lim_max) / (endDerateVoltage - startDerateVoltage) + voltage_lim_max;
    voltage_lim_factor = max(min(voltage_lim_max, voltage_lim_factor), voltage_lim_min);

    temp_lim_factor = (filtered_max_cell_temp - startDerateTemp) * (temp_lim_min - temp_lim_max) / (stopDerateTemp - startDerateTemp) + temp_lim_max;
    temp_lim_factor = max(min(temp_lim_factor, temp_lim_max), temp_lim_min);
    
    acc_derate_factor = min(temp_lim_factor,voltage_lim_factor);
}

float AMSSystem::get_acc_derate_factor() {
    calculate_acc_derate_factor();
    return acc_derate_factor;
}


=======
}
>>>>>>> 4c7842f (Updated amssystem.h):lib/state_machine/src/AMSInterface.cpp
