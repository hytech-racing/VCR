#ifndef VCR_CONSTANTS
#define VCR_CONSTANTS


/* -------------------------------------------------- */
/*                 Teensy 4.1 GPIO pins               */
/* -------------------------------------------------- */
constexpr int ADC0_CS = 40; // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
constexpr int ADC1_CS = 39; // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.
constexpr int BRAKELIGHT_CONTROL_PIN = 32;


/* -------------------------------------------------- */
/*                 ADC pins and configs               */
/* -------------------------------------------------- */

/* Channels on adc_0 */
constexpr int GLV_SENSE_CHANNEL       = 0;
constexpr int CURRENT_SENSE_CHANNEL   = 1;
constexpr int REFERENCE_SENSE_CHANNEL = 2;
constexpr int RL_LOADCELL_CHANNEL     = 3;
constexpr int RR_LOADCELL_CHANNEL     = 4;
constexpr int RL_SUS_POT_CHANNEL      = 5;
constexpr int RR_SUS_POT_CHANNEL      = 6;
// const int UNUSED_CHANNEL       = 7;

/* Channels on ADC_1 */
constexpr int THERMISTOR_0 = 0;
constexpr int THERMISTOR_1 = 1;
constexpr int THERMISTOR_2 = 2;
constexpr int THERMISTOR_3 = 3;
constexpr int THERMISTOR_4 = 4;
constexpr int THERMISTOR_5 = 5;
constexpr int THERMISTOR_6 = 6;
constexpr int THERMISTOR_7 = 7;

/* Scaling and offset */
constexpr float GLV_SENSE_SCALE = (float)(24.0/((2.77149877/3.3)*4096.0)); //unsure about the multiplication by 4.0865
constexpr int GLV_SENSE_OFFSET = 0; //No offset for GLV
constexpr float CURRENT_SENSE_SCALE = (float)(24/((2.77149877/3.3)*4096)); //unsure about the multiplication by 4.0865
constexpr int CURRENT_SENSE_OFFSET = 0; //No offset for CURRENT_SENSE
constexpr float REFERENCE_SENSE_SCALE = (float)(24/((2.77149877/3.3)*4096)); //unsure about the multiplication by 4.0865
constexpr int REFERENCE_SENSE_OFFSET = 0; //No offset for REFERENCE_SENSE

//Values are from the old MCU rev15
constexpr float RL_LOADCELL_SCALE = 1;
constexpr float RL_LOADCELL_OFFSET = 0;
constexpr float RR_LOADCELL_SCALE = 1;
constexpr float RR_LOADCELL_OFFSET = 0;

//does not matter that much
constexpr float RL_SUS_POT_SCALE = 1;
constexpr int RL_SUS_POT_OFFSET = 0;
constexpr float RR_SUS_POT_SCALE = 1;
constexpr int RR_SUS_POT_OFFSET = 0;

//figure out values
const float THERMISTOR_0_SCALE = 1;
const float THERMISTOR_0_OFFSET = 0; 
const float THERMISTOR_1_SCALE = 1;
const float THERMISTOR_1_OFFSET = 0; 
const float THERMISTOR_2_SCALE = 1;
const float THERMISTOR_2_OFFSET = 0; 
const float THERMISTOR_3_SCALE = 1;
const float THERMISTOR_3_OFFSET = 0; 
const float THERMISTOR_4_SCALE = 1;
const float THERMISTOR_4_OFFSET = 0; 
const float THERMISTOR_5_SCALE = 1;
const float THERMISTOR_5_OFFSET = 0; 
const float THERMISTOR_6_SCALE = 1;
const float THERMISTOR_6_OFFSET = 0; 
const float THERMISTOR_7_SCALE = 1;
const float THERMISTOR_7_OFFSET = 0; 

/* Watchdog constants */
constexpr int INVERTER_ENABLE_PIN = 2;
constexpr int WATCHDOG_PIN = 36;
constexpr int SOFTWARE_OK_PIN = 37; // Watchdog's !RESET pin
constexpr unsigned long WATCHDOG_KICK_INTERVAL_MS = 10UL;
constexpr unsigned long VCF_PEDALS_MAX_HEARTBEAT_MS = 50UL;         // 20ms at 60mph is about 0.5 meters
constexpr uint64_t ACU_ACU_OK_MAX_HEARTBEAT_MS = 500;
constexpr unsigned long MAX_ALLOWED_DB_LATENCY_MS = 20; // milliseconds

/* Inverter constants */
constexpr int INVERTER_EN_PIN = 2;
constexpr int INVERTER_MINIMUM_HV_VOLTAGE = 60;

/* Filter constants */
constexpr float LOADCELL_IIR_FILTER_ALPHA = 0.01f;

/* Task loop rates & priorities */
constexpr unsigned long adc0_sample_period_us = 250;                 // 250 us = 4 kHz
constexpr unsigned long adc0_priority = 7;
constexpr unsigned long adc1_sample_period_us = 10000;               // 10 000 us = 100 Hz
constexpr unsigned long adc1_priority = 50;
constexpr unsigned long update_buzzer_controller_period_us = 100000; // 100 000 us = 10 Hz
constexpr unsigned long buzzer_priority = 3;
constexpr unsigned long kick_watchdog_period_us = 10000;             // 10 000 us = 100 Hz
constexpr unsigned long watchdog_priority = 1;
constexpr unsigned long ams_update_period_us = 5000;                 // 5 000 us = 200 Hz
constexpr unsigned long ams_priority = 2;
constexpr unsigned long suspension_can_period_us = 4000;             // 4 000 us = 250 Hz
constexpr unsigned long suspension_priority = 4;
constexpr unsigned long ethernet_update_period = 100000;             // 50 000 us = 20 Hz
constexpr unsigned long ethernet_send_priority = 6;
constexpr unsigned long inv_send_period = 5000;                      // 5 000 us = 200 Hz
constexpr unsigned long inverter_send_priority = 5;
constexpr unsigned long ioexpander_sample_period_us = 50000;         // 50 000 us = 20 Hz
constexpr unsigned long ioexpander_priority = 100;
constexpr unsigned long send_can_period_us = 1000;                   // 1 000 us = 1 000 Hz
constexpr unsigned long send_can_priority = 2;
constexpr unsigned long main_task_period_us = 100;                   // 100 us = 10 kHz
constexpr unsigned long main_task_priority = 0;
constexpr unsigned long update_brakelight_priority = 20;
constexpr unsigned long update_brakelight_period_us = 50000UL;       // 50 000 us = 20 Hz

#endif /* VCR_CONSTANTS */