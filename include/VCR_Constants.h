#ifndef VCR_CONSTANTS
#define VCR_CONSTANTS



/* -------------------------------------------------- */
/*                 Teensy 4.1 GPIO pins               */
/* -------------------------------------------------- */
const int ADC0_CS = 10; // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
const int ADC1_CS = 11; // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.



/* -------------------------------------------------- */
/*                 ADC pins and configs               */
/* -------------------------------------------------- */

/* Channels on ADC_0 */
const int GLV_SENSE_CHANNEL       = 0;
const int CURRENT_SENSE_CHANNEL   = 1;
const int REFERENCE_SENSE_CHANNEL = 2;
const int RL_LOADCELL_CHANNEL     = 3;
const int RR_LOADCELL_CHANNEL     = 4;
const int RL_SUS_POT_CHANNEL      = 5;
const int RR_SUS_POT_CHANNEL      = 6;
// const int UNUSED_CHANNEL       = 7;

/* Channels on ADC_1 */
const int THERMISTOR_0 = 0;
const int THERMISTOR_1 = 1;
const int THERMISTOR_2 = 2;
const int THERMISTOR_3 = 3;
const int THERMISTOR_4 = 4;
const int THERMISTOR_5 = 5;
const int THERMISTOR_6 = 6;
const int THERMISTOR_7 = 7;

/* Scaling and offset */
const float GLV_SENSE_SCALE = -1; //TODO: FIGURE THIS OUT
const int GLV_SENSE_OFFSET = -1; //TODO: FIGURE THIS OUT
const float CURRENT_SENSE_SCALE = -1; //TODO: FIGURE THIS OUT
const int CURRENT_SENSE_OFFSET = -1; //TODO: FIGURE THIS OUT
const float REFERENCE_SENSE_SCALE = -1; //TODO: FIGURE THIS OUT
const int REFERENCE_SENSE_OFFSET = -1; //TODO: FIGURE THIS OUT
const float RL_LOADCELL_SCALE = -1; //TODO: FIGURE THIS OUT
const int RL_LOADCELL_OFFSET = -1; //TODO: FIGURE THIS OUT
const float RR_LOADCELL_SCALE = -1; //TODO: FIGURE THIS OUT
const int RR_LOADCELL_OFFSET = -1; //TODO: FIGURE THIS OUT
const float RL_SUS_POT_SCALE = -1; //TODO: FIGURE THIS OUT
const int RL_SUS_POT_OFFSET = -1; //TODO: FIGURE THIS OUT
const float RR_SUS_POT_SCALE = -1; //TODO: FIGURE THIS OUT
const int RR_SUS_POT_OFFSET = -1; //TODO: FIGURE THIS OUT



#endif /* VCR_CONSTANTS */