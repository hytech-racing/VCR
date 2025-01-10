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

/* Channels on adc_0 */
const int GLV_SENSE_CHANNEL       = 0; //1
const int CURRENT_SENSE_CHANNEL   = 1; //2
const int REFERENCE_SENSE_CHANNEL = 2; //3
const int RL_LOADCELL_CHANNEL     = 3; //4
const int RR_LOADCELL_CHANNEL     = 4; //5
const int RL_SUS_POT_CHANNEL      = 5; //6
const int RR_SUS_POT_CHANNEL      = 6; //7
// const int UNUSED_CHANNEL       = 7; 8

/* Channels on ADC_1 */
const int THERMISTOR_0 = 0; //1
const int THERMISTOR_1 = 1; //2
const int THERMISTOR_2 = 2; //3
const int THERMISTOR_3 = 3; //4
const int THERMISTOR_4 = 4; //5
const int THERMISTOR_5 = 5; //6
const int THERMISTOR_6 = 6; //7
const int THERMISTOR_7 = 7; //8

/* Scaling and offset */
const float GLV_SENSE_SCALE = 24/((2.77149877/3.3)*4096); //unsure about the multiplication by 4.0865
// const int GLV_SENSE_OFFSET = -1; //No offset for GLV
const float CURRENT_SENSE_SCALE = 24/((2.77149877/3.3)*4096); //unsure about the multiplication by 4.0865
// const int CURRENT_SENSE_OFFSET = -1; //No offset for CURRENT_SENSE
const float REFERENCE_SENSE_SCALE = 24/((2.77149877/3.3)*4096); //unsure about the multiplication by 4.0865
// const int REFERENCE_SENSE_OFFSET = -1; //No offset for REFERENCE_SENSE

//Values are from the old MCU rev15
const float LOADCELL_RL_SCALE = 0.1149;
const float LOADCELL_RL_OFFSET = 13.526 / LOADCELL_RL_SCALE;
const float LOADCELL_RR_SCALE = 0.118;
const float LOADCELL_RR_OFFSET = 25.721 / LOADCELL_RR_SCALE;

//does not matter that much
//5/((3/3.3)*4096); //not sure if this also needs to be multiplied by 4.0865
const float RL_SUS_POT_SCALE = 1
const int RL_SUS_POT_OFFSET = 1; //TODO: FIGURE THIS OUT
const float RR_SUS_POT_SCALE = 1; //TODO: FIGURE THIS OUT
const int RR_SUS_POT_OFFSET = 1; //TODO: FIGURE THIS OUT



#endif /* VCR_CONSTANTS */