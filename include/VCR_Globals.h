#ifndef VCR_GLOBALS
#define VCR_GLOBALS

#include <etl/singleton.h>

/* C++ library includes */
#include <array>

/* IO Expander MCP23017 Library */
#include "MCP23017.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* From interfaces */
#include "InverterInterface.h"

/* Interface and system data structs */
extern VCRData_s vcr_data; // NOLINT

/* IO Expander Setup */
using IOExpanderInstance = etl::singleton<MCP23017>;

/* Inverters Setup -- defined in main */
extern InverterInterface fl_inverter_int;
extern InverterInterface fr_inverter_int;
extern InverterInterface rl_inverter_int;
extern InverterInterface rr_inverter_int;

extern unsigned long pulseCount; // NOLINT

#endif /* VCR_GLOBALS */
