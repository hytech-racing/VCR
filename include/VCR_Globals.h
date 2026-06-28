#ifndef VCR_GLOBALS
#define VCR_GLOBALS

#include <etl/singleton.h>

/* C++ library includes */
#include <array>

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* From interfaces */
#include "InverterInterface.h"

/* Interface and system data structs */
extern VCRData_s vcr_data; // NOLINT

/* Inverters Setup -- defined in main */
extern InverterInterface fl_inverter_int;
extern InverterInterface fr_inverter_int;
extern InverterInterface rl_inverter_int;
extern InverterInterface rr_inverter_int;

extern unsigned long pulseCount; // NOLINT

#endif /* VCR_GLOBALS */
