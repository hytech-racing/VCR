#ifdef ARDUINO
#include <Arduino.h>
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* From HT_SCHED libdep */
#include "ht_sched.hpp"

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCR_Globals.h"
#include "VCR_Constants.h"
#include "VCR_Tasks.h"
#include "TorqueControllerMux.hpp"

#include <InverterInterface.h>
#include <HytechCANInterface.h>

#include <hytech.h>

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

void setup(void)
{
    while (!Serial)
    {
        // wait for Arduino Serial Monitor to be ready
    }
    init_can_interface();
}
bool ran_test = false;
void loop()
{

  
}
