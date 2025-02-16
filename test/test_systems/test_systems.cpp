#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_tcmux.h"
#include "test_buzzer.h"
#include "test_watchdog.h"
#include "test_drivebrain_controller.h"

int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
	if(RUN_ALL_TESTS()) {
        // Do nothing (always return 0 and allow PlatformIO to parse result)
    }
	return 0;
}