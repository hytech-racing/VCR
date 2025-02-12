#include "gtest/gtest.h"
#include "AMSSystem.h"

AMSSystem &ams = AMSSystem::getInstance();
unsigned long init_millis = 10000; // arbitrary number

void ASSERT_AMS_SYSTEM_DATA_EQ(AMSSystemData_s expected, AMSSystemData_s actual)
{
    ASSERT_FLOAT_EQ(expected.min_cell_voltage, actual.min_cell_voltage);
    ASSERT_FLOAT_EQ(expected.average_cell_voltage, actual.average_cell_voltage);
    ASSERT_FLOAT_EQ(expected.max_cell_voltage, actual.max_cell_voltage);
    ASSERT_FLOAT_EQ(expected.min_temp_celsius, actual.min_temp_celsius);
    ASSERT_FLOAT_EQ(expected.average_temp_celsius, actual.average_temp_celsius);
    ASSERT_FLOAT_EQ(expected.max_temp_celsius, actual.max_temp_celsius);
    ASSERT_FLOAT_EQ(expected.total_pack_voltage, actual.total_pack_voltage);
    ASSERT_EQ(expected.ams_ok, actual.ams_ok);
}

TEST (AMSSystemTest, initialization_test) {
    // Create expected result of AMS data
    AMSSystemData_s expected_init_data = {
        .min_cell_voltage = 3.5f,
        .average_cell_voltage = 3.5f,
        .max_cell_voltage = 3.5f,
        .min_temp_celsius = 40.0f,
        .average_temp_celsius = 40.0f,
        .max_temp_celsius = 40.0f,
        .total_pack_voltage = 441.0f,
        .ams_ok = true
    };

    AMSSystemData_s init_data = ams.init(init_millis);

    ASSERT_AMS_SYSTEM_DATA_EQ(expected_init_data, init_data);
}

TEST (AMSSystemTest, tick_once) {
    // Declare VCRData_s struct to pass into update function
    VCRData_s state = {};
    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.0f;
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 469.5f;
    state.interface_data.stamped_acu_core_data.last_recv_millis = init_millis + 10;
    
    // Declare the expected result of the test
    AMSSystemData_s expected_result = {
        .min_cell_voltage = 3.4f, // Initialized to 3.5, updated value is 3.0
        .average_cell_voltage = 3.6f, // Initialized to 3.5, updated value is 4.0
        .max_cell_voltage = -1.0f, // Max voltage not used, so set to -1
        .min_temp_celsius = -1.0f, // Min temp not used, so set to -1
        .average_temp_celsius = -1.0f, // Avg temp not used, so set to -1
        .max_temp_celsius = 42.5f, // Initialized to 40, updated to 50
        .total_pack_voltage = 469.5f, // Ovewrriten to 469.5
        .ams_ok = true
    };

    state.system_data.ams_data = ams.init(init_millis); // Init AMSSystem to return it to an expected state
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_result, ams.update_ams_system(init_millis + 10, state));
}

TEST (AMSSystemTest, test_cell_undervoltage_shutdown) {
    // Declare VCRData_s struct to pass into update function
    VCRData_s state = {};
    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.0f;
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 469.5f;
    state.interface_data.stamped_acu_core_data.last_recv_millis = init_millis + 10;
    
    // Declare the expected result of the test
    AMSSystemData_s expected_result = {
        .min_cell_voltage = 3.4f, // Initialized to 3.5, updated value is 3.0
        .average_cell_voltage = 3.6f, // Initialized to 3.5, updated value is 4.0
        .max_cell_voltage = -1.0f, // Max voltage not used, so set to -1
        .min_temp_celsius = -1.0f, // Min temp not used, so set to -1
        .average_temp_celsius = -1.0f, // Avg temp not used, so set to -1
        .max_temp_celsius = 42.5f, // Initialized to 40, updated to 50
        .total_pack_voltage = 469.5f, // Ovewrriten to 469.5
        .ams_ok = true
    };

    state.system_data.ams_data = ams.init(init_millis); // Init AMSSystem to return it to an expected state
    state.system_data.ams_data = ams.update_ams_system(init_millis + 10, state);
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_result, state.system_data.ams_data);

    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.0f; // Lowers min cell voltage to 3.32V
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 500;
    expected_result = {
        .min_cell_voltage = 3.32f,
        .average_cell_voltage = 3.68f, // Initialized to 3.5, updated value is 4.0
        .max_cell_voltage = -1.0f,
        .min_temp_celsius = -1.0f,
        .average_temp_celsius = -1.0f,
        .max_temp_celsius = 44.375, // Initialized to 40, updated to 50
        .total_pack_voltage = 500, // Ovewrriten to 469.5
        .ams_ok = true
    };
    state.system_data.ams_data = ams.update_ams_system(init_millis + 20, state);
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_result, state.system_data.ams_data);

    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.0f; // Lowers min cell voltage to 3.256V
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 500;
    state.interface_data.stamped_acu_core_data.last_recv_millis = init_millis + 30;
    state.system_data.ams_data = ams.update_ams_system(init_millis + 30, state);
    ASSERT_EQ(true, state.system_data.ams_data.ams_ok);

    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 2.9f; // Lowers min cell voltage to 3.185V
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 500;
    state.interface_data.stamped_acu_core_data.last_recv_millis = init_millis + 40;
    state.system_data.ams_data = ams.update_ams_system(init_millis + 40, state);
    ASSERT_EQ(false, state.system_data.ams_data.ams_ok);
}

TEST (AMSSystemTest, test_pack_undervoltage_shutdown) {
    // Declare VCRData_s struct to pass into update function
    VCRData_s state = {};
    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.0f;
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 469.5f;
    state.interface_data.stamped_acu_core_data.last_recv_millis = init_millis + 10;
    
    // Declare the expected result of the test
    AMSSystemData_s expected_result = {
        .min_cell_voltage = 3.4f, // Initialized to 3.5, updated value is 3.0
        .average_cell_voltage = 3.6f, // Initialized to 3.5, updated value is 4.0
        .max_cell_voltage = -1.0f, // Max voltage not used, so set to -1
        .min_temp_celsius = -1.0f, // Min temp not used, so set to -1
        .average_temp_celsius = -1.0f, // Avg temp not used, so set to -1
        .max_temp_celsius = 42.5f, // Initialized to 40, updated to 50
        .total_pack_voltage = 469.5f, // Ovewrriten to 469.5
        .ams_ok = true
    };

    state.system_data.ams_data = ams.init(init_millis); // Init AMSSystem to return it to an expected state
    state.system_data.ams_data = ams.update_ams_system(init_millis + 10, state);
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_result, state.system_data.ams_data);

    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.4f;
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 430;
    state.interface_data.stamped_acu_core_data.last_recv_millis = init_millis + 20;
    expected_result = {
        .min_cell_voltage = 3.4f,
        .average_cell_voltage = 3.68f, // Initialized to 3.5, updated value is 4.0
        .max_cell_voltage = -1.0f,
        .min_temp_celsius = -1.0f,
        .average_temp_celsius = -1.0f,
        .max_temp_celsius = 44.375,
        .total_pack_voltage = 430, // Ovewrriten to 430
        .ams_ok = true
    };
    state.system_data.ams_data = ams.update_ams_system(init_millis + 20, state);
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_result, state.system_data.ams_data);

    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.0f; // Lowers min cell voltage to 3.256V
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 419;
    state.interface_data.stamped_acu_core_data.last_recv_millis = init_millis + 30;
    state.system_data.ams_data = ams.update_ams_system(init_millis + 30, state);
    ASSERT_EQ(false, state.system_data.ams_data.ams_ok);

}

TEST (AMSSystemTest, test_heartbeat_overrun_shutdown) {
    // Declare VCRData_s struct to pass into update function
    VCRData_s state = {};
    state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.0f;
    state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 4.0f;
    state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 469.5f;
    state.interface_data.stamped_acu_core_data.last_recv_millis = init_millis + 100;
    
    // Declare the expected result of the test
    AMSSystemData_s expected_result = {
        .min_cell_voltage = 3.4f, // Initialized to 3.5, updated value is 3.0
        .average_cell_voltage = 3.6f, // Initialized to 3.5, updated value is 4.0
        .max_cell_voltage = -1.0f, // Max voltage not used, so set to -1
        .min_temp_celsius = -1.0f, // Min temp not used, so set to -1
        .average_temp_celsius = -1.0f, // Avg temp not used, so set to -1
        .max_temp_celsius = 42.5f, // Initialized to 40, updated to 50
        .total_pack_voltage = 469.5f, // Ovewrriten to 469.5
        .ams_ok = true
    };

    state.system_data.ams_data = ams.init(init_millis); // Init AMSSystem to return it to an expected state
    state.system_data.ams_data = ams.update_ams_system(init_millis + 100, state);
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_result, state.system_data.ams_data);

    state.system_data.ams_data = ams.update_ams_system(init_millis + 1900, state);
    ASSERT_EQ(true, state.system_data.ams_data.ams_ok);

    state.system_data.ams_data = ams.update_ams_system(init_millis + 2050, state); // Still OK, since last_recv_millis = init_millis + 100
    ASSERT_EQ(true, state.system_data.ams_data.ams_ok);

    state.system_data.ams_data = ams.update_ams_system(init_millis + 2101, state); // Still OK, since last_recv_millis = init_millis + 100
    ASSERT_EQ(false, state.system_data.ams_data.ams_ok);

}