#include "gtest/gtest.h"
#include "AMSSystem.h"

unsigned long init_millis = 10000; // arbitrary number

void ASSERT_AMS_SYSTEM_DATA_EQ(AMSSystemData_s expected, AMSSystemData_s actual)
{
    ASSERT_NEAR(expected.min_cell_voltage, actual.min_cell_voltage, 0.001f);
    ASSERT_NEAR(expected.average_cell_voltage, actual.average_cell_voltage, 0.001f);
    ASSERT_NEAR(expected.max_cell_voltage, actual.max_cell_voltage, 0.001f);
    ASSERT_NEAR(expected.min_temp_celsius, actual.min_temp_celsius, 0.001f);
    ASSERT_NEAR(expected.average_temp_celsius, actual.average_temp_celsius, 0.001f);
    ASSERT_NEAR(expected.max_temp_celsius, actual.max_temp_celsius, 0.001f);
    ASSERT_NEAR(expected.total_pack_voltage, actual.total_pack_voltage, 0.001f);
    ASSERT_EQ(expected.ams_ok, actual.ams_ok);
}

TEST (AMSSystemTest, initialization_test) {
    AMSSystemInstance::destroy();
    AMSSystemInstance::create(HEARTBEAT_INTERVAL_MS, PACK_CHARGE_CRITICAL_THRESHOLD_VOLTS, CELL_CHARGE_CRITICAL_THRESHOLD_VOLTS);
    AMSSystem &ams = AMSSystemInstance::instance();

    VCRData_s vcr_data = {};
    vcr_data.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.4f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 3.5f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 460.0f;

    // Create expected result of AMS data
    AMSSystemData_s expected_data = {
        .min_cell_voltage = 0.0f,
        .average_cell_voltage = 0.0f,
        .max_cell_voltage = 0.0f,
        .min_temp_celsius = 0.0f,
        .average_temp_celsius = 0.0f,
        .max_temp_celsius = 0.0f,
        .total_pack_voltage = 0.0f,
        .ams_ok = true
    };

    AMSSystemData_s actual_data = ams.update_ams_system(init_millis, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, vcr_data.system_data.ams_data);

    actual_data = ams.update_ams_system(init_millis + 1000, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, vcr_data.system_data.ams_data);

    actual_data = ams.update_ams_system(init_millis + 10000, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, vcr_data.system_data.ams_data);

}

TEST (AMSSystemTest, valid_initialization_test) {
    AMSSystemInstance::destroy();
    AMSSystemInstance::create(HEARTBEAT_INTERVAL_MS, PACK_CHARGE_CRITICAL_THRESHOLD_VOLTS, CELL_CHARGE_CRITICAL_THRESHOLD_VOLTS);
    AMSSystem &ams = AMSSystemInstance::instance();

    VCRData_s vcr_data = {};
    vcr_data.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.4f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 3.5f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 460.0f;
    vcr_data.interface_data.stamped_acu_core_data.last_recv_millis = init_millis;

    // Create expected result of AMS data
    AMSSystemData_s expected_data = {
        .min_cell_voltage = 3.4f,
        .average_cell_voltage = 3.5f,
        .max_cell_voltage = -1.0f,
        .min_temp_celsius = -1.0f,
        .average_temp_celsius = -1.0f,
        .max_temp_celsius = 50.0f,
        .total_pack_voltage = 460.0f,
        .ams_ok = true
    };

    AMSSystemData_s actual_data = ams.update_ams_system(init_millis + 10, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, actual_data);

}

TEST (AMSSystemTest, heartbeat_overrun_shutdown) {
    AMSSystemInstance::destroy();
    AMSSystemInstance::create(HEARTBEAT_INTERVAL_MS, PACK_CHARGE_CRITICAL_THRESHOLD_VOLTS, CELL_CHARGE_CRITICAL_THRESHOLD_VOLTS);
    AMSSystem &ams = AMSSystemInstance::instance();

    VCRData_s vcr_data = {};
    vcr_data.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.4f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 3.5f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 460.0f;
    vcr_data.interface_data.stamped_acu_core_data.last_recv_millis = init_millis;

    // Create expected result of AMS data
    AMSSystemData_s expected_data = {
        .min_cell_voltage = 3.4f,
        .average_cell_voltage = 3.5f,
        .max_cell_voltage = -1.0f,
        .min_temp_celsius = -1.0f,
        .average_temp_celsius = -1.0f,
        .max_temp_celsius = 50.0f,
        .total_pack_voltage = 460.0f,
        .ams_ok = true
    };

    AMSSystemData_s actual_data = ams.update_ams_system(init_millis + 10, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, actual_data);

    actual_data = ams.update_ams_system(init_millis + 500, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, actual_data);

    actual_data = ams.update_ams_system(init_millis + 1500, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, actual_data);

    actual_data = ams.update_ams_system(init_millis + 1900, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, actual_data);

    actual_data = ams.update_ams_system(init_millis + 2001, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    expected_data.ams_ok = false;
    ASSERT_AMS_SYSTEM_DATA_EQ(expected_data, actual_data);

}

TEST (AMSSystemTest, cell_under_voltage_shutdown) {
    AMSSystemInstance::destroy();
    AMSSystemInstance::create(HEARTBEAT_INTERVAL_MS, PACK_CHARGE_CRITICAL_THRESHOLD_VOLTS, CELL_CHARGE_CRITICAL_THRESHOLD_VOLTS);
    AMSSystem &ams = AMSSystemInstance::instance();

    VCRData_s vcr_data = {};
    vcr_data.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.1f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 3.5f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 460.0f;
    vcr_data.interface_data.stamped_acu_core_data.last_recv_millis = init_millis;

    AMSSystemData_s actual_data = ams.update_ams_system(init_millis + 10, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_EQ(true, actual_data.ams_ok);

    vcr_data.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.05f;
    actual_data = ams.update_ams_system(init_millis + 20, vcr_data);
    ASSERT_EQ(true, actual_data.ams_ok);

    vcr_data.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 2.99f;
    actual_data = ams.update_ams_system(init_millis + 30, vcr_data);
    ASSERT_EQ(false, actual_data.ams_ok);

}

TEST (AMSSystemTest, pack_under_voltage_shutdown) {
    AMSSystemInstance::destroy();
    AMSSystemInstance::create(HEARTBEAT_INTERVAL_MS, PACK_CHARGE_CRITICAL_THRESHOLD_VOLTS, CELL_CHARGE_CRITICAL_THRESHOLD_VOLTS);
    AMSSystem &ams = AMSSystemInstance::instance();

    VCRData_s vcr_data = {};
    vcr_data.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = 3.1f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = 3.5f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = 50.0f;
    vcr_data.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 460.0f;
    vcr_data.interface_data.stamped_acu_core_data.last_recv_millis = init_millis;

    AMSSystemData_s actual_data = ams.update_ams_system(init_millis + 10, vcr_data);
    vcr_data.system_data.ams_data = actual_data;
    ASSERT_EQ(true, actual_data.ams_ok);

    vcr_data.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 421;
    actual_data = ams.update_ams_system(init_millis + 20, vcr_data);
    ASSERT_EQ(true, actual_data.ams_ok);

    vcr_data.interface_data.stamped_acu_core_data.acu_data.pack_voltage = 419;
    actual_data = ams.update_ams_system(init_millis + 30, vcr_data);
    ASSERT_EQ(false, actual_data.ams_ok);
}