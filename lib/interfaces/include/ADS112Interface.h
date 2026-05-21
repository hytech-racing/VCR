#ifndef ADS112INTERFACE_H
#define ADS112INTERFACE_H

#include <Arduino.h>
#include <array>
#include <cstdint>

#include "AnalogSensorsInterface.h"
#include "etl/singleton.h"

namespace ads112_default_parameters
{
    constexpr const int ADS112U04_NUM_CHANNELS = 4;
    constexpr const int ADS112U04_PIN_UNUSED = -1;

    constexpr const uint32_t ADS112U04_BAUDRATE = 115200;

    constexpr const uint8_t ADS112U04_SYNC_WORD          = 0x55;
    constexpr const uint8_t ADS112U04_RESET_COMMAND      = 0x06;
    constexpr const uint8_t ADS112U04_START_SYNC_COMMAND = 0x08;
    constexpr const uint8_t ADS112U04_POWERDOWN_COMMAND  = 0x02;
    constexpr const uint8_t ADS112U04_RDATA_COMMAND      = 0x10;

    constexpr const uint8_t ADS112U04_WREG_BASE_COMMAND = 0x40;
    constexpr const uint8_t ADS112U04_RREG_BASE_COMMAND = 0x20;

    constexpr const uint32_t ADS112U04_POWER_UP_DELAY_US = 600;
    constexpr const uint32_t ADS112U04_RESET_DELAY_US    = 80;
    constexpr const uint32_t ADS112U04_READ_TIMEOUT_MS   = 5;

    /*
     * Fallback conversion wait when DRDY is not connected.
     * 330 SPS is about 3.03 ms per conversion, so 4000 us is conservative.
     */
    constexpr const uint32_t ADS112U04_CONVERSION_DELAY_US = 4000;

    constexpr const float ADS112U04_REFERENCE_VOLTAGE = 5.0f;
    constexpr const float ADS112U04_GAIN              = 1.0f;
    constexpr const float ADS112U04_FULL_SCALE_COUNTS = 32768.0f;
}

enum class ADS112U04InputMux_e : uint8_t
{
    AIN0_AVSS = 0x08,
    AIN1_AVSS = 0x09,
    AIN2_AVSS = 0x0A,
    AIN3_AVSS = 0x0B
};

enum class ADS112U04Gain_e : uint8_t
{
    GAIN_1   = 0x00,
    GAIN_2   = 0x01,
    GAIN_4   = 0x02,
    GAIN_8   = 0x03,
    GAIN_16  = 0x04,
    GAIN_32  = 0x05,
    GAIN_64  = 0x06,
    GAIN_128 = 0x07
};

enum class ADS112U04DataRate_e : uint8_t
{
    SPS_20   = 0x00,
    SPS_45   = 0x01,
    SPS_90   = 0x02,
    SPS_175  = 0x03,
    SPS_330  = 0x04,
    SPS_600  = 0x05,
    SPS_1000 = 0x06
};

enum class ADS112U04Reference_e : uint8_t
{
    INTERNAL_2V048     = 0x00,
    EXTERNAL_REFP_REFN = 0x01,
    ANALOG_SUPPLY      = 0x02
};

struct ADS112Pinout_s
{
    int reset_pin;
    int drdy_pin;
};

struct ADS112Config_s
{
    std::array<ADS112U04InputMux_e, ads112_default_parameters::ADS112U04_NUM_CHANNELS> muxes;
    ADS112U04Gain_e gain;
    ADS112U04DataRate_e data_rate;
    ADS112U04Reference_e reference;
    float reference_voltage;
    float gain_value;
    float full_scale_counts;
};

struct ADS112Commands_s
{
    uint8_t sync_word;
    uint8_t reset_command;
    uint8_t start_sync_command;
    uint8_t powerdown_command;
    uint8_t rdata_command;
    uint8_t wreg_base_command;
    uint8_t rreg_base_command;
};

struct ADS112Timing_s
{
    uint32_t power_up_delay_us;
    uint32_t reset_delay_us;
    uint32_t read_timeout_ms;
    uint32_t conversion_delay_us;
};

class ADS112Interface : public AnalogMultiSensor<ads112_default_parameters::ADS112U04_NUM_CHANNELS>
{
public:
    static constexpr int NUM_CHANNELS = ads112_default_parameters::ADS112U04_NUM_CHANNELS;

    ADS112Interface(
        HardwareSerial& serial,
        const float scales[NUM_CHANNELS],
        const float offsets[NUM_CHANNELS],
        ADS112Pinout_s pinouts = {
            ads112_default_parameters::ADS112U04_PIN_UNUSED,
            ads112_default_parameters::ADS112U04_PIN_UNUSED
        },
        ADS112Config_s config = {
            {
                ADS112U04InputMux_e::AIN0_AVSS,
                ADS112U04InputMux_e::AIN1_AVSS,
                ADS112U04InputMux_e::AIN2_AVSS,
                ADS112U04InputMux_e::AIN3_AVSS
            },
            ADS112U04Gain_e::GAIN_1,
            ADS112U04DataRate_e::SPS_330,
            ADS112U04Reference_e::ANALOG_SUPPLY,
            ads112_default_parameters::ADS112U04_REFERENCE_VOLTAGE,
            ads112_default_parameters::ADS112U04_GAIN,
            ads112_default_parameters::ADS112U04_FULL_SCALE_COUNTS
        },
        ADS112Commands_s commands = {
            ads112_default_parameters::ADS112U04_SYNC_WORD,
            ads112_default_parameters::ADS112U04_RESET_COMMAND,
            ads112_default_parameters::ADS112U04_START_SYNC_COMMAND,
            ads112_default_parameters::ADS112U04_POWERDOWN_COMMAND,
            ads112_default_parameters::ADS112U04_RDATA_COMMAND,
            ads112_default_parameters::ADS112U04_WREG_BASE_COMMAND,
            ads112_default_parameters::ADS112U04_RREG_BASE_COMMAND
        },
        ADS112Timing_s timing = {
            ads112_default_parameters::ADS112U04_POWER_UP_DELAY_US,
            ads112_default_parameters::ADS112U04_RESET_DELAY_US,
            ads112_default_parameters::ADS112U04_READ_TIMEOUT_MS,
            ads112_default_parameters::ADS112U04_CONVERSION_DELAY_US
        },
        uint32_t baud_rate = ads112_default_parameters::ADS112U04_BAUDRATE
    );

    void init();

    void tick() override;

    AnalogConversion_s get_channel(size_t channel);

    bool is_data_ready();

    void start_conversions();

    void power_down();

private:
    HardwareSerial* _serial;

    ADS112Pinout_s _pinouts;
    ADS112Config_s _config;
    ADS112Commands_s _commands;
    ADS112Timing_s _timing;

    uint32_t _baud_rate;

    void _sample() override;

    void _reset_adc();
    void _configure_adc();

    void _configure_mux(ADS112U04InputMux_e mux);

    void _write_register(uint8_t register_address, uint8_t value);
    uint8_t _read_register(uint8_t register_address);

    uint8_t _build_config_register_0(ADS112U04InputMux_e mux) const;
    uint8_t _build_config_register_1() const;

    void _send_command(uint8_t command);
    void _clear_serial_rx_buffer();

    bool _has_reset_pin() const;
    bool _has_drdy_pin() const;

    bool _wait_for_conversion();
    bool _read_raw_adc_count(int16_t& raw_adc_count);
};

using ADS112InterfaceInstance = etl::singleton<ADS112Interface>;

#endif /* ADS112INTERFACE_H */