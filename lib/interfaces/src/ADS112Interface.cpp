#include "ADS112Interface.h"

// At 115200 baud, one byte (10 bits with start/stop) takes ~86.8us.
// We wait 2 full byte periods between each byte to give the ADC time
// to process the sync word and latch commands on the stop bit.
static constexpr uint32_t INTER_BYTE_DELAY_US = 200U;

ADS112Interface::ADS112Interface(
    HardwareSerial& serial,
    const std::array<float, NUM_CHANNELS>& scales,
    const std::array<float, NUM_CHANNELS>& offsets,
    ADS112Pinout_s pinouts,
    ADS112Config_s config,
    ADS112Commands_s commands,
    ADS112Timing_s timing,
    uint32_t baud_rate
) :
    _serial(&serial),
    _pinouts(pinouts),
    _config(config),
    _commands(commands),
    _timing(timing),
    _baud_rate(baud_rate)
{
    for (int channel_index = 0; channel_index < NUM_CHANNELS; channel_index++)
    {
        this->_channels[channel_index] = AnalogChannel();
        this->setChannelScaleAndOffset(
            channel_index,
            scales[channel_index],
            offsets[channel_index]
        );
    }
}

void ADS112Interface::init()
{
    if (_has_reset_pin())
    {
        pinMode(_pinouts.reset_pin, OUTPUT);
        digitalWrite(_pinouts.reset_pin, HIGH);
    }

    if (_has_drdy_pin())
    {
        pinMode(_pinouts.drdy_pin, INPUT);
    }

    _serial->begin(_baud_rate);

    // Give the serial peripheral time to fully initialise before
    // transmitting anything.
    delay(100);
    _clear_serial_rx_buffer();

    Serial.print("Serial port started at ");
    Serial.println(_baud_rate);

    // Power-up delay per datasheet section 8.4.1.1: minimum 600 us
    // after both supplies exceed their reset thresholds.
    delayMicroseconds(_timing.power_up_delay_us);

    _reset_adc();

    // Verify the ADC is alive: after reset all registers return 0x00.
    Serial.println("Reading Config Register 0 after reset (expect 0x00)...");
    uint8_t reg0 = _read_register(0x00);
    Serial.print("Config Reg 0 after reset: 0x");
    Serial.println(reg0, HEX);

    if (reg0 != 0x00)
    {
        Serial.println("WARNING: unexpected reset value - UART communication may be broken.");
    }

    _configure_adc();

    // Read back all five registers to confirm writes succeeded.
    Serial.println("Register readback after configuration:");
    for (uint8_t i = 0; i <= 4; i++)
    {
        Serial.print("  Reg");
        Serial.print(i);
        Serial.print(": 0x");
        Serial.println(_read_register(i), HEX);
    }

    // Print what we expected to write so it is easy to compare.
    Serial.print("  Expected Reg0: 0x");
    Serial.println(_build_config_register_0(_config.muxes.at(0)), HEX);
    Serial.print("  Expected Reg1: 0x");
    Serial.println(_build_config_register_1(), HEX);

    start_conversions();
}

void ADS112Interface::tick()
{
    _sample();
    this->_convert();
}

AnalogConversion_s ADS112Interface::get_channel(size_t channel)
{
    return this->data.conversions.at(channel);
}

bool ADS112Interface::is_data_ready()
{
    if (!_has_drdy_pin())
    {
        return true;
    }

    return digitalRead(_pinouts.drdy_pin) == LOW;
}

void ADS112Interface::start_conversions()
{
    _send_command(_commands.start_sync_command);
}

void ADS112Interface::power_down()
{
    _send_command(_commands.powerdown_command);
}

void ADS112Interface::sample_internal_temperature()
{
    // Enable internal temperature sensor mode.
    // In this mode config register 0 has no effect and the ADC uses
    // the internal reference.
    _write_register(0x01, _build_config_register_1(true));

    start_conversions();

    if (!_wait_for_conversion())
    {
        _internal_temperature.data_valid = false;
        _write_register(0x01, _build_config_register_1(false));
        return;
    }

    int16_t raw_adc_count = 0;
    const bool read_successful = _read_raw_adc_count(raw_adc_count);

    if (!read_successful)
    {
        _internal_temperature.data_valid = false;
        _write_register(0x01, _build_config_register_1(false));
        return;
    }

    const uint16_t raw_word = static_cast<uint16_t>(raw_adc_count);

    // Temperature data is a 14-bit two's-complement value left-justified
    // in the 16-bit conversion word.  Drop the two unused LSBs.
    uint16_t raw_temperature_14_bit = static_cast<uint16_t>(raw_word >> 2U);

    // Sign-extend the 14-bit value into 16 bits.
    if ((raw_temperature_14_bit & _config.temp_sign_bit_mask) != 0U)
    {
        raw_temperature_14_bit = static_cast<uint16_t>(
            raw_temperature_14_bit | _config.temp_sign_extend_mask
        );
    }

    const int16_t signed_temperature_count = static_cast<int16_t>(raw_temperature_14_bit);

    _internal_temperature.raw_temperature_count = signed_temperature_count;
    _internal_temperature.temperature_c =
        static_cast<float>(signed_temperature_count) * _config.internal_temp_scale;
    _internal_temperature.data_valid = true;

    // Restore normal analog input mode.
    _write_register(0x01, _build_config_register_1(false));
}

ADS112TemperatureReading_s ADS112Interface::get_internal_temperature()
{
    return _internal_temperature;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void ADS112Interface::_sample()
{
    for (int channel_index = 0; channel_index < NUM_CHANNELS; channel_index++)
    {
        _configure_mux(_config.muxes.at(channel_index));

        start_conversions();

        if (!_wait_for_conversion())
        {
            continue;
        }

        int16_t raw_adc_count = 0;
        const bool read_successful = _read_raw_adc_count(raw_adc_count);

        if (!read_successful)
        {
            continue;
        }

        this->_channels[channel_index].lastSample = raw_adc_count;
    }
}

void ADS112Interface::_reset_adc()
{
    if (_has_reset_pin())
    {
        digitalWrite(_pinouts.reset_pin, LOW);
        delayMicroseconds(1);
        digitalWrite(_pinouts.reset_pin, HIGH);
        delayMicroseconds(_timing.reset_delay_us);
    }

    _send_command(_commands.reset_command);

    // Datasheet section 8.4.1.3: after the RESET command is latched,
    // wait at least td(RSRX) = 80 us before sending the next command.
    // We use 10 ms to be conservative and ensure the internal reference
    // and oscillator are fully settled.
    delay(10);

    _clear_serial_rx_buffer();
}

void ADS112Interface::_configure_adc()
{
    _write_register(0x00, _build_config_register_0(_config.muxes.at(0)));
    _write_register(0x01, _build_config_register_1());
    _write_register(0x02, 0x00);
    _write_register(0x03, 0x00);
    _write_register(0x04, _config.config_reg_4_default_value);
}

void ADS112Interface::_configure_mux(ADS112U04InputMux_e mux)
{
    _write_register(0x00, _build_config_register_0(mux));
}

void ADS112Interface::_write_register(uint8_t register_address, uint8_t value)
{
    const uint8_t command = static_cast<uint8_t>(
        _commands.wreg_base_command |
        ((register_address & 0x07U) << 1U)
    );

    // Each byte is followed by a flush-and-delay so the ADC has time to:
    //   1. Detect the baud rate from the sync word.
    //   2. Decode and latch the command on the stop bit.
    //   3. Process the data byte before the next transaction begins.
    _serial->write(_commands.sync_word);
    _serial->flush();
    delayMicroseconds(INTER_BYTE_DELAY_US);

    _serial->write(command);
    _serial->flush();
    delayMicroseconds(INTER_BYTE_DELAY_US);

    _serial->write(value);
    _serial->flush();
    delayMicroseconds(INTER_BYTE_DELAY_US);
}

uint8_t ADS112Interface::_read_register(uint8_t register_address)
{
    const uint8_t command = static_cast<uint8_t>(
        _commands.rreg_base_command |
        ((register_address & 0x07U) << 1U)
    );

    _clear_serial_rx_buffer();

    _serial->write(_commands.sync_word);
    _serial->flush();
    delayMicroseconds(INTER_BYTE_DELAY_US);

    _serial->write(command);
    _serial->flush();

    // Wait for the ADC to transmit the register byte back.
    const uint32_t start_time_ms = millis();
    while (_serial->available() < 1)
    {
        if ((millis() - start_time_ms) > _timing.read_timeout_ms)
        {
            Serial.println("_read_register: TIMEOUT");
            return 0x00;
        }
    }

    return static_cast<uint8_t>(_serial->read());
}

uint8_t ADS112Interface::_build_config_register_0(ADS112U04InputMux_e mux) const
{
    const uint8_t mux_bits  = static_cast<uint8_t>(mux);
    const uint8_t gain_bits = static_cast<uint8_t>(_config.gain);

    // PGA must be bypassed for single-ended (AINx vs AVSS) measurements.
    // The MUX enum values 0x08-0x0B all have AINN = AVSS, so bypass is
    // always appropriate here.
    constexpr uint8_t pga_bypass_enabled = 0x01U;

    return static_cast<uint8_t>(
        (mux_bits  << 4U) |
        (gain_bits << 1U) |
        pga_bypass_enabled
    );
}

uint8_t ADS112Interface::_build_config_register_1(bool temperature_sensor_enabled) const
{
    const uint8_t data_rate_bits  = static_cast<uint8_t>(_config.data_rate);
    const uint8_t reference_bits  = static_cast<uint8_t>(_config.reference);

    constexpr uint8_t normal_mode   = 0x00U;
    constexpr uint8_t single_shot   = 0x00U;
    const     uint8_t temp_sensor   = temperature_sensor_enabled ? 0x01U : 0x00U;

    return static_cast<uint8_t>(
        (data_rate_bits << DATA_RATE_SHIFT)   |
        (normal_mode    << NORMAL_MODE_SHIFT) |
        (single_shot    << SINGLE_SHOT_SHIFT) |
        (reference_bits << REFERENCE_SHIFT)   |
        temp_sensor
    );
}

void ADS112Interface::_send_command(uint8_t command)
{
    _serial->write(_commands.sync_word);
    _serial->flush();
    delayMicroseconds(INTER_BYTE_DELAY_US);

    _serial->write(command);
    _serial->flush();
    delayMicroseconds(INTER_BYTE_DELAY_US);
}

void ADS112Interface::_clear_serial_rx_buffer()
{
    while (_serial->available() > 0)
    {
        _serial->read();
    }
}

bool ADS112Interface::_has_reset_pin() const
{
    return _pinouts.reset_pin >= 0;
}

bool ADS112Interface::_has_drdy_pin() const
{
    return _pinouts.drdy_pin >= 0;
}

bool ADS112Interface::_wait_for_conversion()
{
    if (!_has_drdy_pin())
    {
        delayMicroseconds(_timing.conversion_delay_us);
        return true;
    }

    const uint32_t start_time_ms = millis();
    while (!is_data_ready())
    {
        if ((millis() - start_time_ms) > _timing.read_timeout_ms)
        {
            return false;
        }
    }

    return true;
}

bool ADS112Interface::_read_raw_adc_count(int16_t& raw_adc_count)
{
    _clear_serial_rx_buffer();

    _serial->write(_commands.sync_word);
    _serial->flush();
    delayMicroseconds(INTER_BYTE_DELAY_US);

    _serial->write(_commands.rdata_command);
    _serial->flush();

    // The ADS112U04 transmits data LSB byte first, then MSB byte
    // (see datasheet section 8.5.3.4 and Figure 62).
    const uint32_t start_time_ms = millis();
    while (_serial->available() < 2)
    {
        if ((millis() - start_time_ms) > _timing.read_timeout_ms)
        {
            Serial.println("_read_raw_adc_count: TIMEOUT");
            return false;
        }
    }

    const uint8_t lsb = static_cast<uint8_t>(_serial->read());
    const uint8_t msb = static_cast<uint8_t>(_serial->read());

    const uint16_t raw_word = static_cast<uint16_t>(
        (static_cast<uint16_t>(msb) << 8U) | lsb
    );

    raw_adc_count = static_cast<int16_t>(raw_word);

    return true;
}