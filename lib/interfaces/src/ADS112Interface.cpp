#include "ADS112Interface.h"


ADS112Interface::ADS112Interface(
    HardwareSerial& serial,
    const float scales[NUM_CHANNELS],
    const float offsets[NUM_CHANNELS],
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

    delayMicroseconds(_timing.power_up_delay_us);

    _reset_adc();
    _configure_adc();
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

    delayMicroseconds(_timing.reset_delay_us);
}

void ADS112Interface::_configure_adc()
{
    _write_register(0x00, _build_config_register_0(_config.muxes.at(0)));
    _write_register(0x01, _build_config_register_1());
    _write_register(0x02, 0x00);
    _write_register(0x03, 0x00);
    _write_register(0x04, 0x48);
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

    _serial->write(_commands.sync_word);
    _serial->write(command);
    _serial->write(value);
    _serial->flush();
}

uint8_t ADS112Interface::_read_register(uint8_t register_address)
{
    const uint8_t command = static_cast<uint8_t>(
        _commands.rreg_base_command |
        ((register_address & 0x07U) << 1U)
    );

    _clear_serial_rx_buffer();

    _serial->write(_commands.sync_word);
    _serial->write(command);
    _serial->flush();

    const uint32_t start_time_ms = millis();

    while (_serial->available() < 1)
    {
        if ((millis() - start_time_ms) > _timing.read_timeout_ms)
        {
            return 0x00;
        }
    }

    return static_cast<uint8_t>(_serial->read());
}

uint8_t ADS112Interface::_build_config_register_0(ADS112U04InputMux_e mux) const
{
    const uint8_t mux_bits = static_cast<uint8_t>(mux);
    const uint8_t gain_bits = static_cast<uint8_t>(_config.gain);

    constexpr const uint8_t pga_bypass_enabled = 0x01;

    return static_cast<uint8_t>(
        (mux_bits << 4U) |
        (gain_bits << 1U) |
        pga_bypass_enabled
    );
}

uint8_t ADS112Interface::_build_config_register_1() const
{
    const uint8_t data_rate_bits = static_cast<uint8_t>(_config.data_rate);
    const uint8_t reference_bits = static_cast<uint8_t>(_config.reference);

    constexpr const uint8_t normal_mode = 0x00;
    constexpr const uint8_t single_shot_mode = 0x00;
    constexpr const uint8_t temperature_sensor_disabled = 0x00;

    return static_cast<uint8_t>(
        (data_rate_bits << 5U) |
        (normal_mode << 4U) |
        (single_shot_mode << 3U) |
        (reference_bits << 1U) |
        temperature_sensor_disabled
    );
}

void ADS112Interface::_send_command(uint8_t command)
{
    _serial->write(_commands.sync_word);
    _serial->write(command);
    _serial->flush();
}

void ADS112Interface::_clear_serial_rx_buffer()
{
    while (_serial->available() > 0)
    {
        static_cast<void>(_serial->read());
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

    _send_command(_commands.rdata_command);

    const uint32_t start_time_ms = millis();

    while (_serial->available() < 2)
    {
        if ((millis() - start_time_ms) > _timing.read_timeout_ms)
        {
            return false;
        }
    }

    const uint8_t lsb = static_cast<uint8_t>(_serial->read());
    const uint8_t msb = static_cast<uint8_t>(_serial->read());

    const uint16_t raw_word = static_cast<uint16_t>(
        (static_cast<uint16_t>(msb) << 8U) |
        static_cast<uint16_t>(lsb)
    );

    raw_adc_count = static_cast<int16_t>(raw_word);

    return true;
}