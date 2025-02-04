## Inverter Interface


### GPIO connections


### CAN parameter / message descriptions

#### VCR output / control messages:

below is the description of the control messages that will be configured within the free CAN configuration of the AMKs. these are subject to change, however it is primarily a modification of the "fixed CAN commnicaiton" protocol that the AMK kit ships with / is documented 

- Control Word (will be sent every 50ms, monitored by the inverter to be expected at least every 75ms)
    - `bool inverter_enable` 
        - AKA (`AMK_bInverterOn`)
    - `bool hv_enable` 
        - AKA (`AMK_bDcOn`)
    - `bool driver_enable`
        - AKA (`AMK_bEnable`)
    - `bool remove_error` 
        - AKA (`AMK_bErrorReset`)

- Control Input Message (will be attempted to be sent every 5ms, monitored by the inverter to be expected at least every 20ms)
    - `int16_t speed_setpoint` 
        - __NOTE: active only when in speed control mode__
        - AKA (`AMK_TargetVelocity`) / Special Signal index 6 / AKA 16-bit version of SERCOS parameter ID36 (unknown why this is 16 bit vs the 32 bit SERCOS parameter in manual)
        - the RPM setpoint for the inverter in units of RPM.
    - `int16_t positive_torque_limit` 
        - AKA (`AMK_TorqueLimitPositiv`) / SERCOS parameter ID82 / Special Signal index 13
        - positive torque limit for reaching the desired rpm in units of 0.1% of the nominal torque (=9.8Nm)(known as Mn in docs). 

            EXAMPLE: `positive_torque_limit=1000` means that the positive torque limit is set to 100% of the nominal torque, so the actual torque limit being set by this is equal to 9.8Nm, and 200% would be equal to 19.6Nm.
    - `int16_t negative_torque_limit`
        - AKA (`AMK_TorqueLimitNegativ`) / SERCOS parameter ID83 / Special Signal index 14
        - `positive_torque_limit` with a negative sign.
    - `int16_t torque_setpoint` 
        - __NOTE: active only when in torque control mode.__
        - AKA SERCOS parameter ID80 / Special Signal index 17
        - the torque setpoint for the inverter in units of percentage as described in the `positive_torque_limit` message member above

- Control parameter message (will be sent intermitently, not monitored by the inverter to be expected at any regular period)
    - `uint16_t speed_control_kp`
        - P for the speed control mode
        - AKA SERCOS parameter ID100
    - `uint16_t speed_control_ki`
        - I parameter to be set internally on the inverter
        - AKA SERCOS parameter ID101
    - `uint16_t speed_control_kd`
        - D parameter to be set internally on the inverter
        - AKA SERCOS parameter ID102

#### inverter messages

- inverter status / ( `AMK_Status` (16 bit) +  + DC bus voltage (16 bit) + `AMK_ErrorInfo` (16 bit kinda(?))) (will be periodically sent from the inverter every 20ms)
    - `bool system_ready`
        - AKA (`AMK_bSystemReady`) / bit 9 within Special Signal Status word formula student (System Ready (SBM)) / 
        - displays when the system is error free / ready to be initialized
    - `bool error`
        - AKA (`AMK_bError`) / bit 10 within Special Signal Status word formula student 
        - displays that an error is present
    - `bool warning`
        - AKA (`AMK_bWarn`) / bit 11
        - warning present (such as if derating is on)
    - `bool quit_dc_on`
        - AKA (`AMK_bQuitDcOn`) / bit 12
        - HV activation acknowledgment / verification (says whether or not HV is ACTUALLY present to the inverter)
    - `bool dc_on`
        - AKA (`AMK_bDcOn`) / bit 13
        - mirrors / is feedback of what is set by the VCR for its `hv_enable`
    - `bool quit_inverter_on`
        - AKA (`AMK_bQuitInverterOn`) / bit 14
        - controller is enabled
    - `bool inverter_on`
        - AKA (`AMK_bInverterOn`) / bit 15
        - mirror / feedback of `inverter_enable` set by the control word
    - `bool derating_on`
        - AKA (`AMK_bDerating`) / bit 16
        - says whether or not derating is active (torque derating)
    - `uint16_t dc_bus_voltage`
        - AKA SERCOS Paramter ID32836
        - actual DC bus voltage 
    - `uint16_t diagnostic_number`
        - AKA (`AMK_ErrorInfo`)

- inverter temps (will be periodically sent from the inverter every 20ms)
    - `int16_t motor_temp`
        - temperature of the motor in units of 0.1 degrees celcius
        - AKA (`AMK_TempMotor`) / Special Signal index 7 / SERCOS parameter ID33117 
    - `int16_t inverter_temp`
        - temp of the inverter cold plate in units of 0.1 degrees celcius
        - AKA (`AMK_TempInverter`) / Special Signal index 8 / SERCOS parameter ID33116

            - details: "ID33116 shows the temperature of the cold plate (heat sink of the IGBT and at the same time of the rear wall of the device).
The triggering thresholds are device-specific, are set in the SEEP (Device-internal memory, serial EEPROM) at the factory and cannot be changed by the user.
If critical temperatures occur for the devices, the warning 2350 'Device temperature warning' is generated as well as the error
message 2346 'Converter temperature error' after the warning time1) (ID32943) has expired."
    - `int16_t igbt_temp`
        - temp of the IGBTs in units of 0.1 degrees celcius
        - AKA (`AMK_TempIGBT`) / Special Signal index 27 / SERCOS parameter ID34215

- inverter dynamics data (will be periodically sent from the inverter every 5ms)
    - `uint32_t actual_power_w`
        - mechanical motor power in watts (from actual torque and actual speed)
        - AKA SERCOS parameter ID33100
    - `int16_t actual_torque_nm`
        - actual torque value in the same units as `positive_torque_limit` 
        - AKA Special Signal index 19 / SERCOS parameter ID84
    - `int16_t actual_speed_rpm`
        - motor speed in units of rpm
        - AKA Special Signal value index 6

- inverter electrical power data (will be periodically sent from the inverter every 5ms)
    - `int32_t active_power_w`
        - electrical power (use / creation) in watts of the inverter
        - can be negative when in regen / positive when driving motor
        - AKA SERCOS parameter ID33171
    - `int32_t reactive_power_var` 
        - electrical reactive power (inductive or capacitive)
        - positive value = inductive consumer, negative value = capacitive consumer
        - AKA SERCOS parameter ID33172
    
- inverter parameter feedback (inverter's speed PID vals) (will be sent periodically from the inverter every 20ms)
    - `uint16_t speed_control_kp`
        - inverter's internal value of kp
        - AKA SERCOS parameter ID100
    - `uint16_t speed_control_ki`
        - inverter's internal value of (TN) ki
        - AKA SERCOS parameter ID101
    - `uint16_t speed_control_kd`
        - inverter's internal value of (TD) kd
        - AKA SERCOS parameter ID102