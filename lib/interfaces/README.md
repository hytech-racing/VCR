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
        
