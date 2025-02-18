
## Drivetrain System (tentative design docs)

### list of features

- [ ] ability to initialize inverters 
- [ ] ability to command inverters 
- [ ] ability to switch inverter control modes
- [ ] ability to detect timeout of initialization of inverters
- [ ] ability to change parameters of inverters
- [ ] detailed error status for invalid usage of drivetrain system
- [ ] ability to get all data available from inverter

### interface / usage description

the user of the drivetrain system will be expected to use the `evaluate_drivetrain` function to interact / command and receive the current status of the drivetrain. 

#### initialization
the user shall call the `evaluate_drivetrain` function with the cmdvariant type set to the initialization struct and populated with desired mode to put the drivetrain into. the user should continuously call the `evaluate_drivetrain` with this struct until the drivetrain's state reaches the initialization state expected.

#### drivetrain commanding

once initialized, the user is able to command the drivetrain with either speed or torque commands. 

The user may switch between command modes so long as the drivetrain is not active / the car is not driving (all inverter RPMs < 100). this is purely for safety, however this needs to be tested for the exact requirements to switch between control modes during runtime.

### state machine description

Instead of the drivetrain system being a direct api interface on top of the inverter interface with calls to the drivetrain system that call the lower-level inverter interface immediately to queue CAN messages, the drivetrain system will instead update pieces of the inverter interface's internal state (such as bit flags) for it to send periodically.

This is being done to simplify the interaction between the inverters and the rest of our firmware and to resolve the CAN saturation issues that were occuring with the last drivetrain system / inverter interface issues that were hackily solved with a metro timer dictating when the queue of CAN messages that were being sent out from the inverter could be appended to.


```
---
title: drivetrain state machine
---
stateDiagram-v2

    nc : NOT_CONNECTED 
    not_en : NOT_ENABLED_NO_HV_PRESENT
    not_en_hv : NOT_ENABLED_HV_PRESENT
    ready : INVERTERS_READY 
    hv_en : INVERTERS_HV_ENABLED
    en : INVERTERS_ENABLED
    drive_mode : ENABLED_DRIVE_MODE
    err: ERROR
    clear_err: CLEARING_ERRORS
    note2: note that any state other than NOT_CONNECTED can go to the ERROR state if in any of the states any inverter has an error flag present
    
    note right of err
        during this state all setpoints will be set to 0 and the inverter control word will have the inverter_enable flag set to false
    end note

    note right of clear_err
        during this state all setpoints will be set to 0, (inverter_enable: false, hv_enable: true, driver_enable: true, remove_error: true)
    end note

    note right of drive_mode
        during this mode the drivetrain is free to receive speed commands and car will move
    end note

    
    nc --> not_en: received data from all inverters
    nc --> not_en_hv: received data from all inverters AND the inverters have a voltage above HV level
    
    not_en --> not_en_hv: HV present to all inverters
    not_en_hv --> ready: all inverters have their ready flag set

    hv_en --> ready: on quit dc flag off

    ready --> hv_en: requesting initialization of drivetrain AND all inverters have their quit dc flag on
    ready --> not_en_hv: if any of the inverters have their ready flags not set
    hv_en --> en: on set of all inverters enabled flags
    en --> hv_en: on inverter enabled flag off

    en --> drive_mode: on command request of drive mode
    drive_mode --> err: on error during operation
    drive_mode --> err: incorrect user command type OR any inverter error
    err --> clear_err: on user request of error reset 
    clear_err --> not_en_hv: on successful reset of errors (either internally to the drivetrain system or of the inverters themselves)
```