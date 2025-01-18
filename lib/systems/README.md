
## Drivetrain System (tentative design docs)

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

```mermaid
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
    en_sp_mode : ENABLING_INVERTERS_SPEED_MODE
    en_torq_mode : ENABLING_INVERTERS_TORQUE_MODE
    speed_mode : ENABLED_SPEED_MODE
    torq_mode : ENABLED_TORQUE_MODE
    err: ERROR
    clear_err: CLEARING_ERRORS

    note right of err
        during this state the inverters will be not be sent any control setpoints
    end note

    nc --> not_en: received data from all inverters
    nc --> not_en_hv: received data from all inverters AND the inverters have a voltage above HV level
    not_en --> not_en_hv: HV present to all inverters
    not_en_hv --> ready: all inverters have their ready flag set
    ready --> hv_en: requesting initialization of drivetrain AND all inverters have their quit dc flag on
    ready --> not_en_hv: if any of the inverters have their ready flags not set
    hv_en --> en: on set of all inverters enabled flags
    en --> en_sp_mode: on command request of speed mode
    en_sp_mode --> speed_mode: on verification of inverters GPIO of speed mode set
    en --> en_torq_mode: on command request of torque mode    
    en_torq_mode --> torq_mode: on verification of inverters GPIO of torque mode set
    torq_mode --> err: on error during operation
    speed_mode --> err: on error during operation
    torq_mode --> en_sp_mode: on request of speed mode AND drivetrain is not active
    speed_mode --> en_torq_mode: on request of torque mode AND drivetrain is not active
    speed_mode --> err: incorrect user command type OR any inverter error
    torq_mode --> err: incorrect user command type OR any inverter error
    err --> clear_err: on user request of error reset 
    clear_err --> not_en_hv: on successful reset of errors
```