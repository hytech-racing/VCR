#include <DrivetrainSystem.h>

// DrivetrainSystem::DrivetrainSystem()
// {

// };
DrivetrainSystem::DrivetrainSystem(
    veh_vec<DrivetrainSystem::InverterFuncts> inverter_interfaces)
    : _inverter_interfaces(inverter_interfaces), _state(DrivetrainState_e::NOT_CONNECTED), 
    _check_inverter_ready_flag([](const InverterStatus_s & status) -> bool {return status.inverter_ready;}),
    _check_inverter_connected_flag([](const InverterStatus_s & status) -> bool {return status.connected;}),
    _check_inverter_quit_dc_flag([](const InverterStatus_s & status) -> bool {return status.quit_dc;}),
    _check_inverter_error_flag([](const InverterStatus_s & status) -> bool {return status.error_present;}),
    _check_inverter_hv_present_flag([](const InverterStatus_s & status) -> bool {return status.hv_present;}),
    _check_inverter_hv_not_present_flag([](const InverterStatus_s & status) -> bool {return !status.hv_present;}){};


DrivetrainState_e DrivetrainSystem::get_state()
{
    return _state;
}

DrivetrainStatus_s DrivetrainSystem::evaluate_drivetrain(DrivetrainSystem::CmdVariant cmd) 
{
    _evaluate_state_machine(cmd);

    auto state = get_state();

    DrivetrainStatus_s status;
    status.all_inverters_connected = false;
    // while not all inverters are connected, some may still be here so we can return the entire status for partially connected drivetrain.
    status.inverter_statuses = {_inverter_interfaces.FL.get_status(), 
                                _inverter_interfaces.FR.get_status(), 
                                _inverter_interfaces.RL.get_status(), 
                                _inverter_interfaces.RR.get_status() };

    bool attempting_init_while_not_connected = ((state == DrivetrainState_e::NOT_CONNECTED) && etl::holds_alternative<DrivetrainInit_s>(cmd) && cmd.get<DrivetrainInit_s>().init_drivetrain);
    if (etl::holds_alternative<DrivetrainInit_s>(cmd) && cmd.get<DrivetrainInit_s>().init_drivetrain)
    {
        status.cmd_resp = DrivetrainCmdResponse_e::CANNOT_INIT_NOT_CONNECTED;
    } else {
        status.cmd_resp = DrivetrainCmdResponse_e::COMMAND_INVALID;
    }

    return status; 
}

void DrivetrainSystem::_evaluate_state_machine(DrivetrainSystem::CmdVariant cmd)
{
    switch(get_state())
    {
        // TODO need to ensure that the inverter outputs CAN messages on idle even not when being sent msgs
        case DrivetrainState_e::NOT_CONNECTED:
        {
            bool connected_no_hv_present = (_check_inverter_flags(_check_inverter_connected_flag) && _check_inverter_flags(_check_inverter_hv_not_present_flag) )
            bool connected_hv_present = (_check_inverter_flags(_check_inverter_connected_flag) && _check_inverter_flags(_check_inverter_hv_present_flag)); 
            
            if(connected_no_hv_present)
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT);
            } else if(connected_hv_present)
            {
                _set_state(DrivetrainState_e::NOT_ENABLED_HV_PRESENT);
            }
            
            break;
        }

        case DrivetrainState_e::NOT_ENABLED_NO_HV_PRESENT:
        {

            break;            
        }

        case DrivetrainState_e::NOT_ENABLED_HV_PRESENT:
        {
            break;
        }

        case DrivetrainState_e::ENABLING_INVERTERS_SPEED_MODE:
        {
            break;
        }
        case DrivetrainState_e::ENABLING_INVERTERS_TORQUE_MODE:
        {
            break;
        }
        case DrivetrainState_e::ENABLED_SPEED_MODE:
        {
            break;
        }
        case DrivetrainState_e::ENABLED_TORQUE_MODE:
        {
            break;
        }
        case DrivetrainState_e::ERROR:
        {
            break;
        }
    }
}

// returns false if any of the inverters fail the flag check.
bool DrivetrainSystem::_check_inverter_flags(std::function<bool(const InverterStatus_s&)> flag_check_func)
{
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func : funcs_arr)
    {
        auto inverter_status = func.get_status();
        if(!flag_check_func(inverter_status))
        {
            return false;
        }
    }
    return true;
}

void DrivetrainSystem::keepalive_disabled()
{
    auto funcs_arr = _inverter_interfaces.as_array();
    for(const auto & func: funcs_arr)
    {
        func.set_disable_inverter();
    }
}
