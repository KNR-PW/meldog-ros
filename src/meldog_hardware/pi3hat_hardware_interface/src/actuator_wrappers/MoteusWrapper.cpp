#include "../../include/actuator_wrappers/MoteusWrapper.hpp"

/*
    Moteus Actuator Wrapper. It uses API from moteus repository to communicate with pi3hat interface.
    https://github.com/mjbots/moteus

*/

using namespace actuator_wrappers;

MoteusWrapper::MoteusWrapper(ActuatorParameters& params, 
    mjbots::moteus::Controller::Options& options,
    mjbots::moteus::PositionMode::Command& command): 
ActuatorWrapperBase<MoteusWrapper>(params), mjbots::moteus::Controller(options), 
position_command_(command){ }



void MoteusWrapper::init(CanFrame& tx_frame)
{
    /* create CANFD frame*/
    mjbots::moteus::CanFdFrame can_fd_frame = mjbots::moteus::Controller::MakeStop();

    /* Copy data from CANFD frame to CAN frame*/
    tx_frame.id = can_fd_frame.arbitration_id;
    tx_frame.bus = can_fd_frame.bus;
    tx_frame.size = can_fd_frame.size;
    std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);
}

void MoteusWrapper::command_to_tx_frame(CanFrame& tx_frame, const ActuatorCommand& command)
{
    /* Change command values */
    position_command_.position = command.position_ * radians_to_rotation;
    position_command_.velocity = command.velocity_ * radians_to_rotation;
    position_command_.feedforward_torque = command.torque_;

    /* Create CANFD frame*/
    mjbots::moteus::CanFdFrame can_fd_frame = mjbots::moteus::Controller::MakePosition(position_command_);
    
    /* Copy data from CANFD frame to CAN frame*/
    tx_frame.id = can_fd_frame.arbitration_id;
    tx_frame.bus = can_fd_frame.bus;
    tx_frame.size = can_fd_frame.size;
    std::memcpy(tx_frame.data, can_fd_frame.data, can_fd_frame.size);
}

void MoteusWrapper::rx_frame_to_state(const CanFrame& rx_frame, ActuatorState& state)
{
    /* Parse data from RX CAN frame to Result object */
    if(((rx_frame.id >> 8) & 0x7f) != params_.id) return; /* This should not happen! (map frame to wrapper first) */
    
    mjbots::moteus::Query::Result result = mjbots::moteus::Query::Parse(rx_frame.data, rx_frame.size);
    state.position_ = result.position * rotation_to_radians;
    state.velocity_ = result.velocity * rotation_to_radians;
    state.torque_ = result.torque;
    state.temperature_ = result.temperature;
    state.fault = result.fault;
}
