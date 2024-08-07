#ifndef _MOTEUS_WRAPPER_HPP_
#define _MOTEUS_WRAPPER_HPP_

#include "ControllerWrapper.hpp"
#include "3rd_libs/moteus/moteus.h"
#include <memory>

namespace controller_interface
{

class MoteusWrapper final: public ControllerWrapper 
{
    private:

    /* Const coefficients for easy radians - rotations transform */
    constexpr static double rotation_to_radians_ = 2 * M_PI;
    constexpr static double radians_to_rotation_ = 1 / (2 * M_PI); /* Multiplying is faster than dividing */

    /* Command structure for moteus object*/
    mjbots::moteus::PositionMode::Command position_command_;
    mjbots::moteus::Controller moteus_controller_;
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    MoteusWrapper( 
        const mjbots::moteus::Controller::Options& options,
        const mjbots::moteus::PositionMode::Command& command);
    void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) override;
    void query_to_tx_frame(CanFrame& tx_frame) override;
    void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) override;
    void init_to_tx_frame(CanFrame& tx_frame) override;

};

std::unique_ptr<MoteusWrapper> make_moteus_wrapper(const ControllerParameters& params);

};

#endif