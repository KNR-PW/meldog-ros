#ifndef _MOTEUS_WRAPPER_HPP_
#define _MOTEUS_WRAPPER_HPP_

#include "ControllerWrapper.hpp"
#include "../../3rd_libs/moteus/moteus.h"
#include <memory>

namespace controller_interface
{

class MoteusWrapper final: public ControllerWrapper 
{
    private:

    /* Const coefficients for easy radians - rotations transform */
    constexpr static double rotation_to_radians_ = 2 * M_PI;
    constexpr static double radians_to_rotation_ = 1 / (2 * M_PI); /* Multiplying is faster than dividing */
    constexpr static double startup_coefficient_ = 0.05; /* For slow start-up */

    /* Command structure for moteus object*/
    mjbots::moteus::PositionMode::Command position_command_;
    mjbots::moteus::Controller moteus_controller_;
    
    public:
    using CanFrame = mjbots::pi3hat::CanFrame;

    MoteusWrapper(const ControllerParameters params,
    mjbots::moteus::Controller::Options moteus_options);
    MoteusWrapper(const MoteusWrapper& other);
    MoteusWrapper& operator=(const MoteusWrapper& other) = delete;
    MoteusWrapper(MoteusWrapper&& other);
    MoteusWrapper& operator=(MoteusWrapper&& other) = delete;


    void command_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) override;
    void rx_frame_to_state(const CanFrame& rx_frame, ControllerState& state) override;
    void init_to_tx_frame(CanFrame& tx_frame) override;
    void start_pos_to_tx_frame(CanFrame& tx_frame, const ControllerCommand& command) override;

    ~MoteusWrapper() = default;
};

/* Function for creating unique pointer for moteus wrapper (used in pi3hat_hardware_interface) */
std::unique_ptr<MoteusWrapper> make_moteus_wrapper(const ControllerParameters params);

};

#endif