#ifndef PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_
#define PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_


#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "../actuator_wrappers/MoteusWrapper.hpp"
#include "../imu/IMUTransform.hpp"
#include "../pi3hat/pi3hat.h"
#include "../pi3hat/realtime.h"

#include "visibility_control.hpp"

#include <cmath>

namespace pi3hat_hardware_interface
{
    class Pi3HatHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Pi3HatHardwareInterface)

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        
        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        ROS2_CONTROL_PI3HAT_HARDWARE_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        /* PART FOR COMMUNICATION WITH HARDWARE: */

        /* number of actuators */
        size_t number_of_actuators;

        /* Pi3hat */
        std::shared_ptr<mjbots::pi3hat::Pi3Hat> pi3hat_;

        /* Pi3hat input structure */
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;

        /* IMU states */ 
        mjbots::pi3hat::Attitude attitude_;

        /* IMU transform */
        IMU::IMUTransform imu_transform_;

        /* TX CAN frames */
        std::shared_ptr<mjbots::pi3hat::CanFrame[]> tx_can_frames_;

        /* RX CAN frames */ 
        std::shared_ptr<mjbots::pi3hat::CanFrame[]> rx_can_frames_;

        /* Container for motor_id -> joint_index maping */
        std::map<int, int> actuator_joint_map_;

        /* Actuator states and commands */
        std::vector<actuator_wrappers::ActuatorState> hw_actuator_states_;
        std::vector<actuator_wrappers::ActuatorCommand> hw_actuator_commands_;

        /* Actuator Wrappers (HERE change to your own wrapper) */
        std::vector<actuator_wrappers::MoteusWrapper> moteus_wrappers;


        using JointState = actuator_wrappers::ActuatorState;
        using JointCommand = actuator_wrappers::ActuatorCommand;

        /* Joint states and commands (for transmissions)*/
        std::vector<JointState> hw_joint_states_;
        std::vector<JointCommand> hw_joint_commands_;

        /* Function for creating moteus wrappers (here u can add your own wrapper)
           Remember to change this function in source code */
        void add_actuator_wrapper(const hardware_interface::HardwareInfo &info, const WrapperType type);

        template<class Wrapper>
        void commands_to_tx_frames(std::vector<actuator_wrappers::ActuatorWrapperBase<Wrapper>> actuator_wrappers)
        {
            // TODO: Uporządkuj wcześniej silniki względem id
            for(int i = 0; i < number_of_actuators; i++)
            {   
                actuator_wrappers[i].command_to_tx_frame(tx_can_frames_[i], hw_actuator_commands_[i]);
            }
        };

        template<class Wrapper>
        void rx_frames_to_states(std::vector<actuator_wrappers::ActuatorWrapperBase<Wrapper>> actuator_wrappers)
        {
            for(int i = 0; i < number_of_actuators; i++)
            {   
                int joint_index = actuator_joint_map_[rx_can_frames_[i].id];
                actuator_wrappers[joint_index].rx_frame_to_state(rx_can_frames_[i], hw_actuator_states_[joint_index]);
            }
        }

        /* PART FOR CREATING TRANSMISSION OBJECTS:*/
        
    };


    /* Here add your actuator wrapper type */
    enum WrapperType
    {
        Moteus = 0,

    };

}; 

#endif 
