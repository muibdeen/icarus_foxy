#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "mechanum_wheel_controller/mechanum_wheel_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace mechanum_wheel_controller
{
    //hardware iunfo stores information about hardware defined in the URDF
    hardware_interface::return_type MechanumController::configure(const hardware_interface::HardwareInfo & info_)
    {

        //set state and command contrainers to inital values
        for (uint i = 0; i < hw_states_position_.size(); i++)
        {
            hw_commands_[i] = 0;
            hw_states_position_[i] = 0;
            hw_states_velocity_[i] = 0;
        }

        /*if (hardware_interface::return_type::configure(info_) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }*/

        //set hardware states and commands to be equial to the size of robot joints
        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        //iterate through joints to make sure parameters were properly defined
        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            //each joint should only accept one type of command and 2 state interfaces
            if (joint.command_interfaces.size() !=1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("HardwareInterface"),
                "Joint '%s' has '%zu' command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SimlatorInterface"),
                "Joint '%s' has '%s' command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces.size() !=2)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SimulatorInterface"),
                "Joint '%s' has '%zu' state interfaces. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SimulatorInterface"),
                "Joint '%s' has '%s' state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::return_type::ERROR;
            }
            
            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SimulatorInterface"),
                "Joint '%s' has '%s' state interface. '%s'  expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

    //configure callback occurs when hardware node transitions to a configuring state
    /*hardware_interface::return_type MechanumController::configure(const rclcpp_lifecycle::State & previous_state)
    {
        //set state and command contrainers to inital values
        for (uint i = 0; i < hw_states_position_.size(); i++)
        {
            hw_commands_[i] = 0;
            hw_states_position_[i] = 0;
            hw_states_velocity_[i] = 0;
        }
        RCLCPP_INFO(rclcpp::get_logget("HardwareInterface"), "Successfully configured!");
        return hardware_interface::return_type::OK;
    } */

    hardware_interface::return_type MechanumController::start()
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MechanumController::stop()
    {
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> MechanumController::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i<info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    std::vector<hardware_interface::StateInterface> MechanumController::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i<info_.joints.size(); i++)
        {   
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    hardware_interface::return_type MechanumController::write()
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MechanumController::read()
    {
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mechanum_wheel_controller::MechanumController, hardware_interface::SystemInterface)