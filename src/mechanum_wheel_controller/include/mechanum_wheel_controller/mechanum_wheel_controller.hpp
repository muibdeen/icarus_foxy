#ifndef MECHANUM_WHEEL_CONTROLLER_
#define MECHANUM_WHEEL_CONTROLLER_

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
//#include "control_toolbox/pid.hpp"
#include <pigpiod_if2.h>

namespace mechanum_wheel_controller
{
class MechanumController : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
    public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MechanumController)

    MECHANUM_WHEEL_CONTROLLER_
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

    MECHANUM_WHEEL_CONTROLLER_
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MECHANUM_WHEEL_CONTROLLER_
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MECHANUM_WHEEL_CONTROLLER_
    hardware_interface::return_type start() override;

    MECHANUM_WHEEL_CONTROLLER_
    hardware_interface::return_type stop() override;

    MECHANUM_WHEEL_CONTROLLER_
    hardware_interface::return_type read() override;

    MECHANUM_WHEEL_CONTROLLER_
    hardware_interface::return_type write() override; 

    private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
    //pigpiio pins
};
}
#endif