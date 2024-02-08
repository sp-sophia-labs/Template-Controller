#include "fsm_impedance_controller/fsm_impedance_controller.hpp"
#include <fsm_impedance_controller/robot_state.hpp>

#include <cmath>
#include <memory>

#include <controller_interface/controller_interface.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include <franka/robot_state.h>

#define IDENTITY Eigen::MatrixXd::Identity(6,6)

namespace fsm_ic
{

  controller_interface::InterfaceConfiguration FSMImpedanceController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    return command_interfaces_config;
  }
  controller_interface::InterfaceConfiguration FSMImpedanceController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    return state_interfaces_config;
  }   

  CallbackReturn FSMImpedanceController::on_init()
  {
    return CallbackReturn::SUCCESS;
  }

  void FSMImpedanceController::starting(const rclcpp::Time& /*time*/) 
  {

  }


  controller_interface::return_type FSMImpedanceController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    return controller_interface::return_type::OK;
  }

}

// Expose the controller as visible to the rest of ros2_control
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fsm_ic::FSMImpedanceController,
  controller_interface::ControllerInterface
)
