#include "fsm_impedance_controller/fsm_impedance_controller.hpp"

namespace fsm_ic
{
  controller_interface::InterfaceConfiguration FSMImpedanceController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // get command config
    for (int i = 1; i <= num_joints; i++) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration FSMImpedanceController::state_interface_configuration()const
  {
    // Define state interfaces
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Creates state interface for everything published in robot state (joint, position, velocity)
    for (const auto& franka_robot_state_name : franka_robot_state_->get_state_interface_names()) {
      config.names.push_back(franka_robot_state_name);
    }
    // Create state interface to read robot model for computations involving robot dynamics
    for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(franka_robot_model_name);
    }
    return config;
  }


  CallbackReturn FSMImpedanceController::on_init()
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type FSMImpedanceController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    robot_state_ = franka_msgs::msg::FrankaRobotState();
    franka_robot_state_->get_values_as_message(robot_state_);
    std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state_.measured_joint_state.position.data());

    return controller_interface::return_type::OK;
  }

  CallbackReturn FSMImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
  
    franka_robot_state_ =
      std::make_unique<franka_semantic_components::FrankaRobotState>(
        franka_semantic_components::FrankaRobotState(arm_id_ + "/" + k_robot_state_interface_name));

    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(
        arm_id_ + "/" + k_robot_model_interface_name,
        arm_id_ + "/" + k_robot_state_interface_name));
    
    init_robot_state_ = franka_msgs::msg::FrankaRobotState();
    franka_robot_state_->get_values_as_message(init_robot_state_);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn FSMImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
  {
    franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn FSMImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    franka_robot_state_->release_interfaces();
    franka_robot_model_->release_interfaces();
    return CallbackReturn::SUCCESS;
  }

}

// Expose the controller as visible to the rest of ros2_control
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fsm_ic::FSMImpedanceController,
  controller_interface::ControllerInterface
)
