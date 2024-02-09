#include "fsm_impedance_controller/fsm_impedance_controller.hpp"
#include <fsm_impedance_controller/robot_state.hpp>

#include <controller_interface/controller_interface.hpp>


namespace {
  template <class T, size_t N>
  std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
  }
}  // anonymous namespace


namespace fsm_ic
{

  controller_interface::InterfaceConfiguration FSMImpedanceController::command_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration FSMImpedanceController::state_interface_configuration()const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
      state_interfaces_config.names.push_back(franka_robot_model_name);
    }
    return state_interfaces_config;
  }


  CallbackReturn FSMImpedanceController::on_init()
  {
    // try {
    //   if (!get_node()->get_parameter("arm_id", arm_id_)) {
    //     RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_id parameter");
    //     get_node()->shutdown();
    //     return CallbackReturn::ERROR;
    //   }
    // } catch (const std::exception& e) {
    //   fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    //   return CallbackReturn::ERROR;
    // }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type FSMImpedanceController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
    std::array<double, 7> coriolis = franka_robot_model_->getCoriolisForceVector();
    std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();
    std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kJoint4);
    std::array<double, 42> joint4_body_jacobian_wrt_joint4 =
      franka_robot_model_->getBodyJacobian(franka::Frame::kJoint4);
    std::array<double, 42> endeffector_jacobian_wrt_base =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    return controller_interface::return_type::OK;
  }

  CallbackReturn FSMImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
    franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));

    RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn FSMImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
  {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn FSMImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
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
