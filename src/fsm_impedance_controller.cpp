#include "fsm_impedance_controller/fsm_impedance_controller.hpp"

namespace fsm_ic
{
  namespace {
    template <class T, size_t N>
    std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
        //ostream << "[";
        std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
        std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
        //ostream << "]";
        return ostream;
    }
  }  // anonymous namespace

  inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
  }

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
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
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

    // std::array<double, 42> jacobian_array =
    //     franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(init_robot_state_.measured_joint_state.position.data());

    Eigen::Vector3d position(
    init_robot_state_.o_t_ee.pose.position.x,
    init_robot_state_.o_t_ee.pose.position.y,
    init_robot_state_.o_t_ee.pose.position.z);
    Eigen::Quaterniond orientation(
    init_robot_state_.o_t_ee.pose.orientation.w,
    init_robot_state_.o_t_ee.pose.orientation.x,
    init_robot_state_.o_t_ee.pose.orientation.y,
    init_robot_state_.o_t_ee.pose.orientation.z);
    Eigen::Affine3d initial_transform = Eigen::Affine3d::Identity();
    initial_transform.translation() = position;
    initial_transform.rotate(orientation.toRotationMatrix());

    position_d_ = initial_transform.translation();
    orientation_d_ = initial_transform.rotation();
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = initial_transform.rotation();

    auto F_T_EE = init_robot_state_.f_t_ee;
    auto EE_T_K = init_robot_state_.ee_t_k;

    auto q_d_nullspace_ = q_initial;
    nullspace_stiffness_target_ = 30;

    K.topLeftCorner(3, 3) = 200 * Eigen::Matrix3d::Identity();
    K.bottomRightCorner(3, 3) << 90, 0, 0, 0, 90, 0, 0, 0, 80;
    D.topLeftCorner(3, 3) = 35 * Eigen::Matrix3d::Identity();
    D.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 12;
    cartesian_stiffness_target_ = K;
    cartesian_damping_target_ = D;

    R = 0.00001; C << 0.0, 0, 0.0;
    repulsion_K.setZero(); repulsion_D.setZero();
    repulsion_K = Eigen::Matrix3d::Identity(); repulsion_D = Eigen::Matrix3d::Identity();

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
