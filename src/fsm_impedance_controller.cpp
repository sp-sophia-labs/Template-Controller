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

  Eigen::Matrix<double, 7, 1> FSMImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) 
  {  
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
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
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // for (int i = 1; i <= num_joints; i++) {
    //   config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    //   config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    //   config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    // }
    for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
      state_interfaces_config.names.push_back(franka_robot_model_name);
    }
    for (const auto& franka_robot_state_name : franka_robot_state_->get_state_interface_names()) {
      state_interfaces_config.names.push_back(franka_robot_state_name);
    }
    return state_interfaces_config;
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
    
    // Robot model data
    std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
    std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
    std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    
    //  Robot state data
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state_.measured_joint_state.position.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state_.measured_joint_state.velocity.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state_.measured_joint_state.effort.data());

    Eigen::Vector3d position(
    robot_state_.o_t_ee.pose.position.x,
    robot_state_.o_t_ee.pose.position.y,
    robot_state_.o_t_ee.pose.position.z);
    Eigen::Quaterniond orientation(
    robot_state_.o_t_ee.pose.orientation.w,
    robot_state_.o_t_ee.pose.orientation.x,
    robot_state_.o_t_ee.pose.orientation.y,
    robot_state_.o_t_ee.pose.orientation.z);
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = position;
    transform.rotate(orientation.toRotationMatrix());

    Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
    T = Lambda;
    error.head(3) << position - position_d_;

    if(orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << -transform.rotation() * error.tail(3);

    // F_ext = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data()) * 0.999 + 0.001 * F_ext;
    Eigen::Matrix<double, 6, 1> o_f_ext_hat_k_data;
    o_f_ext_hat_k_data << robot_state_.o_f_ext_hat_k.wrench.force.x, robot_state_.o_f_ext_hat_k.wrench.force.y,
    robot_state_.o_f_ext_hat_k.wrench.force.z, robot_state_.o_f_ext_hat_k.wrench.torque.x,
    robot_state_.o_f_ext_hat_k.wrench.torque.y, robot_state_.o_f_ext_hat_k.wrench.torque.z;    
    
    F_ext = o_f_ext_hat_k_data* 0.999 + 0.001 * F_ext;
    I_F_error += dt*(F_contact_des - F_ext);
    F_cmd = 0.2 * (F_contact_des - F_ext) + 0.1 * I_F_error + F_contact_des - 0 *Sf * F_impedance;
    
    Eigen::VectorXd tau_nullspace(7), tau_d(7), tau_impedance(7);

    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    F_impedance = -Lambda * T.inverse() * (D * (jacobian * dq) + K * error);

    Eigen::Vector3d r = position - C;
    double penetration_depth = std::max(0.0, R-r.norm());
    Eigen::Vector3d v = (jacobian*dq).head(3);
    v = v.dot(r)/r.squaredNorm() * r;
    bool isInSphere = r.norm() < R;
    Eigen::Vector3d projected_error = error.head(3).dot(r)/r.squaredNorm() * r;
    double r_eq = 0.8 * R;
    repulsion_K = (K * r_eq/(R-r_eq)).topLeftCorner(3,3);
    repulsion_D = 2 * (repulsion_K).array().sqrt();
    if(isInSphere){
        F_repulsion.head(3) = 0.99* (repulsion_K * penetration_depth * r/r.norm() - repulsion_D * v) + 0.01 * F_repulsion.head(3); //assume Theta = Lambda
    }
    else{
        double decay = -log(0.0001)/R; //at 2R the damping is divided by 10'000
        F_repulsion.head(3) = - exp(decay * (R-r.norm())) * 0.1 * repulsion_D * v + 0.9 * F_repulsion.head(3); // 0.005 * F_repulsion_new + 0.995 * F_repulsion_old
    }

    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                        jacobian.transpose() * jacobian_transpose_pinv) *
                        (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q) - 
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
    
    double wall_pos = 12.0;
    if (std::abs(position.y()) >= wall_pos){
        F_impedance.y() = -(500 * (position.y()-wall_pos)) + 45 *(jacobian*dq)(1,0) * 0.001 + 0.999 * F_impedance(1,0);
    }

    tau_impedance = jacobian.transpose() * Sm * (F_impedance + F_repulsion) + jacobian.transpose() * Sf * F_cmd;
    tau_d << tau_impedance + tau_nullspace + coriolis; //add nullspace and coriolis components to desired torque
    tau_d << saturateTorqueRate(tau_d, tau_J_d);  // Saturate torque rate to avoid discontinuities
    
    for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].set_value(tau_d(i));
    }

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
