#include "fsm_impedance_controller/fsm_impedance_controller.hpp"

#include <typeinfo>
#include <chrono>
#include <thread>

#include <iostream>
#include <boost/type_index.hpp>

// ANSI escape codes for text color
#define ANSI_COLOR_RESET   "\x1B[0m"
#define ANSI_COLOR_GREEN   "\x1B[32m"

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

    // If the singular values are very small, increasing the damping factor might help stabilize the computation.

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // it's possible that the Jacobian matrix is close to singular If you notice 
    // very small singular values, it might indicate a near-singular matrix, 
    // and the damping term in your pseudo-inverse function might need adjustment.
    std::cout << "Singular values:\n" << svd.singularValues() << std::endl;
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
      S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
  }

  Eigen::Matrix<double, 7, 1> FSMImpedanceController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d) 
  {  
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void FSMImpedanceController::update_stiffness_and_references()
  {
      // update parameters changed online either through dynamic reconfigure or through the interactive
      // target by filtering
      /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
      K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
      D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
      nullspace_stiffness_ =
              filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
      std::lock_guard<std::mutex> position_d_target_mutex_lock(
              position_and_orientation_d_target_mutex_);
      position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
      orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }


  CallbackReturn FSMImpedanceController::on_init()
  {
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;
    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration FSMImpedanceController::command_interface_configuration() const
  {
    // Define command interfaces
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    // get command config
    for (int i = 1; i <= num_joints; i++) {
      command_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration FSMImpedanceController::state_interface_configuration()const
  {
    // Define state interfaces
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    // Creates state interface for the desired equilibrium position
    state_interfaces_config.names = equilibrium_pose_d_->get_state_interface_names();

    // Creates state interface for robot state 
    for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
      state_interfaces_config.names.push_back(franka_robot_model_name);
    }
    // Creates state interface for robot model
    for (const auto& franka_robot_state_name : franka_robot_state_->get_state_interface_names()) {
      state_interfaces_config.names.push_back(franka_robot_state_name);
    }
    return state_interfaces_config;
  }

  CallbackReturn FSMImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    equilibrium_pose_d_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated));
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(franka_semantic_components::FrankaRobotState(arm_id_ + "/" + k_robot_state_interface_name));
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name, arm_id_ + "/" + k_robot_state_interface_name));
    return CallbackReturn::SUCCESS;
  }

  // In ROS 2, the equivalent functionality to the starting() method in ROS 1 is typically handled in the on_activate() method. 
  // The on_activate() method is called when a controller is activated, and this is a suitable place to perform actions 
  // that need to occur just before the controller starts updating its state.
  CallbackReturn FSMImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
  {
    franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
    equilibrium_pose_d_->assign_loaned_state_interfaces(state_interfaces_);

    // Starting()
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    init_robot_state_ = franka_msgs::msg::FrankaRobotState();
    franka_robot_state_->get_values_as_message(init_robot_state_);

    // std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector); Not used

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(init_robot_state_.measured_joint_state.position.data());

    Eigen::Vector3d position_init(
    init_robot_state_.o_t_ee.pose.position.x,
    init_robot_state_.o_t_ee.pose.position.y,
    init_robot_state_.o_t_ee.pose.position.z);
    Eigen::Quaterniond orientation_init(
    init_robot_state_.o_t_ee.pose.orientation.w,
    init_robot_state_.o_t_ee.pose.orientation.x,
    init_robot_state_.o_t_ee.pose.orientation.y,
    init_robot_state_.o_t_ee.pose.orientation.z);
    Eigen::Affine3d initial_transform = Eigen::Affine3d::Identity();
    initial_transform.translation() = position_init;
    initial_transform.rotate(orientation_init.toRotationMatrix());

    position_d_ = initial_transform.translation();
    orientation_d_ = initial_transform.rotation();
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = initial_transform.rotation();

    F_T_EE = init_robot_state_.f_t_ee;
    EE_T_K = init_robot_state_.ee_t_k;

    q_d_nullspace_ = q_initial;
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

  controller_interface::CallbackReturn FSMImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    franka_robot_state_->release_interfaces();
    franka_robot_model_->release_interfaces();
    equilibrium_pose_d_->release_interfaces();
    RCLCPP_INFO(get_node()->get_logger(), ANSI_COLOR_GREEN"deactivated successfully");
    return CallbackReturn::SUCCESS;
  }


  controller_interface::return_type FSMImpedanceController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                  "---------------------------- update() -----------------------------");

    robot_state_ = franka_msgs::msg::FrankaRobotState();
    franka_robot_state_->get_values_as_message(robot_state_);
    
    // 1.Robot model data
    std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
    std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    
    // 2.Robot state data
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state_.measured_joint_state.position.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state_.measured_joint_state.velocity.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_j_d(robot_state_.measured_joint_state.effort.data());
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

    // 3.Compute error to desired pose
    Eigen::Matrix<double, 6, 1> error;
    // position error
    error.head(3) << position - position_d_;
    // orientation error
    if(orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.rotation() * error.tail(3); 


    // 4.Compute control law
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // FIX PSEUDOINVERSE
    std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

    std::array<double, 42> endeffector_jacobian_wrt_base = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,"end_effector_jacobian in base frame :" << endeffector_jacobian_wrt_base);

    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    std::cout << "Jacobian:\n" << jacobian << std::endl;

    Eigen::MatrixXd jacobian_transpose_pinv;

    try {
      pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    } catch (const std::exception& e) {
      std::cerr << "Error during pseudo-inverse computation: " << e.what() << std::endl;
    }
    
    // pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    
    // pseudoinverse for nullspace handling
    
    // pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv); //run error
    
    // Cartesian PD control with damping ratio = 1
    // tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    //             tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
    //             jacobian.transpose()* jacobian_transpose_pinv) *
    //             (nullspace_stiffness_ * (q_d_nullspace_ - q) -
    //             (2.0 * sqrt(nullspace_stiffness_)) * dq);

    // tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    //             tau_nullspace << (Eigen::MatrixXd::Identity(7, 7)) *
    //             (nullspace_stiffness_ * (q_d_nullspace_ - q) -
    //             (2.0 * sqrt(nullspace_stiffness_)) * dq);

    // tau_d << tau_task + tau_nullspace + coriolis;
    // saturate the commanded torque to joint limits
    // tau_d << saturateTorqueRate(tau_d, tau_j_d);

    // for (int i = 0; i < num_joints; i++) {
    //   // command_interfaces_ is already defined as std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_
    //   // in controller_interface_base
    //   command_interfaces_[i].set_value(tau_d[i]); 
    //   std::cout<<tau_d[i]<<std::endl;
    // }
    
    return controller_interface::return_type::OK;
  }


}

// Expose the controller as visible to the rest of ros2_control
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fsm_ic::FSMImpedanceController,
  controller_interface::ControllerInterface
)
