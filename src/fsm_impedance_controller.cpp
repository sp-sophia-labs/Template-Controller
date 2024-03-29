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
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;
    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    return CallbackReturn::SUCCESS;
  }

  void FSMImpedanceController::starting(const rclcpp::Time& /*time*/) 
  {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    // franka::RobotState initial_state = state_handle_->getRobotState();
    auto initial_state = std::make_shared<RobotStateListener>()->robot_state;

    // get jacobian
    std::array<double, 42> jacobian_array =
            model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to eigen
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial = Eigen::Map<Eigen::Matrix<double, 7, 1>>(initial_state->measured_joint_state.position.data());

    // Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    geometry_msgs::msg::Pose initial_transform_Pose = initial_state->o_t_ee.pose;
    // convert Pose to Eigen::Affine3d
    Eigen::Translation3d translation(initial_transform_Pose.position.x, initial_transform_Pose.position.y, initial_transform_Pose.position.z);
    Eigen::Quaterniond rotation(initial_transform_Pose.orientation.w, initial_transform_Pose.orientation.x, initial_transform_Pose.orientation.y, initial_transform_Pose.orientation.z);
    Eigen::Affine3d initial_transform = translation * rotation;
    
    // Not sure which one we should use F_T_EE or f_t_ee.pose ?
    // F_T_EE = initial_state.F_T_EE;
    // EE_T_K = initial_state.EE_T_K;
    geometry_msgs::msg::Pose F_T_EE = initial_state->f_t_ee.pose;
    geometry_msgs::msg::Pose EE_T_K = initial_state->ee_t_k.pose;

    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
    nullspace_stiffness_target_ = 30;
    K.topLeftCorner(3, 3) = 200 * Eigen::Matrix3d::Identity();
    K.bottomRightCorner(3, 3) << 90, 0, 0, 0, 90, 0, 0, 0, 80;
    D.topLeftCorner(3, 3) = 35 * Eigen::Matrix3d::Identity();
    D.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 12;
    cartesian_stiffness_target_ = K;
    cartesian_damping_target_ = D;
    //T.topLeftCorner(3, 3) = 1 * Eigen::Matrix3d::Identity();
    //T.bottomRightCorner(3, 3) = 0.1 * Eigen::Matrix3d::Identity();

    // construct repulsing sphere around 0, 0, 0
    R = 0.00001; C << 0.0, 0, 0.0;
    repulsion_K.setZero(); repulsion_D.setZero();
    // set zero stiffness and damping for rotational velocities and positions
    repulsion_K = Eigen::Matrix3d::Identity(); repulsion_D = Eigen::Matrix3d::Identity();
  }


  controller_interface::return_type FSMImpedanceController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    // get state variables
    // franka::RobotState robot_state = state_handle_->getRobotState();
    auto robot_state = std::make_shared<RobotStateListener>()->robot_state;
    std::array<double, 49> mass = model_handle_->getMassMatrix();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolisForceVector();
    std::array<double, 42> jacobian_array =
              model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state->measured_joint_state.position.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state->measured_joint_state.velocity.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
            robot_state->desired_joint_state.effort.data());
    
    // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    geometry_msgs::msg::Pose transform_pose = robot_state->o_t_ee.pose;
    // convert Pose to Eigen::Affine3d
    Eigen::Translation3d translation(transform_pose.position.x, transform_pose.position.y, transform_pose.position.z);
    Eigen::Quaterniond rotation(transform_pose.orientation.w, transform_pose.orientation.x, transform_pose.orientation.y, transform_pose.orientation.z);
    Eigen::Affine3d transform = translation * rotation;
    
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.rotation());

    Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
    T = Lambda; // let robot behave with it's own physical inertia
    // compute error to desired pose
    // position error
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.rotation() * error.tail(3);

    // compute control
    F_impedance = -Lambda * T.inverse() * (D * (jacobian * dq) + K * error);

    // Force PID
    Eigen::Matrix<double, 6, 1> o_f_ext_hat_k_data;
    o_f_ext_hat_k_data << robot_state->o_f_ext_hat_k.wrench.force.x, robot_state->o_f_ext_hat_k.wrench.force.y,
          robot_state->o_f_ext_hat_k.wrench.force.z, robot_state->o_f_ext_hat_k.wrench.torque.x,
          robot_state->o_f_ext_hat_k.wrench.torque.y, robot_state->o_f_ext_hat_k.wrench.torque.z;
    
    F_ext = o_f_ext_hat_k_data* 0.999 + 0.001 * F_ext;
    I_F_error += dt*(F_contact_des - F_ext);
    F_cmd = 0.2 * (F_contact_des - F_ext) + 0.1 * I_F_error + F_contact_des - 0 *Sf * F_impedance;

    // F_ext = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.O_F_ext_hat_K.data()) * 0.999 + 0.001 * F_ext;
    // I_F_error += dt*(F_contact_des - F_ext);
    // F_cmd = 0.2 * (F_contact_des - F_ext) + 0.1 * I_F_error + F_contact_des - 0 *Sf * F_impedance; //no need to multiply with Sf since it is done afterwards anyway
    // //F_cmd = F_contact_des;

    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);

    // pseudoinverse for nullspace handling
    // kinematic pseudoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    //construct external repulsion force
    Eigen::Vector3d r = position - C; // compute vector between EE and sphere
    //ROS_INFO_STREAM("r is " << r.transpose());
    double penetration_depth = std::max(0.0, R-r.norm());
    //ROS_INFO_STREAM("penetration depth is " << penetration_depth);
    Eigen::Vector3d v = (jacobian*dq).head(3);
    v = v.dot(r)/r.squaredNorm() * r; //projected velocity
    //ROS_INFO_STREAM("projected velocity is " << v.transpose());
    bool isInSphere = r.norm() < R;
    //double alpha = error.norm()/(r.norm()+0.0001); //scaling to reach same forces in repulsion and add offset to not divide by 0
    Eigen::Vector3d projected_error = error.head(3).dot(r)/r.squaredNorm() * r;
    double r_eq = 0.8 * R;
    //double alpha = projected_error.norm()*R*0.95;
    repulsion_K = (K * r_eq/(R-r_eq)).topLeftCorner(3,3); //assume Lambda = Theta(T) to avoid numerical issues
    repulsion_D = 2 * (repulsion_K).array().sqrt();

    if(isInSphere){
        F_repulsion.head(3) = 0.99* (repulsion_K * penetration_depth * r/r.norm() - repulsion_D * v) + 0.01 * F_repulsion.head(3); //assume Theta = Lambda
    }
    else{
        double decay = -log(0.0001)/R; //at 2R the damping is divided by 10'000
        F_repulsion.head(3) = - exp(decay * (R-r.norm())) * 0.1 * repulsion_D * v + 0.9 * F_repulsion.head(3); // 0.005 * F_repulsion_new + 0.995 * F_repulsion_old
    }

    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                      (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q) - //Do not use joint positions yet
                      (2.0 * sqrt(nullspace_stiffness_)) * dq);

    //virtual walls
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
    
    update_stiffness_and_references();
    return controller_interface::return_type::OK;
  }

}

// Expose the controller as visible to the rest of ros2_control
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fsm_ic::FSMImpedanceController,
  controller_interface::ControllerInterface
)
