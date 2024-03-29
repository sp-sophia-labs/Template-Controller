#ifndef FSM_IMPEDANCE_CONTROLLER_HPP
#define FSM_IMPEDANCE_CONTROLLER_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <fstream>
#include <ostream>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include <rclcpp/time.hpp> 
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include <controller_interface/controller_interface.hpp> 

#include "hardware_interface/types/hardware_interface_type_values.hpp" 
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/macros.hpp"
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>

#include "franka_hardware/franka_hardware_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

#include "fsm_impedance_controller/visibility_control.h"

#define IDENTITY Eigen::MatrixXd::Identity(6,6) 

namespace fsm_ic
{
class FSMImpedanceController : public controller_interface::ControllerInterface
{
    public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    CallbackReturn on_init() override;
    void starting(const rclcpp::Time&);
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    void update_stiffness_and_references();
    void log_values_to_file(bool do_logging);

    private:

    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
            const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)


    std::unique_ptr<franka_semantic_components::FrankaRobotState> state_handle_; // std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> model_handle_; // std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::CommandInterface> joint_handles_;// std::vector<hardware_interface::JointHandle> joint_handles_;

    Eigen::Matrix<double, 6,6> Lambda = IDENTITY; // operational space mass matrix
    Eigen::Matrix<double, 6, 6> Sm = IDENTITY; //task space selection matrix for positions and rotation
    Eigen::Matrix<double, 6, 6> Sf = Eigen::MatrixXd::Zero(6,6); //task space selection matrix for forces
    std::array<double, 16> F_T_EE; //end effector in flange frame
    std::array<double, 16> EE_T_K; //stiffness frame in EE frame
    Eigen::Affine3d pose_desired;
    Eigen::Matrix<double, 6, 1> error; //pose error (6d)
    Eigen::Matrix<double, 6, 1>  F_contact_des = Eigen::MatrixXd::Zero(6,6); //desired contact force
    Eigen::Matrix<double, 6, 1>  F_ext = Eigen::MatrixXd::Zero(6,6); //external forces
    Eigen::Matrix<double, 6, 1>  F_cmd = Eigen::MatrixXd::Zero(6,6); //commanded contact force
    Eigen::Matrix<double, 6, 1>  I_F_error = Eigen::MatrixXd::Zero(6,6); //force error integral
    Eigen::Matrix<double, 6,6> T = IDENTITY; // impedance inertia term
    Eigen::Matrix<double, 6,6> K = IDENTITY; //impedance stiffness term
    Eigen::Matrix<double, 6,6> D = IDENTITY; //impedance damping term
    Eigen::Matrix<double, 6,6> cartesian_stiffness_target_; //impedance damping term
    Eigen::Matrix<double, 6,6> cartesian_damping_target_; //impedance damping term
    Eigen::Matrix<double, 6,6> cartesian_inertia_target_; //impedance damping term

    //FLAGS
    bool config_control = true; //sets if we want to control the configuration of the robot in nullspace
    bool do_logging = false; //set if we do log values
    // end FLAGS
    double filter_params_{0.005};
    double nullspace_stiffness_{40};
    double nullspace_stiffness_target_{40.0};
    const double delta_tau_max_{1.0};
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;

    std::mutex position_and_orientation_d_target_mutex_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;
    
    double count = 0; //logging

    // Is trigger needed ?
    // franka_hw::TriggerRate log_rate_{1000}; //logging
    double dt = 0.001;

    //repulsion sphere test;
    double R;
    Eigen::Vector3d C;
    Eigen::Matrix<double, 3, 3> repulsion_K, repulsion_D;
    Eigen::Matrix<double, 6, 1> F_repulsion;
    Eigen::Matrix<double, 6, 1> F_impedance;
};

}

#endif // FSM_IMPEDANCE_CONTROLLER_HPP
