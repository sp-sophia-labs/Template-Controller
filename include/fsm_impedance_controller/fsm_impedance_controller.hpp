/*
Implement ControllerInterface class as follows:

- If there are any member variables, initialized those in the constructor.
- In the init() method, first call ControllerInterface::init() to initialize the lifecycle of the controller. Following this,
declare all parameters defining their default values.
- Implement the state_interface_configuration() and command_interface_configuration() methods.
- Design the update() function for the controller. (real-time)

- Add the required lifecycle management methods (others are optional):
        on_configure() - reads parameters and configures controller.
        on_activate() - called when controller is activated (started) (real-time)
        on_deactivate() - called when controller is deactivated (stopped) (real-time)
*/

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <fstream>
#include <ostream>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <thread>
#include <cmath>

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
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <franka/robot_state.h>

#include "fsm_impedance_controller/visibility_control.h"

#define IDENTITY Eigen::MatrixXd::Identity(6,6)

namespace fsm_ic
{
class FSMImpedanceController : public controller_interface::ControllerInterface
{
    public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    
    // Config methods
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // ROS2 lifecycle related methods
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    // Main real-time method
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // Custom methods    
    void update_stiffness_and_references();
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose;
    void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);


    private:
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;
    std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> equilibrium_pose_d_;
    bool k_elbow_activated{true};
        
    franka_msgs::msg::FrankaRobotState robot_state_, init_robot_state_;

    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};

    std::string arm_id_{"fr3"};
    int num_joints{7};

    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
            const Eigen::Matrix<double, 7, 1>& tau_J_d); 

    // Classic cartesian controller 
    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{20.0};
    const double delta_tau_max_{1.0};
    
    /*Stiffness Matrix (K): Describes how stiff the end-effector behaves in response to
    external forces. A higher stiffness results in less compliance to external forces.
    Damping Matrix (D): Represents how the system resists changes in velocity.
    It helps dampen oscillations.*/
    /*
    Depending on the parameters of the impedance matrix, you can control the stiffness
    (resistance to deformation) and damping (rate of motion reduction) of the end-effector. 
    */
    /*
    The controller should maintain the equilibrium pose when no external forces are applied.
    This is the natural resting position of the robot's end-effector.
    */
    /*
    The equilibrium pose in Cartesian Impedance Control is the position and orientation
    where the controlled system (robotic manipulator) would naturally come to rest when
    no external forces or torques are applied. This can be thought of as the "relaxed" or
    "neutral" pose of the end-effector.
    */
    /*The equilibrium pose refers to a specific configuration or 
    position of the robot where the applied forces or torques, 
    as well as the robot's response, balance out, resulting in a stable state.
    In the context of Cartesian Impedance Control, 
    achieving an equilibrium pose means that the end-effector 
    is positioned and oriented in a way that the controlled forces 
    and compliance are balanced, resulting in a stable and controlled
    interaction with the environment.
    In simpler terms, the equilibrium pose is a state where the robot, 
    under the influence of the Cartesian Impedance Controller, 
    is in a balanced configuration concerning external forces. 
    This balance ensures that the robot can perform tasks with
    controlled interaction forces, making it suitable for applications
    such as contact tasks, object manipulation, or tasks requiring force-sensitive interactions.*/
    
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    std::mutex position_and_orientation_d_target_mutex_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;   
};

}
