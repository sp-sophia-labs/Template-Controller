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


class RobotStateListener : public rclcpp::Node {
public:

    franka_msgs::msg::FrankaRobotState::SharedPtr robot_state;

    RobotStateListener() : Node("robot_state_listener") {
        robot_state_subscription_ = create_subscription<franka_msgs::msg::FrankaRobotState>
        ("/franka_robot_state_broadcaster/robot_state",
        10,  
        [this](const franka_msgs::msg::FrankaRobotState::SharedPtr msg) { 
            processRobotState(msg);
            });
    }
    // Public function to get the last received message
    franka_msgs::msg::FrankaRobotState::SharedPtr getRobotState() const {
        return robot_state;
    }


    void processRobotState(const franka_msgs::msg::FrankaRobotState::SharedPtr msg){
        robot_state = msg;

        // position q velocity dq effort tau_J
        std::vector<double> q(7), dq(7), tau_j(7);
        q = robot_state->measured_joint_state.position;
        dq = robot_state->measured_joint_state.velocity;
        tau_j = robot_state->measured_joint_state.effort;

        // desired position q_d velocity dq_d effort tau_J_d
        std::vector<double> q_d(7), dq_d(7), tau_j_d(7);
        q_d = robot_state->desired_joint_state.position;
        dq_d = robot_state->desired_joint_state.velocity;
        tau_j_d = robot_state->desired_joint_state.effort;

        // measured motor state of the joints consisting out of position (theta) and velocity (dtheta)
        std::vector<double> theta(7), dtheta(7);
        theta = robot_state->measured_joint_motor_state.position;
        dtheta = robot_state->measured_joint_motor_state.velocity;  

        // desired joint acceleration
        std::array<double, 7> ddq_d;
        ddq_d = robot_state->ddq_d;

        // derivative of the measured torque signal
        std::array<double, 7> dtau_j;
        dtau_j = robot_state->dtau_j;

        // filtered external torque. The JointState consists out of effort (tau_ext_hat_filtered)
        std::vector<double> tau_ext_hat_filtered(7);
        tau_ext_hat_filtered = robot_state->tau_ext_hat_filtered.effort;

        // active wrenches acting on the stiffness frame expressed relative to 
        // stiffness frame        
        geometry_msgs::msg::Wrench k_f_ext_hat_k;
        k_f_ext_hat_k = robot_state->k_f_ext_hat_k.wrench;
        // base frame
        geometry_msgs::msg::Wrench o_f_ext_hat_k;
        o_f_ext_hat_k = robot_state->o_f_ext_hat_k.wrench;   

        // poses describing the transformations between different frames of the arm
        // Measured end-effector pose in base frame
        geometry_msgs::msg::Pose o_t_ee;
        o_t_ee = robot_state->o_t_ee.pose;
        // Last desired end-effector pose of motion generation in base frame
        geometry_msgs::msg::Pose o_t_ee_d;
        o_t_ee_d = robot_state->o_t_ee_d.pose;
        // Last commanded end-effector pose of motion generation in base frame
        geometry_msgs::msg::Pose o_t_ee_c;
        o_t_ee_c = robot_state->o_t_ee_c.pose;
        // Flange to end-effector frame
        geometry_msgs::msg::Pose f_t_ee;
        f_t_ee = robot_state->f_t_ee.pose;   
        // End-effector to stiffness frame
        geometry_msgs::msg::Pose ee_t_k;
        ee_t_k = robot_state->ee_t_k.pose; 

        // Desired end effector twist in base frame
        geometry_msgs::msg::Twist o_dp_ee_d;
        o_dp_ee_d = robot_state->o_dp_ee_d.twist;
        // Last commanded end effector twist in base frame
        geometry_msgs::msg::Twist o_dp_ee_c;
        o_dp_ee_c = robot_state->o_dp_ee_c.twist;
        // Last commanded end effector acceleration in base frame
        geometry_msgs::msg::Accel o_ddp_ee_c;
        o_ddp_ee_c = robot_state->o_ddp_ee_c.accel;

        // Additional information
        double time, control_command_success_rate;
        time = robot_state->time;
        control_command_success_rate = robot_state->control_command_success_rate;
        uint robot_mode;
        robot_mode = robot_state->robot_mode;

    }
    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_subscription_;
    
};