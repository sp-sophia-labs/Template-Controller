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

namespace fsm_ic
{
class FSMImpedanceController : public controller_interface::ControllerInterface
{
    public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // The Needs thing to be implemented for ros2_control interface
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // The following is all ROS2's lifecycle related stuff
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;


    private:
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;
    std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
    
    franka_msgs::msg::FrankaRobotState robot_state_, init_robot_state_;

    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};

    std::string robot_description_;
    std::string robot_name_;

    // std::string arm_id_;
    std::string arm_id_{"panda"};
    int num_joints{7};
};

}
