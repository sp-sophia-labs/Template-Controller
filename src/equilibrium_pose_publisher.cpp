#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace fsm_ic
{
    class EquilibriumPosePublisher : public rclcpp::Node
    {
    public:
    EquilibriumPosePublisher()
        : Node("equilibrium_pose_publisher")
    {
        publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("equilibrium_pose", 10);

        // Create a timer to publish messages at a fixed rate (you can modify this as needed)
        timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&EquilibriumPosePublisher::publishEquilibriumPose, this));
    }

    private:
    void publishEquilibriumPose()
    {
        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();

        // Set the values of the PoseStamped message (modify this as needed)
        msg->header.stamp = now();
        msg->header.frame_id = "fr3_link7";  // Set the appropriate frame_id
        msg->pose.position.x = 1.0;
        msg->pose.position.y = 1.0;
        msg->pose.position.z = 1.0;
        msg->pose.orientation.x = 0.0;
        msg->pose.orientation.y = 0.0;
        msg->pose.orientation.z = 0.0;
        msg->pose.orientation.w = 1.0;

        // Publish the message
        publisher_->publish(std::move(msg));
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    };
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto eq_pose_publisher = std::make_shared<fsm_ic::EquilibriumPosePublisher>();
  while (rclcpp::ok()) {
    rclcpp::spin(eq_pose_publisher);
  }
  rclcpp::shutdown();
  return 0;
}

