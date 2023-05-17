#include <memory>
#include <string>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("isaac_awake")
  {
    // Create a subscriber to the topic '/sdv_localization/enu_pose'
    subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/sdv_localization/enu_pose", 10, std::bind(&Listener::listener_callback, this, std::placeholders::_1));
  }

private:
  void listener_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
  {

    // If the message received is valid, run the launch file
    if (!msg->pose.pose.position.x == 0) {
      run_launch_file();
    }
  }

  void run_launch_file() const
  {
    // Run the launch file using system
    // Change the path to your launch file
    std::string launch_file_path = "/workspaces/isaac_ros-dev/src/isaac_ros_visual_slam/isaac_ros_visual_slam/launch/isaac_ros_visual_slam.launch.py";
    int exit_code = system(("ros2 launch " + launch_file_path).c_str());
    if (exit_code == -1) {
      RCLCPP_ERROR(get_logger(), "Error running the launch file");
    } else if (WIFEXITED(exit_code)) {
      RCLCPP_INFO(get_logger(), "Launch process exited with code: %d", WEXITSTATUS(exit_code));
    } else if (WIFSIGNALED(exit_code)) {
      RCLCPP_ERROR(get_logger(), "Launch process terminated by signal: %d", WTERMSIG(exit_code));
    } else {
      RCLCPP_ERROR(get_logger(), "Launch process exited with unknown state");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create an instance of the Listener class
  auto listener = std::make_shared<Listener>();

  // Start the ROS2 node
  rclcpp::spin(listener);

  // Shutdown the node and stop ROS2
  rclcpp::shutdown();

  return 0;
}

