#include <memory>
#include <string>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "isaac_ros_visual_slam_interfaces/srv/set_odometry_pose.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("isaac_awake")
  {
    subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/sdv_localization/enu_pose", 10, std::bind(&Listener::listener_callback, this, std::placeholders::_1));
    //client_ = create_client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>("/visual_slam/set_odometry_pose");
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  }

private:
  void listener_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // If the message received is valid and it's the first valid message, run the launch file
    if (!msg->pose.pose.position.x == 0 && !first_msg_received) {
      first_msg_received = true;
      first_msg = *msg;
      broadcast_transform();
      run_launch_file();
    }
  }
  void run_launch_file()
  {
    std::string launch_file_path = "/workspaces/isaac_ros-dev/src/sdv_localization/sdv_localization/launch/isaac_ros_visual_slam.launch.py";
    int exit_code = system(("ros2 launch " + launch_file_path).c_str());

   // auto request = std::make_shared<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose::Request>();
    //request->pose = first_msg.pose.pose; // use the pose from the first message received
    //auto result_future = client_->async_send_request(request);
    //if (result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    //{
     // auto result = result_future.get();
    //  // TODO: handle the result of the service call
    //}

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
  void broadcast_transform()
  {
      geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->now();
    static_transform_stamped.header.frame_id = "odom";
    static_transform_stamped.child_frame_id = "odom_isaac";
    static_transform_stamped.transform.translation.x = first_msg.pose.pose.position.x;
    static_transform_stamped.transform.translation.y = first_msg.pose.pose.position.y;
    static_transform_stamped.transform.translation.z = first_msg.pose.pose.position.z;
    //static_transform_stamped.transform.rotation = first_msg.pose.pose.orientation; 
        tf2::Quaternion original_orientation(
        first_msg.pose.pose.orientation.x,
        first_msg.pose.pose.orientation.y,
        first_msg.pose.pose.orientation.z,
        first_msg.pose.pose.orientation.w);

    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, M_PI/2); // 90 degrees rotation around z-axis

    tf2::Quaternion new_orientation = original_orientation * rotation;
    new_orientation.normalize(); // It's a good practice to normalize the quaternion after the multiplication

    static_transform_stamped.transform.rotation.x = new_orientation.x();
    static_transform_stamped.transform.rotation.y = new_orientation.y();
    static_transform_stamped.transform.rotation.z = new_orientation.z();
    static_transform_stamped.transform.rotation.w = new_orientation.w();


    // Interchange x and y orientations
    //static_transform_stamped.transform.rotation.x = first_msg.pose.pose.orientation.y;
    //static_transform_stamped.transform.rotation.y = first_msg.pose.pose.orientation.x;
    // Keep z and w the same
    //static_transform_stamped.transform.rotation.z = first_msg.pose.pose.orientation.z;
    //static_transform_stamped.transform.rotation.w = first_msg.pose.pose.orientation.w;
    broadcaster_->sendTransform(static_transform_stamped);

 }
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>::SharedPtr client_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  geometry_msgs::msg::PoseWithCovarianceStamped first_msg;
  bool first_msg_received = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto listener = std::make_shared<Listener>();
  rclcpp::spin(listener);
  rclcpp::shutdown();

  return 0;
}

