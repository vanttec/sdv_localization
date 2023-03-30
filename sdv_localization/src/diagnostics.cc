#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sdv_localization_msgs/msg/sensor_diag.hpp"
#include "sdv_localization_msgs/msg/sensor_stat_array.hpp"


using namespace std::chrono_literals;

//ROS2 node class for odometry, NED pose, and velodyne transform - 
class sdv_diagnostics : public rclcpp::Node
  {
    

public:
  sdv_diagnostics() : Node("sdv_diagnosticsNode")
  {
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    options.topic_stats_options.publish_period = std::chrono::seconds(10);
    options.topic_stats_options.publish_topic = "/vn_statistics";

    timer_ = this->create_wall_timer(250ms,  std::bind(&sdv_diagnostics::timer_callback, this));


    //Publishers
    pub_diagnostics_ = this->create_publisher<sdv_localization_msgs::msg::SensorStatArray>("sdv_localization/diagnostics", 10);

    // Subscribers
    
    auto sub_vn_common_cb = std::bind(&sdv_diagnostics::sub_vn_common, this, std::placeholders::_1);
    sub_vn_common_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", 10, sub_vn_common_cb, options);

    auto sub_velodyne_cb = std::bind(&sdv_diagnostics::sub_velodyne, this, std::placeholders::_1);
    sub_velodyne_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, sub_velodyne_cb, options);

    auto sub_multisense_cb = std::bind(&sdv_diagnostics::sub_multisense, this, std::placeholders::_1);
    sub_multisense_ = this->create_subscription<sensor_msgs::msg::Image>(
      "multisense/left/image_mono", 10, sub_multisense_cb, options);





  }



private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */
  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
  {
    if (!hasPublishervn)
    {
      hasPublishervn = true;
    }

    if (msg_in->position.x != 0) {
      isGpsAvailable = true;
    }
  }

  void sub_velodyne(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
  {
    if (!hasPublishervl)
    {
      hasPublishervl = true;
    }
  }

  void sub_multisense(const sensor_msgs::msg::Image::SharedPtr msg_in)
  {
    if (!hasPublisherms)
    {
      hasPublisherms = true;
    }
  }

  void timer_callback()
  {
    sdv_localization_msgs::msg::SensorStatArray msg_output;
    int novnpub = sub_vn_common_->get_publisher_count();
    int novlpub  = sub_velodyne_pcl_->get_publisher_count();
    int nomspub = sub_multisense_ ->get_publisher_count();

    msg_output.header.stamp = this->get_clock()->now();
    
    if (novnpub == 0){
      hasInitializedvn = false;
      hasPublishervn = false;

    }

    else if (novnpub >= 1 && !hasInitializedvn){
      hasInitializedvn = true;
    }

    if (novlpub == 0){
      hasInitializedvl = false;
      hasPublishervl = false;
    }
    else if (novlpub >= 1 && !hasInitializedvl) {
      hasInitializedvl = true;
    }

    if (nomspub == 0){
      hasInitializedms = false;
      hasPublisherms = false;
    }

    if (nomspub >= 1 && !hasInitializedms) {
      hasInitializedms = true;
    }
    
    //If topic has a single publisher

      msg_output.vn_status.is_enabled = hasInitializedvn;
      msg_output.vl_status.is_enabled = hasInitializedvl;
      msg_output.ms_status.is_enabled = hasInitializedms;
    
    //If topic has been published

      msg_output.vn_status.is_publishing = hasPublishervn;
      msg_output.vl_status.is_publishing = hasPublishervl;
      msg_output.ms_status.is_publishing = hasPublisherms;
    
    //If topic has missing fields //TODO

    //If topic has working gps
    msg_output.is_gps_ready = isGpsAvailable;


    pub_diagnostics_ -> publish(msg_output); 
  }


  
  //vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX
  // Member Variables
  
  //Timer variable
  rclcpp::TimerBase::SharedPtr timer_;


  /// Publishers
  rclcpp::Publisher<sdv_localization_msgs::msg::SensorStatArray>::SharedPtr pub_diagnostics_;

  /// Subscribers
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr sub_vn_common_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_velodyne_pcl_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_multisense_;



  /// Status variables - Initialized in false 
  bool hasInitializedvn = false;
  bool hasInitializedvl = false;
  bool hasInitializedms = false;
  bool hasPublishervn = false;
  bool hasPublishervl = false;
  bool hasPublisherms = false;
  bool isChangingvn = false;
  bool isChangingvl = false;
  bool isChangingms = false;

  //Vn state variables
  bool isGpsAvailable = false;

  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdv_diagnostics>());
  rclcpp::shutdown();
  return 0;
}