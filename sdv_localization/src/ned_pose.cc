#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"


//ROS2 node class for odometry, NED pose, and velodyne transform - 
class VnOdomNedMsgs : public rclcpp::Node
{
  

public:
  VnOdomNedMsgs() : Node("vn_odom_ned_msgs")
  {
    //Parameter
    declare_parameter<std::vector<double>>("orientation_covariance", orientation_covariance_);
    declare_parameter<std::vector<double>>("angular_velocity_covariance", angular_velocity_covariance_);
    declare_parameter<std::vector<double>>("linear_acceleration_covariance", linear_acceleration_covariance_);
    declare_parameter<std::vector<double>>("magnetic_covariance", magnetic_field_covariance_);

    // Publishers

    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("vectornav/odom", 10);
    
    vn_velodyne_tf_broafcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pub_ned_pose =  this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("vectornav/ned_pose", 10);
    pub_ref_ecef = this->create_publisher<geometry_msgs::msg::Point>("vectornav/ref_ecef", 10);
    pub_ref_ins = this->create_publisher<geometry_msgs::msg::Point>("vectornav/ref_ins", 10);
    pub_ned_path =  this->create_publisher<nav_msgs::msg::Path>("vectornav/ned_path", 10);
    
    // Subscribers
    
    auto sub_vn_common_cb = std::bind(&VnOdomNedMsgs::sub_vn_common, this, std::placeholders::_1);
    sub_vn_common_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", 10, sub_vn_common_cb);

    auto sub_vn_time_cb = std::bind(&VnOdomNedMsgs::sub_vn_time, this, std::placeholders::_1);
    sub_vn_time_ = this->create_subscription<vectornav_msgs::msg::TimeGroup>(
      "vectornav/raw/time", 10, sub_vn_time_cb);

    auto sub_vn_imu_cb = std::bind(&VnOdomNedMsgs::sub_vn_imu, this, std::placeholders::_1);
    sub_vn_imu_ = this->create_subscription<vectornav_msgs::msg::ImuGroup>(
      "vectornav/raw/imu", 10, sub_vn_imu_cb);

    auto sub_vn_gps_cb = std::bind(&VnOdomNedMsgs::sub_vn_gps, this, std::placeholders::_1);
    sub_vn_gps_ = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "vectornav/raw/gps", 10, sub_vn_gps_cb);

    auto sub_vn_attitude_cb =
      std::bind(&VnOdomNedMsgs::sub_vn_attitude, this, std::placeholders::_1);
    sub_vn_attitude_ = this->create_subscription<vectornav_msgs::msg::AttitudeGroup>(
      "vectornav/raw/attitude", 10, sub_vn_attitude_cb);

    auto sub_vn_ins_cb = std::bind(&VnOdomNedMsgs::sub_vn_ins, this, std::placeholders::_1);
    sub_vn_ins_ = this->create_subscription<vectornav_msgs::msg::InsGroup>(
      "vectornav/raw/ins", 10, sub_vn_ins_cb);

    auto sub_vn_gps2_cb = std::bind(&VnOdomNedMsgs::sub_vn_gps2, this, std::placeholders::_1);
    sub_vn_gps2_ = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "vectornav/raw/gps2", 10, sub_vn_gps2_cb);
  }



private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */
  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
  {
    RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_in->header.frame_id.c_str());

    // Time Reference (Startup)


    //NED_POSE_Publish
    geometry_msgs::msg::PoseWithCovarianceStamped ned_pose_msg;
    if(hasRef){
        
        float latInRads = deg2rad(ref_ins_poslla_.x); 
        float lonInRads = deg2rad(ref_ins_poslla_.y);

        tf2::Matrix3x3 RotMatEcef2Ned;
        
        
        RotMatEcef2Ned.setValue(-sin(latInRads) * cos(lonInRads), -sin(latInRads) * sin(lonInRads), cos(latInRads),
		   -sin(lonInRads), cos(lonInRads), 0,
		   -cos(latInRads) * cos(lonInRads), -cos(latInRads) * sin(lonInRads), -sin(latInRads));
            
        tf2::Vector3 VectorRefEcef, VectorPoseEcef;

        VectorRefEcef.setX(ref_ins_posecef_.x);
        VectorRefEcef.setY(ref_ins_posecef_.y);
        VectorRefEcef.setZ(ref_ins_posecef_.z);

        VectorPoseEcef.setX(ins_posecef_.x);
        VectorPoseEcef.setY(ins_posecef_.y);
        VectorPoseEcef.setZ(ins_posecef_.z);

        //We changed to TF2 standard messages for easier matrix rotation and subtraction

        tf2::Vector3 VectorPoseInNED;

        VectorPoseInNED = RotMatEcef2Ned * (VectorPoseEcef - VectorRefEcef);

        ned_pose_msg.pose.pose.position.x = VectorPoseInNED.getX();
        ned_pose_msg.pose.pose.position.y = VectorPoseInNED.getY();
        ned_pose_msg.pose.pose.position.z = 0;
        ned_pose_msg.pose.pose.orientation = msg_in->quaternion;

        ned_pose_msg.header.frame_id = "map";
        ned_pose_msg.header.set__stamp(msg_in->header.stamp);

        pub_ned_pose->publish(ned_pose_msg);

        geometry_msgs::msg::PoseStamped pathToAdd;

        pathToAdd.pose = ned_pose_msg.pose.pose;
        pathToAdd.header = ned_pose_msg.header;

        NED_path.header = ned_pose_msg.header;

        NED_path.poses.push_back(pathToAdd);

        pub_ned_path->publish(NED_path);

        //Odometry publish
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.child_frame_id = "odom";
        odom_msg.header = ned_pose_msg.header;
        odom_msg.pose = ned_pose_msg.pose;
        odom_msg.twist.twist.angular = msg_in->angularrate;
        odom_msg.twist.twist.linear = msg_in->velocity;
        
        pub_odom_->publish(odom_msg);

        //Transform map -> odom publish
        geometry_msgs::msg::TransformStamped map2odom_tf;

        map2odom_tf.header.frame_id = "map";
        map2odom_tf.header.stamp = odom_msg.header.stamp;
        map2odom_tf.child_frame_id = "odom";
        map2odom_tf.transform.translation.x = ned_pose_msg.pose.pose.position.x;
        map2odom_tf.transform.translation.y = ned_pose_msg.pose.pose.position.y;
        map2odom_tf.transform.translation.z = ned_pose_msg.pose.pose.position.z;
        map2odom_tf.transform.set__rotation(msg_in->quaternion);

        odom_tf_broadcaster_->sendTransform(map2odom_tf);

        




    }
    else{
        RCLCPP_INFO(get_logger(), "Reference can't be calculated, location data is 0");
    }
        
     
      

    

    
  }

  /** Convert VN time group data to ROS2 standard message types
   *
   */
  void sub_vn_time(const vectornav_msgs::msg::TimeGroup::SharedPtr msg_in) const {}

  /** Convert VN imu group data to ROS2 standard message types
   *
   */
  void sub_vn_imu(const vectornav_msgs::msg::ImuGroup::SharedPtr msg_in) const {}

  /** Convert VN gps group data to ROS2 standard message types
   *
   * TODO(Dereck): Consider alternate sync methods
   */
  void sub_vn_gps(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in)
  {
    gps_fix_ = msg_in->fix;


  }

  /** Convert VN attitude group data to ROS2 standard message types
   *
   */
  void sub_vn_attitude(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg_in) const {}

  /** Convert VN ins group data to ROS2 standard message types
   *
   */
  void sub_vn_ins(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in)
  {
    RCLCPP_INFO(this->get_logger(), "posecef: %f, %f, %f", msg_in->posecef.x, msg_in->posecef.y, msg_in->posecef.z);
    ins_velbody_ = msg_in->velbody;
    ins_posecef_ = msg_in->posecef;
    ins_poslla_ = msg_in->poslla;
    ins_velned_ = msg_in->velned;
    
    //If the system still doesn't have a reference
    if(!hasRef){
    
        //If the system has data different than zero
        if(ins_posecef_.x!=0 && ins_posecef_.y!=0 && ins_poslla_.x!=0 & ins_poslla_.y!= 0){
            ref_ins_posecef_ = ins_posecef_;
            ref_ins_poslla_ = ins_poslla_;

            hasRef = true;
        }
    }
  }

  /** Convert VN gps2 group data to ROS2 standard message types
   *
   */
  void sub_vn_gps2(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in) const {}

  /** Copy a covariance matrix array from a parameter into a msg array
   *
   * If a single value is provided, this will set the diagonal values
   * If three values are provided, this will set the diagonal values
   * If nine values are provided, this will fill the matrix
   *
   * \param param_name Name of the parameter to read
   * \param array Array to fill
   */
  void fill_covariance_from_param(std::string param_name, std::array<double, 9> & array) const
  {
    auto covariance = get_parameter(param_name).as_double_array();

    auto length = covariance.size();
    switch (length) {
      case 1:
        array[0] = covariance[0];
        array[3] = covariance[0];
        array[8] = covariance[0];
        break;

      case 3:
        array[0] = covariance[0];
        array[3] = covariance[1];
        array[8] = covariance[3];
        break;

      case 9:
        std::copy(covariance.begin(), covariance.end(), array.begin());
        break;

      default:
        RCLCPP_ERROR(
          get_logger(), "Parameter '%s' length is %zu; expected 1, 3, or 9", param_name.c_str(),
          length);
    }
  }

  /// Convert from DEG to RAD
  inline static double deg2rad(double in) { return in * M_PI / 180.0; }

  //vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX
  // Member Variables
  //

  /// Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> vn_velodyne_tf_broafcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_ned_pose;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ref_ins;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ref_ecef;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ned_path;

  /// Subscribers
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr sub_vn_common_;
  rclcpp::Subscription<vectornav_msgs::msg::TimeGroup>::SharedPtr sub_vn_time_;
  rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr sub_vn_imu_;
  rclcpp::Subscription<vectornav_msgs::msg::GpsGroup>::SharedPtr sub_vn_gps_;
  rclcpp::Subscription<vectornav_msgs::msg::AttitudeGroup>::SharedPtr sub_vn_attitude_;
  rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr sub_vn_ins_;
  rclcpp::Subscription<vectornav_msgs::msg::GpsGroup>::SharedPtr sub_vn_gps2_;

  /// Default orientation Covariance
  const std::vector<double> orientation_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                       0.0000, 0.0000, 0.0000, 0.0000};

  /// Default angular_velocity Covariance
  const std::vector<double> angular_velocity_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                            0.0000, 0.0000, 0.0000, 0.0000};

  /// Default linear_acceleration Covariance
  const std::vector<double> linear_acceleration_covariance_ = {
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};

  /// Default magnetic field Covariance
  const std::vector<double> magnetic_field_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                          0.0000, 0.0000, 0.0000, 0.0000};

  /// TODO(Dereck): Find default covariance values

  //Vars to store data from the INS Common groups
  uint8_t gps_fix_ = vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX;
  geometry_msgs::msg::Vector3 ins_velbody_;
  geometry_msgs::msg::Point ins_posecef_;
  geometry_msgs::msg::Point ins_poslla_;
  geometry_msgs::msg::Vector3 ins_velned_;

  //Path message to publish
  nav_msgs::msg::Path NED_path;

  //Reference data
  geometry_msgs::msg::Point ref_ins_posecef_;
  geometry_msgs::msg::Point ref_ins_poslla_;
  bool hasRef = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VnOdomNedMsgs>());
  rclcpp::shutdown();
  return 0;
}