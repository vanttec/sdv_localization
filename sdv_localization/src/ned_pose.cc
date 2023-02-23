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
class VnNedPose : public rclcpp::Node
{
  

public:
  VnNedPose() : Node("VnNedPoseNode")
  {
    //Parameter
    declare_parameter<std::vector<double>>("orientation_covariance", orientation_covariance_);
    declare_parameter<std::vector<double>>("angular_velocity_covariance", angular_velocity_covariance_);
    declare_parameter<std::vector<double>>("linear_acceleration_covariance", linear_acceleration_covariance_);
    declare_parameter<std::vector<double>>("magnetic_covariance", magnetic_field_covariance_);

    declare_parameter<std::vector<double>>("global_lla_reference", global_ref_ins_poslla_);
    declare_parameter<std::vector<double>>("global_ecef_reference", global_ref_ins_posecef_);

    // Publishers

    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("vectornav/odom", 10);
    vn_velodyne_tf_broafcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pub_ned_pose =  this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("vectornav/ned_pose", 10);
    pub_ref_ecef = this->create_publisher<geometry_msgs::msg::Point>("vectornav/ref_ecef", 10);
    pub_ref_ins = this->create_publisher<geometry_msgs::msg::Point>("vectornav/ref_ins", 10);
    pub_ned_path =  this->create_publisher<nav_msgs::msg::Path>("vectornav/ned_path", 10);
    
    
    // Subscribers
    
    auto sub_vn_common_cb = std::bind(&VnNedPose::sub_vn_common, this, std::placeholders::_1);
    sub_vn_common_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", 10, sub_vn_common_cb);

    auto sub_vn_time_cb = std::bind(&VnNedPose::sub_vn_time, this, std::placeholders::_1);
    sub_vn_time_ = this->create_subscription<vectornav_msgs::msg::TimeGroup>(
      "vectornav/raw/time", 10, sub_vn_time_cb);

    auto sub_vn_imu_cb = std::bind(&VnNedPose::sub_vn_imu, this, std::placeholders::_1);
    sub_vn_imu_ = this->create_subscription<vectornav_msgs::msg::ImuGroup>(
      "vectornav/raw/imu", 10, sub_vn_imu_cb);

    auto sub_vn_gps_cb = std::bind(&VnNedPose::sub_vn_gps, this, std::placeholders::_1);
    sub_vn_gps_ = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "vectornav/raw/gps", 10, sub_vn_gps_cb);

    auto sub_vn_attitude_cb =
      std::bind(&VnNedPose::sub_vn_attitude, this, std::placeholders::_1);
    sub_vn_attitude_ = this->create_subscription<vectornav_msgs::msg::AttitudeGroup>(
      "vectornav/raw/attitude", 10, sub_vn_attitude_cb);

    auto sub_vn_ins_cb = std::bind(&VnNedPose::sub_vn_ins, this, std::placeholders::_1);
    sub_vn_ins_ = this->create_subscription<vectornav_msgs::msg::InsGroup>(
      "vectornav/raw/ins", 10, sub_vn_ins_cb);

    auto sub_vn_gps2_cb = std::bind(&VnNedPose::sub_vn_gps2, this, std::placeholders::_1);
    sub_vn_gps2_ = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "vectornav/raw/gps2", 10, sub_vn_gps2_cb);
  }



private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */
  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
  {
    std::array<double, 3> global_ref_ins_poslla_;
    std::array<double, 3> global_ref_ins_posecef_;

    fill_insreference_from_param("global_lla_reference", global_ref_ins_poslla_);
    fill_insreference_from_param("global_ecef_reference", global_ref_ins_posecef_);
    
    RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_in->header.frame_id.c_str());

    // Time Reference (Startup)

    //GLOBAL is NED with a constant reference
    //LOCAL is NED with a reference defined as the first lla, ecef coordinate received

    //NED_POSE_Publish
    geometry_msgs::msg::PoseWithCovarianceStamped ned_pose_msg;
    if(hasInitialized){
        std::array<double, 3> currECEF;;
        currECEF[0] = ins_posecef_.x;
        currECEF[1] = ins_posecef_.y;
        currECEF[2] = ins_posecef_.z;


        std::array<double, 3> globalNED = calculateNED(global_ref_ins_posecef_, global_ref_ins_poslla_, currECEF);

        //We changed to TF2 standard messages for easier matrix rotation and subtraction


        // VectorPoseInNED = RotMatEcef2Ned * (VectorPoseEcef - VectorRefEcef);

        ned_pose_msg.pose.pose.position.x = globalNED[0];
        ned_pose_msg.pose.pose.position.y = globalNED[1];
        ned_pose_msg.pose.pose.position.z = 0;
        ned_pose_msg.pose.pose.orientation = msg_in->quaternion;

        

        ned_pose_msg.header.frame_id = "odom";
        ned_pose_msg.header.set__stamp(msg_in->header.stamp);

        pub_ned_pose->publish(ned_pose_msg);

        geometry_msgs::msg::PoseStamped pathToAdd;

        pathToAdd.pose = ned_pose_msg.pose.pose;
        pathToAdd.header = ned_pose_msg.header;

        ned_path.header = ned_pose_msg.header;

        ned_path.poses.push_back(pathToAdd);

        pub_ned_path->publish(ned_path);

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

        map2odom_tf.header.frame_id = "odom";
        map2odom_tf.header.stamp = odom_msg.header.stamp;
        map2odom_tf.child_frame_id = "base_link";
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
    //RCLCPP_INFO(this->get_logger(), "posecef: %f, %f, %f", msg_in->posecef.x, msg_in->posecef.y, msg_in->posecef.z);
    ins_velbody_ = msg_in->velbody;
    ins_posecef_ = msg_in->posecef;
    ins_poslla_ = msg_in->poslla;
    ins_velned_ = msg_in->velned;
    
    //If the system still hasnt initialized
    if(!hasInitialized){
    
        //If the system has data different than zero
        if(ins_posecef_.x!=0 && ins_posecef_.y!=0 && ins_poslla_.x!=0 & ins_poslla_.y!= 0){       

            hasInitialized = true;
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


    void fill_insreference_from_param(std::string param_name, std::array<double, 3> & array) const
  {
    auto references = get_parameter(param_name).as_double_array();


    auto length = references.size();
    switch (length) {
      case 2:
        array[0] = references[0];
        array[1] = references[1];
        break;

      case 3:
        array[0] = references[0];
        array[1] = references[1];
        array[2] = references[2];
        break;

      default:
        RCLCPP_ERROR(
          get_logger(), "Parameter '%s' length is %zu; expected 1, 3, or 9", param_name.c_str(),length);
    }
    
  }

  //Calculates NED position, without orientation or Z value
  std::array<double, 3> calculateNED(std::array<double, 3> ecefref, std::array<double, 3> llaref, std::array<double, 3> ecefcurr){
    
    std::array<double, 3> NED;  //X(N), Y(E), Z(D)

     float latInRads = deg2rad(llaref[0]); 
      float lonInRads = deg2rad(llaref[1]);

        tf2::Matrix3x3 RotMatEcef2Ned;
        
        
        RotMatEcef2Ned.setValue(-sin(latInRads) * cos(lonInRads), -sin(latInRads) * sin(lonInRads), cos(latInRads),
		   -sin(lonInRads), cos(lonInRads), 0,
		   -cos(latInRads) * cos(lonInRads), -cos(latInRads) * sin(lonInRads), -sin(latInRads));
            
        tf2::Vector3 VectorRefEcef, VectorPoseEcef;

        VectorRefEcef.setX(ecefref[0]);
        VectorRefEcef.setY(ecefref[1]);
        VectorRefEcef.setZ(ecefref[2]);

        VectorPoseEcef.setX(ecefcurr[0]);
        VectorPoseEcef.setY(ecefcurr[1]);
        VectorPoseEcef.setZ(ecefcurr[2]);

        //We changed to TF2 standard messages for easier matrix rotation and subtraction

        tf2::Vector3 VectorPoseInNED;

        VectorPoseInNED = RotMatEcef2Ned * (VectorPoseEcef - VectorRefEcef);

        NED[0] = VectorPoseInNED.getX();
        NED[1] = VectorPoseInNED.getY();
        NED[2] = 0;

        return NED;

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


  //Parameters
  const std::vector<double> global_ref_ins_poslla_ = {25.65014586802158, -100.28985364572286}; // Coordenadas entre Biblio y CETEC
  const std::vector<double> global_ref_ins_posecef_ = {-1027768.8799482058, -5661145.344370203, 2744403.2051628013};               

  /// TODO(Dereck): Find default covariance values

  // Parameter declaration
  
  

  //Vars to store data from the INS Common groups
  uint8_t gps_fix_ = vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX;
  geometry_msgs::msg::Vector3 ins_velbody_;
  geometry_msgs::msg::Point ins_posecef_;
  geometry_msgs::msg::Point ins_poslla_;
  geometry_msgs::msg::Vector3 ins_velned_;

  //Path message to publish
  nav_msgs::msg::Path ned_path;


  bool hasInitialized = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VnNedPose>());
  rclcpp::shutdown();
  return 0;
}