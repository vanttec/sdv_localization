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


//ROS2 node class for odometry, NED pose, and velodyne transform - 
class sbgGPSPose : public rclcpp::Node
{
  

public:
  sbgGPSPose() : Node("sbg_gps_pose_node")
  {
    //Parameter
    declare_parameter<std::vector<double>>("orientation_covariance", orientation_covariance_);
    declare_parameter<std::vector<double>>("angular_velocity_covariance", angular_velocity_covariance_);
    declare_parameter<std::vector<double>>("linear_acceleration_covariance", linear_acceleration_covariance_);
    declare_parameter<std::vector<double>>("magnetic_covariance", magnetic_field_covariance_);

    declare_parameter<std::vector<double>>("global_lla_reference", global_ref_ins_poslla_);
    declare_parameter<std::vector<double>>("global_ecef_reference", global_ref_ins_posecef_);

    // Publishers

    pub_sbg_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("sdv_localization/sbg_odom", 10);
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pub_sbg_ned_pose =  this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("sdv_localization/sbg_ned_pose", 10);
    pub_sbg_enu_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("sdv_localization/sbg_enu_pose", 10);
    
    pub_ref_ecef = this->create_publisher<geometry_msgs::msg::Point>("sdv_localization/sbg_ref_ecef", 10);
    pub_ref_ins = this->create_publisher<geometry_msgs::msg::Point>("sdv_localization/sbg_ref_ins", 10);

    pub_sbg_ned_path =  this->create_publisher<nav_msgs::msg::Path>("sdv_localizationav/sbg_ned_path", 10);
    pub_sbg_enu_path =  this->create_publisher<nav_msgs::msg::Path>("sdv_localizationav/sbg_enu_path", 10);

    
    // Subscribers
    
    auto sub_sbg_ecef_cb = std::bind(&sbgGPSPose::sub_sbg_ecef, this, std::placeholders::_1);
    sub_sbg_ecef_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "imu/pose_ecef", 10, sub_sbg_ecef_cb);

  }

private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */
  void sub_sbg_ecef(const geometry_msgs::msg::PointStamped::SharedPtr msg_in)
  {
    std::array<double, 3> global_ref_ins_poslla_;

    std::array<double, 3> global_ref_ins_posecef_;

    fill_insreference_from_param("global_lla_reference", global_ref_ins_poslla_);
    fill_insreference_from_param("global_ecef_reference", global_ref_ins_posecef_);
    
    //RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_in->header.frame_id.c_str());
    // Time Reference (Startup)

    //NED_POSE_Publish
    geometry_msgs::msg::PoseWithCovarianceStamped ned_pose_msg;

    //ENU_POSE_Publish
    geometry_msgs::msg::PoseWithCovarianceStamped enu_pose_msg;


    if(hasInitialized){

        std::array<double, 3> currECEF;;
        currECEF[0] = msg_in->point.x;
        currECEF[1] = msg_in->point.y;
        currECEF[2] = msg_in->point.z;


        std::array<double, 3> globalNED = calculateNED(global_ref_ins_posecef_, global_ref_ins_poslla_, currECEF);

        //We changed to TF2 standard messages for easier matrix rotation and subtraction

        ned_pose_msg.pose.pose.position.x = globalNED[0];
        ned_pose_msg.pose.pose.position.y = globalNED[1]; //To define 
        ned_pose_msg.pose.pose.position.z = 0;

        enu_pose_msg.pose.pose.position.x = globalNED[1]; //Change between 
        enu_pose_msg.pose.pose.position.y = globalNED[0]; 
        enu_pose_msg.pose.pose.position.z = 0;


        //Removal of pitch and roll angles, since we're only interested in working in a x,y plane 
        //tf2::Quaternion quaternionTransformNed;

        //quaternionTransformNed.setRPY(0, 0, msg_in->yawpitchroll.x);
        //ned_pose_msg.pose.pose.orientation = tf2::toMsg(quaternionTransformNed);

        tf2::Quaternion quaternionTransformEnu, quaternionResult;
        //quaternionTransformEnu.setRPY(0, 0, -90);
        //tf2::convert(msg_in->quaternion, quaternionResult);
        //quaternionResult = quaternionResult * quaternionTransformEnu;
        //quaternionResult.setZ(-quaternionResult.getZ());
        //quaternionResult.normalize();
        
        enu_pose_msg.pose.pose.orientation = tf2::toMsg(quaternionResult);

        ned_pose_msg.pose.pose.orientation = tf2::toMsg(quaternionResult);

        ned_pose_msg.header.frame_id = "odom";
        ned_pose_msg.header.set__stamp(msg_in->header.stamp);

        enu_pose_msg.header.frame_id = "odom";
        enu_pose_msg.header.set__stamp(msg_in->header.stamp);

        //Temporary 


        pub_sbg_ned_pose->publish(ned_pose_msg);
        pub_sbg_enu_pose->publish(enu_pose_msg);

        geometry_msgs::msg::PoseStamped pathToAdd;

        pathToAdd.pose = ned_pose_msg.pose.pose;
        pathToAdd.header = ned_pose_msg.header;
        ned_path.header = ned_pose_msg.header;
        ned_path.poses.push_back(pathToAdd);
        pub_sbg_ned_path->publish(ned_path);

        pathToAdd.pose = enu_pose_msg.pose.pose;
        pathToAdd.header = enu_pose_msg.header;
        enu_path.header = enu_pose_msg.header;
        enu_path.poses.push_back(pathToAdd);
        pub_sbg_enu_path->publish(enu_path);

        //Odometry publish
        // nav_msgs::msg::Odometry odom_msg;

        // odom_msg.child_frame_id = "odom";
        // odom_msg.header = enu_pose_msg.header;
        // odom_msg.pose = enu_pose_msg.pose;
        // geometry_msgs::msg::Vector3 tempVector3;
        // //Switch between Yawpitchroll in NED to ENU format = pitch <-> roll
        // tempVector3.x = msg_in->angularrate.x;
        // tempVector3.y = msg_in->angularrate.z;
        // tempVector3.z = msg_in->angularrate.y;

        // odom_msg.twist.twist.angular = tempVector3;
        
        // //Switch from NED to ENU velocity

        // // tempVector3.x = msg_in->velocity.y;
        // // tempVector3.y = msg_in->velocity.x;
        // // tempVector3.z = -msg_in->velocity.z;
        // odom_msg.twist.twist.linear = ins_velbody_;
        

        // pub_sbg_odom_->publish(odom_msg);

        //Transform odom to base_link publish in a ENU frame
        geometry_msgs::msg::TransformStamped odom2baselink_tf;

        // odom2baselink_tf.header.frame_id = "odom";
        // odom2baselink_tf.header.set__stamp(msg_in->header.stamp);
        // odom2baselink_tf.child_frame_id = "base_link";
        // odom2baselink_tf.transform.translation.x = enu_pose_msg.pose.pose.position.x;  //
        // odom2baselink_tf.transform.translation.y = enu_pose_msg.pose.pose.position.y;  //
        // odom2baselink_tf.transform.translation.z = enu_pose_msg.pose.pose.position.z;  //
        // odom2baselink_tf.transform.set__rotation(enu_pose_msg.pose.pose.orientation);  //
        // //odom_tf_broadcaster_->sendTransform(odom2baselink_tf);
        
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Reference can't be calculated, location data is 0");
    } 

  }

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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_sbg_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_sbg_ned_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_sbg_enu_pose;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ref_ins;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ref_ecef;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_sbg_ned_path;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_sbg_enu_path;

  /// Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_sbg_ecef_;


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
  nav_msgs::msg::Path enu_path;


  bool hasInitialized = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sbgGPSPose>());
  rclcpp::shutdown();
  return 0;
}