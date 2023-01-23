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
    pub_local_pose =  this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("vectornav/local_pose", 10);
    pub_ref_ecef = this->create_publisher<geometry_msgs::msg::Point>("vectornav/ref_ecef", 10);
    pub_ref_ins = this->create_publisher<geometry_msgs::msg::Point>("vectornav/ref_ins", 10);
    
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

  sensor_msgs::msg::NavSatFix ref_ins;
  geometry_msgs::msg::PoseWithCovarianceStamped ref_ecef;
  geometry_msgs::msg::PoseWithCovarianceStamped local_pose;
  bool hasRef;


private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */
  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
  {
    // RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_in->header.frame_id.c_str());

    // Time Reference (Startup)

    //Local Pose publish
    {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      //First verify if no reference is obtained

      if(!hasRef){
        bool diffFromCero;
        
        //Verifies is given ECEF Coordinates are different from cero
        diffFromCero = (ins_posecef_.x != 0 && ins_posecef_.y != 0 && ins_posecef_.z != 0);

        if(!(gps_fix_ == vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX) && diffFromCero){ //A fix is ready

          RCLCPP_INFO_ONCE(this->get_logger(), "Fix is ready!");

          geometry_msgs::msg::PoseWithCovarianceStamped newRef;
          newRef.header = msg_in-> header;
          newRef.header.frame_id = "";
          newRef.pose.pose.position = ins_posecef_;

          // Converts Quaternion in NED to ECEF
          tf2::Quaternion q, q_enu2ecef, q_ned2enu;
          q_ned2enu.setRPY(M_PI, 0.0, M_PI / 2);

          auto latitude = deg2rad(msg_in->position.x);
          auto longitude = deg2rad(msg_in->position.y);
          q_enu2ecef.setRPY(0.0, latitude, longitude);

          fromMsg(msg_in->quaternion, q);

          newRef.pose.pose.orientation = toMsg(q_ned2enu * q_enu2ecef * q);

          ref_ecef = newRef; //Check whaths happening

          sensor_msgs::msg::NavSatFix newRef2;
          newRef2.latitude = msg_in->position.x;
          newRef2.longitude = msg_in->position.y;
          newRef2.altitude = msg_in->position.z;
          
          ref_ins = newRef2;
          hasRef = true;
          RCLCPP_INFO_ONCE(this->get_logger(), "Reference is done ready!");
          RCLCPP_INFO_ONCE(this->get_logger(), "ECEF ref: %d, %d, %d", newRef.pose.pose.position.x, newRef.pose.pose.position.y, newRef.pose.pose.position.z);
          
        }
      }
      //Dosnt do anything if not ready
      if (hasRef){ //If reference is done, calculate local
      tf2::Vector3 NED;


      tf2::Matrix3x3 Rne;
        Rne.setValue(-sin(ref_ins.latitude) * cos(ref_ins.longitude), -sin(ref_ins.latitude) * sin(ref_ins.longitude), cos(ref_ins.latitude),
		   -sin(ref_ins.longitude), cos(ref_ins.longitude), 0,
		   -cos(ref_ins.latitude) * cos(ref_ins.longitude), -cos(ref_ins.latitude) * sin(ref_ins.longitude), -sin(ref_ins.latitude));
        
      tf2::Vector3 Pe;
      tf2::Vector3 Pe_ref;
      //tf2::Vector3 ActualPe;

      double newX, newY, newZ;
            
      Pe.setX(ins_posecef_.x);
      Pe.setY(ins_posecef_.y);
      Pe.setZ(ins_posecef_.z);

      Pe_ref.setX(ref_ecef.pose.pose.position.x);
      Pe_ref.setY(ref_ecef.pose.pose.position.y);
      Pe_ref.setZ(ref_ecef.pose.pose.position.z);

    
      RCLCPP_INFO(this->get_logger(), "ECEF ref: %f, %f, %f", ref_ecef.pose.pose.position.x, ref_ecef.pose.pose.position.y, ref_ecef.pose.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "ECEF pos: %f, %f, %f", ins_posecef_.x, ins_posecef_.y, ins_posecef_.z);

      newX = (ins_posecef_.x - ref_ecef.pose.pose.position.x);
      newY = (ins_posecef_.y - ref_ecef.pose.pose.position.y);
      newZ = (ins_posecef_.z - ref_ecef.pose.pose.position.z);

      RCLCPP_INFO(this->get_logger(), "X calculations is: %f, -  %f, =  %f", ins_posecef_.x, ref_ecef.pose.pose.position.x, (ins_posecef_.x - ref_ecef.pose.pose.position.x));

      
      //RCLCPP_INFO(this->get_logger(), "ECEF curr pos: %f, %f, %f", ActualPe.getX(), ActualPe.getY(), ActualPe.getZ());
      
      RCLCPP_INFO(this->get_logger(), "ECEF curr pos: %f, %f, %f", newX, newY, newZ);


      NED = Rne * (Pe - Pe_ref);

       //NED = Rne * (msg_in->position - ref_ecef.pose.pose.position); //Const problem
        double N = NED.getX();
        double E = NED.getY();
        double D = NED.getZ();

         local_pose.pose.pose.position.x = N;
         local_pose.pose.pose.position.y = E;
         local_pose.pose.pose.position.z = D;


        msg.pose.pose.position = local_pose.pose.pose.position;
        RCLCPP_INFO(this->get_logger(), "Local pos: %f, %f, %f", N, E, D);

        msg.header.frame_id = "local";
        msg.header.stamp = msg_in->header.stamp;


         pub_local_pose->publish(msg);

        geometry_msgs::msg::Point temp_ref_ecef;
        geometry_msgs::msg::Point temp_ref_ins;
        temp_ref_ecef = ref_ecef.pose.pose.position;
        temp_ref_ins.x = ref_ins.latitude;
        temp_ref_ins.y = ref_ins.longitude;
        temp_ref_ins.z = ref_ins.altitude;

         pub_ref_ecef->publish(temp_ref_ecef);
         pub_ref_ins->publish(temp_ref_ins);


      }
      //If gps fix is done, save reference ECEF, gnss and publish
    }


    // ODOM Publish
    {
      //ODOM CHECKLIST
      

      //Declaracion header, frame inicial
      geometry_msgs::msg::TransformStamped msg_tf;
      geometry_msgs::msg::TransformStamped vn_velodyne_msg_tf;
      nav_msgs::msg::Odometry msg;
      msg.header = msg_in -> header;
      msg.header.frame_id = "local";  //Frame "Earth"

      //Pose ECEF, pero queremos posicion NED Local
      msg.pose.pose.position = local_pose.pose.pose.position;        //Cambiar a pose NED
            // Converts Quaternion in NED to ECEF
      tf2::Quaternion q, q_enu2ecef, q_ned2enu;
      q_ned2enu.setRPY(M_PI, 0.0, M_PI / 2);

      auto latitude = deg2rad(msg_in->position.x);
      auto longitude = deg2rad(msg_in->position.y);
      q_enu2ecef.setRPY(0.0, latitude, longitude);

      fromMsg(msg_in->quaternion, q);

      msg.pose.pose.orientation = toMsg(q_ned2enu * q_enu2ecef * q);
      msg.header.stamp = msg_in->header.stamp;

      //NED  msg.header = msg_in->header;
      msg.twist.twist.linear = ins_velbody_;
      msg.twist.twist.angular = msg_in->angularrate;
      
      geometry_msgs::msg::Vector3 NED;

      msg.child_frame_id = "vehicle"; 
      
      //TODO Vehicle Transform checklist

      pub_odom_->publish(msg);

      msg_tf.header.stamp = msg_in->header.stamp;              
      msg_tf.header.frame_id = "local";
      msg_tf.child_frame_id = "vehicle";

      vn_velodyne_msg_tf.header.stamp = msg_in->header.stamp;
      vn_velodyne_msg_tf.header.frame_id = "local";
      vn_velodyne_msg_tf.child_frame_id = "velodyne";
      // vn_velodyne_msg_tf.transform.translation.x = 0.0;
      // vn_velodyne_msg_tf.transform.translation.y = 0.0;
      // vn_velodyne_msg_tf.transform.translation.z = 0.0;
      // vn_velodyne_msg_tf.transform.rotation.x = 0.0;
      // vn_velodyne_msg_tf.transform.rotation.y = 0.0;
      // vn_velodyne_msg_tf.transform.rotation.z = 0.0;
      // vn_velodyne_msg_tf.transform.rotation.w = 1.0;

      vn_velodyne_msg_tf.transform.translation.x = local_pose.pose.pose.position.x;
      vn_velodyne_msg_tf.transform.translation.y = local_pose.pose.pose.position.y;
      vn_velodyne_msg_tf.transform.translation.z = 0.0;
      vn_velodyne_msg_tf.transform.rotation.x = q.getX();
      vn_velodyne_msg_tf.transform.rotation.y = q.getY();
      vn_velodyne_msg_tf.transform.rotation.z = q.getZ();
      vn_velodyne_msg_tf.transform.rotation.w = q.getW();

      msg_tf.transform.translation.x = local_pose.pose.pose.position.x;
      msg_tf.transform.translation.y = local_pose.pose.pose.position.y;
      msg_tf.transform.translation.z = 0.0;
      msg_tf.transform.rotation.x = q.getX();
      msg_tf.transform.rotation.y = q.getY();
      msg_tf.transform.rotation.z = q.getZ();
      msg_tf.transform.rotation.w = q.getW();
      odom_tf_broadcaster_-> sendTransform(msg_tf);
      vn_velodyne_tf_broafcaster_->sendTransform(vn_velodyne_msg_tf);

      

      

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
    gps_posu_ = msg_in->posu;
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
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_local_pose;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ref_ins;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ref_ecef;

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

  // State Vars
  uint8_t gps_fix_ = vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX;
  geometry_msgs::msg::Vector3 gps_posu_;
  geometry_msgs::msg::Vector3 ins_velbody_;
  geometry_msgs::msg::Point ins_posecef_;
};

/// TODO(Dereck): convert to ros2 component
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VnOdomNedMsgs>());
  rclcpp::shutdown();
  return 0;
}