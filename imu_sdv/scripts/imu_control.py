#!/usr/bin/env python

import math
from math import sin, cos, pi
import subprocess
import rospy
import tf
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3,PoseStamped
from sensor_msgs.msg import NavSatFix

class ImuControl:
    def __init__(self) -> None:
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.path_pub = rospy.Publisher("/sdv/path1", Path, queue_size=50)
        self.imu_sub = rospy.Subscriber("/vectornav/Odom", Odometry, self.imu_callback)
        self.imu_sub = rospy.Subscriber("/vectornav/GPS", NavSatFix, self.gps_callback)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.sdv_path=Path()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.1
        self.vy = -0.1
        self.vth = 0.1

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.past_x=-1
        self.past_y=-1
        self.past_z=-1
        self.past_th=-1
        self.imu_longitude=0
        self.imu_latitude=0
        self.enable_imu=False
        self.current_node=None
        self.origin_x=-100.289790
        self.origin_y=25.651249
        self.offsetx=64824.56666
        self.offsety=350000
        self.offset_angle=(180-20)*(math.pi/180)
        #360+20
    def imu_callback(self,data):
        self.vx=data.twist.twist.linear.x
        self.vy=data.twist.twist.linear.y
        self.vz=data.twist.twist.linear.z
        self.vth=data.twist.twist.angular.z
    def gps_callback(self,data):
        self.imu_latitude=data.latitude
        self.imu_longitude=data.longitude
            

    def imu_odom(self):
        if self.imu_longitude!=0 and self.imu_latitude!=0 and self.enable_imu==False:
            new_x=(self.imu_longitude-self.origin_x )*self.offsetx
            new_y=(self.imu_latitude-self.origin_y)*self.offsety
            current_coord=str(new_x)+" "+str(new_y)+" "+"0 "+str(self.offset_angle)+" 0 0 map odom2"
            rospy.loginfo("New coord")
            rospy.loginfo(self.imu_longitude-self.origin_x )
            rospy.loginfo(self.imu_latitude-self.origin_y)
            rospy.loginfo(current_coord)
            self.current_node = subprocess.Popen('exec ' + "rosrun tf2_ros static_transform_publisher "+current_coord, stdout=subprocess.PIPE, shell = True )
            self.enable_imu=True
        if self.enable_imu==False:
            rospy.loginfo("No GPS")
            rospy.loginfo(self.imu_longitude)
            rospy.loginfo(self.imu_latitude)

        if self.enable_imu:
            self.current_time = rospy.Time.now()
            # compute odometry in a typical way given the velocities of the robot
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
            delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
            delta_th = self.vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # obtain a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, -self.y, 0.),
                odom_quat,
                self.current_time,
                "base_link2",
                "odom2"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom2"

            # set the position
            odom.pose.pose = Pose(Point(self.x, -self.y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link2"
            odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

            # publish the message
            self.odom_pub.publish(odom)
            self.last_time = self.current_time
            #Draw path
            if self.x != self.past_x or -self.y != self.past_y or self.th != self.past_th:
                self.sdv_path.header.frame_id = "odom2"
                self.sdv_path.header.stamp = rospy.Time.now()
                pose = PoseStamped()
                pose.pose.position.x = self.x
                pose.pose.position.y = -self.y
                pose.pose.position.z = 0

                quaternion = odom_quat
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                self.sdv_path.poses.append(pose)
                self.past_x=self.x
                self.past_y=-self.y
                self.past_th=self.th
                self.path_pub.publish(self.sdv_path)




def main():
    rospy.init_node("odometry_publisher", anonymous=False)
    rate = rospy.Rate(20)
    imu_control = ImuControl()
    while not rospy.is_shutdown():
        imu_control.imu_odom()
        rate.sleep()
    imu_control.current_node.terminate()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass