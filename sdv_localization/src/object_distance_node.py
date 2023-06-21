#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Vector3, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from image_geometry import PinholeCameraModel, StereoCameraModel
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import ColorRGBA, Float32, String
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import math 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import LookupException, ExtrapolationException, TransformException, ConnectivityException
import tf2_ros
import tf2_geometry_msgs
class localizationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('localization_node')
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            depth = 10
            
        )

        # Declare model type (use stereo)
        self.modelType = 'pinhole'

        # Variable to check if monocular camera info is set 
        self.setG = False
        self.setD = False

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer,self)
        self.posePublisher2 = self.create_publisher(PointStamped, 'yolo_detections', 10)
        self.publisher_ = self.create_publisher(String,'/sdv/perception/traffic_status',10)

        # Variables to check if left and right camera info is set 
        self.setR = False
        self.setL = False

        # Variable where left camera info is saved temporarily
        self.leftCamInfo = None

        # Subscription to YOLO detections, change data type to published data
        
        # self.create_subscription(obj_detected_list, '/yolov7/objects_detected', self.detected_objects_callback, 10)
        #create a subscription to the 2d detections
        self.create_subscription(Detection2DArray, '/yolo_objects/detections', self.detected_objects_callback, 10)

        # if model type is stereo
        if self.modelType == 'stereo':
            # Declare a stereo camera model
            self.stereo = StereoCameraModel()
            
            # Subscribe to the disparity image, left and right camera info
            self.create_subscription(DisparityImage, '/multisense/right/disparity_image', self.dispCB, 10)
            self.create_subscription(CameraInfo, '/multisense/left/image_color/camera_info', self.infoCamRightCB, 10)
            self.create_subscription(CameraInfo, '/multisense/right/image_color/camera_info', self.infoCamLeftCB, 10)
            self.dispImg = None
        else:
            # Declare a pinhole camera model
            self.camera = PinholeCameraModel()
            # Subscribe to the camera info and depth image
            self.create_subscription(CameraInfo, '/multisense/left/image_color/camera_info', self.infoCamCB, 10)
            self.create_subscription(Image, '/multisense/left/depth', self.depthImageCB, qos_profile)
            self.depthImage = None
        self.get_logger().warn('test')
    
    def pixelto3D(self, u, v, d):
        # if modeltype is stereo
        if self.modelType == 'stereo':
            # Get disparity at pixel u, v
            disp = self.dispImg[int(v), int(u)] * -1
            # Get 3d point and transform world frame
            (y,z,x) = self.stereo.projectPixelTo3d((u,v),disp)
        else:
            # Get 3d ray that goes from pinhole through u,v pixel and 3d point
            ray = self.camera.projectPixelTo3dRay((u,v))

            # Multiply by the depth to get 3d point and transform to world frame
            (y, z, x) = [el * (d) for el in ray]
        return (x, y, z)
    
    def dispCB(self, msg):
        # Callback disparity image
        self.dispImg = CvBridge().imgmsg_to_cv2(msg.image, msg.image.encoding)

    def infoCamRightCB(self, msg):
        # Callback right camera info
        if not self.setR and self.setL:
            self.stereo.fromCameraInfo(self.leftCamInfo, msg)
            self.setR = True

    def infoCamLeftCB(self, msg):
        # Callback left camera info
        if not self.setL:
            self.stereo.left.fromCameraInfo(msg)
            self.leftCamInfo = msg
            self.setL = True
    
    def infoCamCB(self, msg):
        # Callback camera info
        if not self.setG:
            self.camera.fromCameraInfo(msg)
            self.setG = True

    def depthImageCB(self, msg):
        # Callback depth image 
        self.depthImage = CvBridge().imgmsg_to_cv2(msg,"passthrough")
        # self.get_logger().warn(str(msg))
        self.setD = True

    def detected_objects_callback(self, msg):
        # Callback YOLO detections
        # self.get_logger().warn(str(msg))
        # if camera info is set
        self.get_logger().warn("depth" + str(self.setD))
        self.get_logger().warn("cam" + str(self.setG))

        if self.setG and self.setD:
            # self.get_logger().warn(str(msg.detections))

            # For every object detected
            for detection in msg.detections:

                # Get the class id of the detection
                
                # self.get_logger().warn(str(self.depthImage))
                # Get the center of the bounding box

                u = detection.bbox.center.position.x
                v = detection.bbox.center.position.y

                # Get depth at pixel u, v. Note that you may need to convert u and v to integers.
                depth = self.depthImage[int(v),int(u)]
                self.get_logger().warn("sgh")

                self.get_logger().warn(str(depth))
                # Get x y and z position 
                x, y, z = self.pixelto3D(u, v, depth)

                # Store in point 
                p = Point()
                p.x = x
                p.y = y
                p.z = z

                self.get_logger().warn(str(p))
                # if position is valid
                if not math.isnan(p.x) and not str(p.x) == "-inf":
                    # Check if point in range, if true, publish flag
                    print(p)
                    point_x = PointStamped()
                    point_x.header.frame_id = 'multisense/head'
                    point_x.point.x = p.x
                    point_x.point.y = p.y
                    point_x.point.z = p.z


                    try:
                        self.tf_buffer.can_transform('map', 'multisense/head', self.get_clock().now())
                    except (LookupException, ConnectivityException, ExtrapolationException):
                        self.get_logger().warn("transform is not possible")
                        

                    try:
                        #object_pose_map = self.tf_buffer.transform(point_x, 'map',rclpy.duration.Duration(seconds = 1.0))
                        self.posePublisher2.publish(point_x)
                        #self.publisher_.publish(String(point_x.point.z))
                        pedestrian = 5
                        stop = 2
                        mensaje = String()
                        if z < stop:
                            data = "stop"
                        elif z < pedestrian and z > stop:
                            data = "pedestrian"

                        mensaje.data = data
                        self.publisher_.publish(mensaje)

                    except (LookupException, ConnectivityException, ExtrapolationException):
                        self.get_logger().warn("transform is not possible 2")
                    
                    # try:
                    #     self.tf_buffer.can_transform(
                    #         point_x, 'map', rospy.Duration()
                    #     )



def main():
    rclpy.init()
    node = localizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


