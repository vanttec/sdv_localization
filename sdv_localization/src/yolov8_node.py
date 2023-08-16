import cv2
import torch
import random

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.tracker import BOTSORT, BYTETracker
from ultralytics.tracker.trackers.basetrack import BaseTrack
from ultralytics.yolo.utils import IterableSimpleNamespace, yaml_load
from ultralytics.yolo.utils.checks import check_requirements, check_yaml

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import SetBool


class Yolov8Node(Node):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # params
        self.declare_parameter("model", "/ws/RoadSeg/weights/best150.pt")
        model = self.get_parameter(
            "model").get_parameter_value().string_value
        self.get_logger().warn(f"model: {model}")
        self.declare_parameter("tracker", "bytetrack.yaml")
        tracker = self.get_parameter(
            "tracker").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")
        device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value
        
        self.declare_parameter("mode", "segment")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        self._class_to_color = {}
        self.cv_bridge = CvBridge()
        self.tracker = self.create_tracker(tracker)
        self.yolo = YOLO(model)
        #self.yolo.fuse()
        self.yolo.to(device)

        # topcis
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._dbg_pub = self.create_publisher(Image, "annotated_image", 10)
        self._sub = self.create_subscription(
            Image, "/multisense/left/image_color", self.new_image_cb,
            qos_profile_sensor_data
        )
        self.myimage=Image()
        # services
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)
        self.get_logger().warn('test2' )
        #timer_period = 0.1 #1 second
        #self.timer=self.create_timer(timer_period,self.timer_callback)


    def create_tracker(self, tracker_yaml) -> BaseTrack:

        TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
        check_requirements("lap")  # for linear_assignment

        tracker = check_yaml(tracker_yaml)
        cfg = IterableSimpleNamespace(**yaml_load(tracker))

        assert cfg.tracker_type in ["bytetrack", "botsort"], \
            f"Only support 'bytetrack' and 'botsort' for now, but got '{cfg.tracker_type}'"
        tracker = TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=1)
        return tracker

    def enable_cb(self,
                  req: SetBool.Request,
                  res: SetBool.Response
                  ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res
    #def timer_callback(self):
        #self.get_logger().warn('si funciona '+str(self.myimage ))
        # publish detections and dbg image
        #self._pub.publish(detections_msg)
        #self._dbg_pub.publish(self.myimage)#self.cv_bridge.cv2_to_imgmsg(cv_image,encoding=msg.encoding))

    import numpy as np

    def new_image_cb(self, msg):
        self.myimage = msg

        if True:
            # Convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            #self.get_logger().warn('test')
            #self.get_logger().warn(str(msg))
            if self.mode == "segment":
                results_list = self.yolo(
                    source=cv_image, conf=0.25, verbose=True, mode="predict", task="segment")
            else:
                results_list = self.yolo(
                    source=cv_image, conf=0.25, verbose=True, mode="predict", task="detect")
            cv_image = results_list[0].plot()

            # Get the segmented area as a binary mask
            results = results_list[0]
            if self.mode == "segment" and results.masks is not None:
               mask_tensor = results.masks.data  # Get the masks tensor directly
               mask = mask_tensor[0].cpu().numpy()

               # Calculate the centroid of the segmented area if moments are valid
               M = cv2.moments(mask, binaryImage=True)
               if M["m00"] != 0:
                 centroid_x = int(M["m10"] / M["m00"])
                 centroid_y = int(M["m01"] / M["m00"])
                 # Calculate the centroid relative to the bounding box
                 bbox = results.boxes.data[0]
                 bbox_center_x = bbox[0] + bbox[2] / 2
                 bbox_center_y = bbox[1] + bbox[3] / 2
                 centroid_x = int(bbox_center_x + (centroid_x - bbox_center_x) * 0.5)
                 centroid_y = int(bbox_center_y + (centroid_y - bbox_center_y) * 0.5)
                 # Draw the red circle at the centroid
                 cv2.circle(cv_image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)

            if self.mode == "detect" and len(results):
                det = results[0].boxes.cpu().numpy()
                self.get_logger().info(f"{det}")
                if len(det) > 0:
                    im0s = self.yolo.predictor.batch[2]
                    im0s = im0s if isinstance(im0s, list) else [im0s]

                    tracks = self.tracker.update(det, im0s[0])
                    if len(tracks) > 0:
                        results[0].update(boxes=torch.as_tensor(tracks[:, :-1]))

                # create detections msg
                detections_msg = Detection2DArray()
                detections_msg.header = msg.header

                results = results[0].cpu()

                for b in results.boxes:
                    label = self.yolo.names[int(b.cls)]
                    score = float(b.conf)

                    if score < self.threshold:
                        continue

                    detection = Detection2D()

                    box = b.xywh[0]

                    # get boxes values
                    detection.bbox.center.position.x = float(box[0])
                    detection.bbox.center.position.y = float(box[1])
                    detection.bbox.size_x = float(box[2])
                    detection.bbox.size_y = float(box[3])

                    # get track id
                    track_id = ""
                    if not b.id is None:
                        track_id = str(int(b.id))
                    detection.id = track_id

                    # get hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = label
                    hypothesis.hypothesis.score = score
                    detection.results.append(hypothesis)
                    detections_msg.detections.append(detection)
                    self._pub.publish(detections_msg)


            # Publish the modified image with the red circle
            self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=msg.encoding))
            
        return






    def image_cb(self, msg):
        self.myimage =msg
        self.get_logger().warn("Callback in")  # Debugging message

	
        if True:
            

            # convert im        results = results_list[0]elf.cv_bridge.imgmsg_to_cv2(msg)
            #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            #self.get_logger().warn('afer "%s"' % str(cv_image) )
            results = self.yolo(
                source=cv_image, conf=0.25, verbose=True, mode="predict", task="segment")
            #self.get_logger().info(f"boxes: {results[0].boxes.data}")
            #self.get_logger().info(f"results: {results}")
            im = results[0].plot()
            #self.get_logger().info("im: {im}")
            self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(im,
                                                               encoding=msg.encoding))

            
            return

            #results = self.yolo(
            #    source=cv_image,
            #    verbose=True,
            #    stream=False,
            #    conf=0.25,
            #    task='segment',
            #    mode="predict"
            #)
            #print(results)

            # track
            det = results[0].boxes.cpu().numpy()
            self.get_logger().info(f"{det}")
            if len(det) > 0:
                im0s = self.yolo.predictor.batch[2]
                im0s = im0s if isinstance(im0s, list) else [im0s]

                tracks = self.tracker.update(det, im0s[0])
                if len(tracks) > 0:
                    results[0].update(boxes=torch.as_tensor(tracks[:, :-1]))

            # create detections msg
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            results = results[0].cpu()
            self.get_logger().warn("Entering")  # Debugging message

            for b in results.boxes:
                self.get_logger().warn("entered")
                self.get_logger().info(f"{hypothesis}")


                label = self.yolo.names[int(b.cls)]
                score = float(b.conf)

                if score < self.threshold:
                    continue

                detection = Detection2D()

                box = b.xywh[0]

                # get boxes values
                detection.bbox.center.position.x = float(box[0])
                detection.bbox.center.position.y = float(box[1])
                detection.bbox.size_x = float(box[2])
                detection.bbox.size_y = float(box[3])

                # get track id
                track_id = ""
                if not b.id is None:
                    track_id = str(int(b.id))
                detection.id = track_id

                # get hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)

                self.get_logger().info(f"{hypothesis}")
                print('Hello')

                # draw boxes for debug
                if label not in self._class_to_color:
                    r = random.randint(0, 255)
                    g = random.randint(0, 255)
                    b = random.randint(0, 255)
                    self._class_to_color[label] = (r, g, b)
                color = self._class_to_color[label]

                min_pt = (round(detection.bbox.center.position.x - detection.bbox.size_x / 2.0),
                          round(detection.bbox.center.position.y - detection.bbox.size_y / 2.0))
                max_pt = (round(detection.bbox.center.position.x + detection.bbox.size_x / 2.0),
                          round(detection.bbox.center.position.y + detection.bbox.size_y / 2.0))
                cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

                label = "{} ({}) ({:.3f})".format(label, str(track_id), score)
                pos = (min_pt[0] + 5, min_pt[1] + 25)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, label, pos, font,
                            1, color, 1, cv2.LINE_AA)

                # append msg
                detections_msg.detections.append(detection)

            # publish detections and dbg image
            self._pub.publish(detections_msg)
            self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image,
                                                               encoding=msg.encoding))


def main(args=None):
    rclpy.init(args=args)
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    
    main()
