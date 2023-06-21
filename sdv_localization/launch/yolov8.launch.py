
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    #
    # ARGS
    #
    model_segment = LaunchConfiguration("model_segment")
    model_segment_cmd = DeclareLaunchArgument(
        "model_segment",
        default_value="/ws/RoadSeg/weights/best150.pt",
        description="Model name or path")

    model_detect = LaunchConfiguration("model_detect")
    model_detect_cmd = DeclareLaunchArgument(
        "model_detect",
        default_value="/ws/src/yolov8n.pt",
        description="Model name or path")


    tracker = LaunchConfiguration("tracker")
    tracker_cmd = DeclareLaunchArgument(
        "tracker",
        default_value="bytetrack.yaml",
        description="Tracker name or path")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Device to use (GPU/CPU)")

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",
        description="Wheter to start darknet enabled")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/rgb/image_raw",
        description="Name of the input image topic")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="yolo",
        description="Namespace for the nodes")

    #
    # NODES
    #
    segment_node_cmd = Node(
        package="yolov8_ros",
        executable="yolov8_node",
        name="yolov8_segment",
        namespace="yolo_segmentation",
        parameters=[{"model": model_segment,
                     "tracker": tracker,
                     "device": device,
                     "enable": enable,
                     "threshold": threshold}],
        #remappings=[("/yolo/segment/annotated_img", "annotated_image")]
    )
    detect_node_cmd = Node(
        package="yolov8_ros",
        executable="yolov8_node",
        name="yolov8_detection",
        namespace="yolo_objects",
        parameters=[{"model": model_detect,
                     "tracker": tracker,
                     "device": device,
                     "enable": enable,
                     "mode": "detect",
                     "threshold": threshold}],
                     
        #remappings=[("/yolo/detect/annotated_img", "annotated_image")]
    )



    ld = LaunchDescription()

    ld.add_action(model_segment_cmd)
    ld.add_action(model_detect_cmd)

    ld.add_action(tracker_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(detect_node_cmd)
    
    ld.add_action(segment_node_cmd)
    return ld
