# sdv_localization
Localization module for sdv.

This ROS2 package provides the necessary drivers to the VN-300 and Velodyne HDL32-E Sensor for use in VantTec's Self Driving Vehicle.

## Building package

This tutorial assumes you already have a functioning ROS2 distro installed in your operating system.

NOTE: Some launch scripts require the isaac_ros_visual_slam package to correctly execute

1. In your workspace create a `src` folder and add this repo using `git clone`
2. Inside the repository, run `git submodule update --init --recursive`
3. From your workspace folder, run `rosdep install --from-paths src --ignore-src -r -y` to install dependencies using the `package.xml` from each package
4. Run `echo "source install/setup.bash" >> ~/.bashrc` to add the ROS2 package to the path
   
## Running sensor's drivers

The multisense and velodyne sensors are configured at 'X.X.X.X' and at 'X.X.X.X' with an MTU of 'XXXX', henceforth the connected computer must have an IP in the same network, we use 'X.X.X.X' by default. The vectornav sensor by default is connected at `/dev/ttyUSB0` and write/read permissions must be allowed using `sudo chmod 777 /dev/ttyUSB0`

### Launching velodyne sensor

Run `ros2 launch sdv_localization velodyne.launch.py`

### Launching multisense sensor

Run `ros2 launch sdv_localization multisense_launch.py`

### Launching vectornav sensor

For the default launch file, run `ros2 launch vectornav vectornav.launch.py`

For the sdv_localization launch file, run `ros2 launch sdv_localization vectornav.launch.py`

The difference is in the configuration, the default configuration fills the 'common' binary output. The sdv_localization launch file fills the 'common', 'ins' and 'attitude' binary outputs. For further reference, you can consult section 5 in the [technical document](https://smode.whoi.edu/waveglider/doc/VN-300.pdf)

## Running localization algorithms

The localization algorithms are GPS dependent, meaning they won't initialize if the VN-300 sensor can't lock in a GPS signal. On average, 15 minutes in a clear-sky area is required for this.

To monitor and verify when the status is ready, you can run `ros2 topic echo /vectornav/raw/common` and when the `status` field has a value of 2, it means it is receiving GPS signals and the onboard compass is correctly aligned with the GPS.

To start publishing pose and orientation, run `ros2 launch sdv_localization sdv_enu_tf.launch.py`

To visualize the model of the car, run `ros2 launch sdv_localization model_sdv_.launch.py`

To visualize the current map with the SDV's position, go to `sdv_localization/sdv_localization/config` and run `map_broadcaster.launch.py`

Note: All elements must be added in RVIZ2 for visualization, additionally, the map topic's configuration may need to be changed to 'volatile' to correctly work.

## Mapping

The use of the slam_toolbox package is required to map and use the information. As of now, mapping efforts have been only applied on rosbags and not on real-time due to limits in processing power. 

While running the rosbag with the `/scan` and `/vectornav/raw/common` topics, run `ros2 launch sdv_localization sdv_enu_tf.launch.py`

If everything can be visualized, you can run `ros2 launch sdv_localization rosbag_mapping.launch.py`

To save the map, in Rviz2 go to 'Panels' -> 'Add New Panel' -> 'slam_toolbox/SlamToolboxPlugin' and press 'Save Map' on the new window.
