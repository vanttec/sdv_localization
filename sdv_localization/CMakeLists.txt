cmake_minimum_required(VERSION 3.5)
project(sdv_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# TODO[DERECK] fix library warnings
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# endif()

# find dependencies
# Possibly add for rosdep 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sdv_localization_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# vncxx
add_subdirectory(vnproglib-1.2.0.0/cpp)

#vn_odom_ned_msgs

add_executable(vn_gps_pose src/gps_pose.cc)
add_executable(diagnostics src/diagnostics.cc)
add_executable(vslam_repub src/visual_slam_republisher.cc)
#add_executable(isaac_awake src/isaac_vslam_awake2.cpp)
add_executable(velocity_visualizer src/odom_velocity_visualizer.cpp)
ament_target_dependencies(vn_gps_pose rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs)
ament_target_dependencies(diagnostics rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs sdv_localization_msgs)
ament_target_dependencies(vslam_repub rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs sdv_localization_msgs)
ament_target_dependencies(velocity_visualizer rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs sdv_localization_msgs visualization_msgs)
#ament_target_dependencies(isaac_awake rclcpp geometry_msgs)

target_link_libraries(vn_gps_pose vncxx )
target_link_libraries(diagnostics vncxx )
target_link_libraries(vslam_repub vncxx )
target_link_libraries(velocity_visualizer vncxx)


install(TARGETS
  vn_gps_pose
  diagnostics
  vslam_repub
  #isaac_awake
  velocity_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
