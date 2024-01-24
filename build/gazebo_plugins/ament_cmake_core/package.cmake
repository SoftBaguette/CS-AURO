set(_AMENT_PACKAGE_NAME "gazebo_plugins")
set(gazebo_plugins_VERSION "3.7.0")
set(gazebo_plugins_MAINTAINER "Jose Luis Rivero <jrivero@osrfoundation.org>, Louise Poubel <louise@openrobotics.org>")
set(gazebo_plugins_BUILD_DEPENDS "gazebo_dev" "gazebo_msgs" "gazebo_ros" "rclcpp" "camera_info_manager" "cv_bridge" "geometry_msgs" "image_transport" "nav_msgs" "sensor_msgs" "std_msgs" "std_srvs" "tf2_geometry_msgs" "tf2_ros" "trajectory_msgs" "assessment_interfaces")
set(gazebo_plugins_BUILDTOOL_DEPENDS "ament_cmake")
set(gazebo_plugins_BUILD_EXPORT_DEPENDS "camera_info_manager" "cv_bridge" "geometry_msgs" "image_transport" "nav_msgs" "sensor_msgs" "std_msgs" "std_srvs" "tf2_geometry_msgs" "tf2_ros" "trajectory_msgs" "assessment_interfaces")
set(gazebo_plugins_BUILDTOOL_EXPORT_DEPENDS )
set(gazebo_plugins_EXEC_DEPENDS "gazebo_dev" "gazebo_msgs" "gazebo_ros" "rclcpp" "camera_info_manager" "cv_bridge" "geometry_msgs" "image_transport" "nav_msgs" "sensor_msgs" "std_msgs" "std_srvs" "tf2_geometry_msgs" "tf2_ros" "trajectory_msgs" "assessment_interfaces")
set(gazebo_plugins_TEST_DEPENDS "ament_cmake_gtest" "ament_lint_auto" "ament_lint_common" "cv_bridge")
set(gazebo_plugins_GROUP_DEPENDS )
set(gazebo_plugins_MEMBER_OF_GROUPS )
set(gazebo_plugins_DEPRECATED "")
set(gazebo_plugins_EXPORT_TAGS)
list(APPEND gazebo_plugins_EXPORT_TAGS "<build_type>ament_cmake</build_type>")