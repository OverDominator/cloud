cmake_minimum_required(VERSION 3.0.2)
project(fire_perception)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  costmap_2d
  cv_bridge
  geometry_msgs
  move_base_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES far_marker_global_planner
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(far_marker_global_planner
  src/far_marker_global_planner.cpp
)
target_link_libraries(far_marker_global_planner ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS
    scripts/fire_color_detector.py
    scripts/fire_pointcloud_localizer.py
    scripts/fire_keepout_cloud.py
    scripts/victim_color_detector.py
    scripts/victim_pointcloud_localizer.py
    scripts/victim_mission_manager.py
    scripts/experiment_data_logger.py
    scripts/far_move_base_bridge.py
    scripts/far_command_adapter.py
    scripts/navigation_safety_guard.py
    scripts/navigation_safety_geometry.py
    scripts/navigation_preflight.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS far_marker_global_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES fire_perception_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
