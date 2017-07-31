cmake_minimum_required(VERSION 2.8.3)
project(racecar_grand_prix)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    scripts/ar_localization.py
    scripts/start_tracker.py
    scripts/vision_tracker.py
    scripts/wall_tracker.py
    scripts/bridge_tracker.py
    scripts/prix_controller.py
    DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION))
