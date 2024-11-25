cmake_minimum_required(VERSION 3.8)
project(pro_arm_moveit)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

# Install Python scripts with execute permissions
install(
  PROGRAMS
    nodes/emergency_stop_marker.py
    scripts/trigger_emergency_stop.py
  DESTINATION 
    lib/${PROJECT_NAME}
)

# Install configuration directories
install(
  DIRECTORY 
    config 
    launch 
    rviz 
    srdf
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()