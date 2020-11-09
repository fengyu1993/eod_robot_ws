cmake_minimum_required(VERSION 3.0.2)
project(eod_robot_moveit_learn)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED 
COMPONENTS 
    roscpp 
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    eod_robot_library)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${EIGEN3_INCLUDE_DIR}
  include ${Boost_INCLUDE_DIR} 
  ../eod_robot_library/src/modern_robotics
  ../eod_robot_library/src/eod_robotics
)

add_subdirectory(doc/moveit_group_interface)
add_subdirectory(doc/robot_model_and_robot_state)
