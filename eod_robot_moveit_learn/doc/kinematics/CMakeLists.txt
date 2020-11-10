# 运行：
# roslaunch eod_robot_moveit_learn ros_api_tutorial.launch
# rosrun eod_robot_moveit_learn ros_api_tutorial

add_executable(ros_api_tutorial
  src/ros_api_tutorial.cpp
)
add_dependencies(ros_api_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_api_tutorial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
  modern_robotics_library 
  eod_robotics_library
)