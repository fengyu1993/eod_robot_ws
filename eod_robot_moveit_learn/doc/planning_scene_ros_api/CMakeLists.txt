# 运行：
# roslaunch eod_robot_moveit_config demo.launch
# roslaunch eod_robot_moveit_learn planning_scene_ros_api.launch

add_executable(planning_scene_ros_api
  src/planning_scene_ros_api.cpp
)
add_dependencies(planning_scene_ros_api ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_scene_ros_api
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)