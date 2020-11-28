# 运行：
# roslaunch eod_robot_moveit_config demo.launch
# roslaunch eod_robot_moveit_learn motion_planning_api_tutorial.launch

add_executable(motion_planning_api_tutorial
  src/motion_planning_api_tutorial.cpp
)
add_dependencies(motion_planning_api_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_planning_api_tutorial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)