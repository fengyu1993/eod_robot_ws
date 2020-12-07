# 运行：
# roslaunch eod_robot_moveit_config demo.launch  (displays删除PlanningScene加入RobotState)
# roslaunch eod_robot_moveit_learn state_display_tutorial.launch

add_executable(state_display_tutorial
  src/state_display_tutorial.cpp
)
add_dependencies(state_display_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_display_tutorial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)