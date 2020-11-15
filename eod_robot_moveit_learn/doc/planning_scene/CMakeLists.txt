# 运行：
# roslaunch eod_robot_moveit_learn planning_scene_tutorial.launch

add_executable(planning_scene_tutorial
  src/planning_scene_tutorial.cpp
)
add_dependencies(planning_scene_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_scene_tutorial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
  modern_robotics_library 
  eod_robotics_library
)