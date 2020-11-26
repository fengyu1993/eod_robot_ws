# 运行：
# roslaunch eod_robot_moveit_learn visualizing_collisions_tutorial.launch

add_executable(visualizing_collisions_tutorial
  src/visualizing_collisions_tutorial.cpp
)

add_dependencies(visualizing_collisions_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(visualizing_collisions_tutorial
  ${interactive_markers_LIBRARIES}
  interactivity_utils
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
  modern_robotics_library 
  eod_robotics_library
)