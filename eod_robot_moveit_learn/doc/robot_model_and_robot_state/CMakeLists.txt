add_executable(robot_model_and_robot_state_tutorial
  src/robot_model_and_robot_state_tutorial.cpp
)
add_dependencies(robot_model_and_robot_state_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_model_and_robot_state_tutorial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
  modern_robotics_library 
  eod_robotics_library
)