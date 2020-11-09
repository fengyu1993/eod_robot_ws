


add_executable(move_group_interface_tutorial
  src/move_group_interface_tutorial.cpp
)
add_dependencies(move_group_interface_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_group_interface_tutorial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
  modern_robotics_library 
  eod_robotics_library
)