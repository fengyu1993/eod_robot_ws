# 运行：
# roslaunch eod_robot_moveit_learn motion_planning_pipeline_tutorial.launch 

add_executable(motion_planning_pipeline_tutorial
  src/motion_planning_pipeline_tutorial.cpp
)
add_dependencies(motion_planning_pipeline_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_planning_pipeline_tutorial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
  modern_robotics_library 
  eod_robotics_library
)