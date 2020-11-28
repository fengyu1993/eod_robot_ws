# 运行：
# roslaunch eod_robot_moveit_learn motion_planning_pipeline_tutorial_right_arm.launch 
# roslaunch eod_robot_moveit_learn motion_planning_pipeline_tutorial_left_arm.launch 

add_executable(motion_planning_pipeline_tutorial_right_arm
  src/motion_planning_pipeline_tutorial_right_arm.cpp
)
add_dependencies(motion_planning_pipeline_tutorial_right_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_planning_pipeline_tutorial_right_arm
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)

add_executable(motion_planning_pipeline_tutorial_left_arm
  src/motion_planning_pipeline_tutorial_left_arm.cpp
)
add_dependencies(motion_planning_pipeline_tutorial_left_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_planning_pipeline_tutorial_left_arm
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)