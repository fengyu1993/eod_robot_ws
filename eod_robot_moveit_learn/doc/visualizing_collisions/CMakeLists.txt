# 运行：
# roslaunch eod_robot_moveit_learn visualizing_collisions_tutorial_right_arm.launch
# roslaunch eod_robot_moveit_learn visualizing_collisions_tutorial_left_arm.launch

add_executable(visualizing_collisions_tutorial_right_arm
  src/visualizing_collisions_tutorial_right_arm.cpp
)
add_executable(visualizing_collisions_tutorial_left_arm
  src/visualizing_collisions_tutorial_left_arm.cpp
)

add_dependencies(visualizing_collisions_tutorial_right_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(visualizing_collisions_tutorial_left_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(visualizing_collisions_tutorial_right_arm
  ${interactive_markers_LIBRARIES}
  interactivity_utils
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)
target_link_libraries(visualizing_collisions_tutorial_left_arm
  ${interactive_markers_LIBRARIES}
  interactivity_utils
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)