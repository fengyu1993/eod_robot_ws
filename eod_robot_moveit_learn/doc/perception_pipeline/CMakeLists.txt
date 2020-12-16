# 运行：
# roslaunch eod_robot_moveit_learn detect_and_add_cylinder_collision_object_demo.launch

add_executable(cylinder_segment
  src/cylinder_segment.cpp
)
add_dependencies(cylinder_segment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cylinder_segment
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)

add_executable(bag_publisher_maintain_time
  src/bag_publisher_maintain_time.cpp
)
add_dependencies(bag_publisher_maintain_time ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bag_publisher_maintain_time
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)


