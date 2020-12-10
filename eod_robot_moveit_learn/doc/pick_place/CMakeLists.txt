# 运行：
# roslaunch eod_robot_moveit_config demo.launch
# roslaunch eod_robot_moveit_learn pick_place_tutorial.launch

add_executable(pick_place_tutorial
  src/pick_place_tutorial.cpp
)
add_dependencies(pick_place_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place_tutorial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${Boost_LIBRARIES}
)