cmake_minimum_required(VERSION 2.8.3)

project(eod_robot_description)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation controller_manager)

catkin_package()

find_package(roslaunch)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gazebo_mover
  src/gazebo_mover.cpp
)
add_dependencies(gazebo_mover ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_mover
  ${catkin_LIBRARIES}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
