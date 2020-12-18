cmake_minimum_required(VERSION 3.0.2)
project(eod_robot_library)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy tf sensor_msgs std_msgs)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(EOD_ROBOT_LIB_SOURCES 
    src/robotics/modern_robotics_lib.cpp
    src/robotics/eod_robotics_lib.cpp
    src/exception/eod_robot_exception.cpp)

add_library(eod_robot_library ${EOD_ROBOT_LIB_SOURCES})

target_link_libraries(eod_robot_library
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
)

catkin_package(CATKIN_DEPENDS roscpp rospy tf sensor_msgs std_msgs
               DEPENDS EIGEN3
               INCLUDE_DIRS include
               LIBRARIES eod_robot_library)

install(TARGETS eod_robot_library
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


add_executable(robotics_test
  src/robotics/test.cpp 
  src/robotics/modern_robotics_lib.cpp
  src/robotics/eod_robotics_lib.cpp)
add_dependencies(robotics_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robotics_test ${catkin_LIBRARIES} ${EIGEN3_LIBS})

add_executable(exception_test
  src/exception/test.cpp 
  src/exception/eod_robot_exception.cpp)
add_dependencies(exception_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exception_test ${catkin_LIBRARIES})
