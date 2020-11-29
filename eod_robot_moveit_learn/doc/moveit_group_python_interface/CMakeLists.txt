# 运行：
# roslaunch eod_robot_moveit_config demo.launch
# rosrun eod_robot_moveit_learn move_group_python_interface_tutorial.py
catkin_install_python(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)