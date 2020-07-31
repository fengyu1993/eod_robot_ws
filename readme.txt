作者：陈禹含
邮箱：3120185244@bit.edu.cn

2020.07.13
eod_robot_description: 
	机器人的URDF模型和eod_robot_param.yaml参数文件
	yaml参数文件存放着eod_robot的基本参数，其他需要控制机器人运动的包都需加载这个文件
	eod_robot的参数的名字是固定的，不允许改动
	可在rviz中显示并动态调节各关节角度
	gezabo仿真还未验证

2020.07.13
eod_robot_state_publisher:
	可以通过程序是机器人在rviz中运动
	只有在rviz中显示，就要运行这个包
	用到tf, JointState工具和eod_robot_library库
	读取eod_robot参数，并发布到rviz中
	
	

2020.07.19
eod_robot_library:
	利用eigen函数库撰写的modern robotics相关程序(POE)
	在modern robotics库基础上撰写eod_robotics_lib，适用于开发的排爆机器人
	modern_robotics_lib:
		POE机器人学
	eod_robotics_lib:
		eod_robot机器人学
		读取和设置eod_robot参数
		旋转矩阵、rpy欧拉角、四元数转换


2020.07.26
eod_robot_teleop_twist_keyboard:
	通过按键控制机器人运动
	具体包括车体运动、夹爪开合，左右机械臂在关节空间和任务空间运动
	用到了机器人学的知识，用到eod_robot_library库
	撰写了msg和yaml文件

2020.07.30
eod_robot_arm_rviz_marker:
	通过拖动rviz中的marker改变两机械臂末端执行器的位置
	用到逆运动学


