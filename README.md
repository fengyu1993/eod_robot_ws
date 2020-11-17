
作者：陈禹含 ... ... 
邮箱：3120185244@bit.edu.cn

先运行(安装依赖项)
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y

# 2020.07.13 eod_robot_description: 
	机器人的URDF模型和eod_robot_param.yaml参数文件
	yaml参数文件存放着eod_robot的基本参数，其他需要控制机器人运动的包都需加载这个文件
	eod_robot的参数的名字是固定的，不允许改动
	可在rviz中显示并动态调节各关节角度
	gezabo仿真也已验证,车体加装了三个轮子（两个差速、一个万向）可以通过插件控制，机械爪也没有问题
	加装了3个d435i相机:车体1个，左右臂各1个，车体的d435i有imu信息


# 2020.07.13 eod_robot_state_publisher:
	state_publisher:
		可以通过程序是机器人在rviz中运动
		只有在rviz中显示，就要运行这个包
		用到tf, JointState工具和eod_robot_library库
		读取eod_robot参数，并发布到rviz中
	state_publisher_gazebo:
		用法与上相似，但这是应用到gazebo仿真环境的
	
	

# 2020.07.19 eod_robot_library:
	利用eigen函数库撰写的modern robotics相关程序(POE)
	在modern robotics库基础上撰写eod_robotics_lib，适用于开发的排爆机器人
	modern_robotics_lib:
		POE机器人学
		机器人逆运动学解析解法
	eod_robotics_lib:
		eod_robot机器人学：正运动学，逆运动学数值与解析解法
		读取和设置eod_robot参数
		旋转矩阵、rpy欧拉角、四元数转换


# 2020.07.26 eod_robot_teleop_twist_keyboard:
	通过按键控制机器人运动
	具体包括车体运动、夹爪开合，左右机械臂在关节空间和任务空间运动
	用到了机器人学的知识，用到eod_robot_library库
	撰写了msg和yaml文件
	可在rviz和gazebo仿真


# 2020.07.30 eod_robot_arm_rviz_marker:
	通过拖动rviz中的marker改变两机械臂末端执行器的位置
	用到逆运动学


# 2020.07.31 eod_robot_dependlib:
	存放着eod_robot需要的依赖文件:
		roboticsgroup_gazebo_plugins: 机械爪gazebo仿真需要的插件
		realsense-ros: d435i摄像头需要的插件


# 2020.08.01 eod_robot_moveit_config：
	eod_robot的moveit配置文件


# 2020.08.13 eod_robot_sensor:
	这机器人传感器用到的文件夹，包含下package:
		eod_robot_camera:
			left_camera_show:	左臂相机显示彩色和深度图像
			right_camera_show:	右臂相机显示彩色和深度图像
			vehicle_camera_show:	车体相机显示彩色和深度图像

# 2020.09.21 eod_robot_moveit_config:
	eod_robot的moveit配置文件

# 2020.09.28 eod_robot_moveit_learn:
	eod_robot实现moveit教程: http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html
	robot_model_and_robot_state: 
			1. 读取关节角度
			2. 测试关节限制
			3. 计算正运动学
			4. 求解逆运动学
			5. 计算雅克比矩阵 (前3行是线速度分量，后3行是角速度分量) ---- 这个存在疑问，与本人计算的结果线速度分量不符
	planning_scene_tutorial:
			1. 碰撞检测：检测各关节是否碰撞
			2. 约束检测：检测关节约束、位置约束、方向约束和可视化约束
			3. 可行性检测：通过回调函数，用户自定义的约束（例如：第一个关节角度大于0）
	planning_scene_ros_api:
			1. 在环境中添加或移除物体
			2. 在机器人连杆上依附或去掉物体
	
















