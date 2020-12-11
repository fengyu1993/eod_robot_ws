
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
			left_camera_show:		左臂相机显示彩色和深度图像
			right_camera_show:		右臂相机显示彩色和深度图像
			vehicle_camera_show:	车体相机显示彩色和深度图像

# 2020.09.21 eod_robot_moveit_config:
	eod_robot的moveit配置文件

# 2020.09.28 eod_robot_moveit_learn:
	eod_robot实现moveit教程: http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html
		class：
			RobotModel: http://docs.ros.org/en/melodic/api/moveit_core/html/classmoveit_1_1core_1_1RobotModel.html
			RobotState: http://docs.ros.org/en/melodic/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html
			PlanningScene: http://docs.ros.org/en/melodic/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html
			PlanningSceneMonitor： http://docs.ros.org/en/melodic/api/moveit_ros_planning/html/classplanning__scene__monitor_1_1PlanningSceneMonitor.html
			RobotModelLoader: http://docs.ros.org/en/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
			InteractiveRobot: https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/interactivity/src/interactive_robot.cpp
		package:
			kinematic_constraints: http://docs.ros.org/en/melodic/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
	
	robot_model_and_robot_state: (类 RobotModel, RobotState)
			1. 读取关节角度
			2. 测试关节限制
			3. 计算正运动学
			4. 求解逆运动学
			5. 计算雅克比矩阵 (前3行是线速度分量，后3行是角速度分量) ---- 这个存在疑问，与本人计算的结果线速度分量不符
	
	planning_scene:(类 PlanningScene)
			1. 碰撞检测：检测各关节是否碰撞
			2. 约束检测：检测关节约束、位置约束、方向约束和可视化约束
			3. 可行性检测：通过回调函数，用户自定义的约束（例如：第一个关节角度大于0）
	
	planning_scene_ros_api:
			1. 在环境中添加或移除物体
			2. 在机器人连杆上依附或去掉物体
	
	motion_planning_api:(类 RobotModelLoader 包 kinematic_constraints)
			1. 在任务空间进行规划，并在rviz中显示路径
			2. 在关节空间进行规划
	
	motion_planning_pipeline:(路径规划推荐使用这个方法)
			与motion_planning_api类似，但是这个命令多了路径规划的预处理和后处理功能(Iterative Parabolic Time Parameterization)
			1. 在任务空间进行规划，并在rviz中显示路径
			2. 在关节空间进行规划
			3. 左右臂都可以规划成功 (左臂在zero位置是碰撞状态，所以规划时要现将初始规划位置设在不碰撞的姿态)
	
	visualizing_collisions:(类 InteractiveRobot)
			1. 用到了interactivity文件中InteractiveRobot类（在开始时刻使机器臂处在work位置）
			2. 可以拖拽末端执行器的Marker,实现机械臂运动，并且可以检测碰撞
			3. 可以拖拽创建的Marker,实现与机器人的碰撞检测
	
	moveit_group_interface:
			1. 用c++语言完成moveit的相关操作
			2. 是之前几节的结合：
				robot_model_and_robot_state
				planning_scene
				planning_scene_ros_api
				motion_planning_api
				motion_planning_pipeline
				visualizing_collisions
	
	state_display:
			1. 发布机器人状态消息（moveit_msgs::DisplayRobotState）
			2. 设置左右机械臂处于随机位姿
			3. 设置左右机械臂处于预先定义的位姿（work）
			4. 读取左右机械臂末端执行器位姿
			5. 计算左右机械臂逆运动学

	pick_and_place_tutorial:
			1. 用move_group进行关节空间运动（从“front”到“work”）
			2. 用std::vector<moveit_msgs::CollisionObject>创建碰撞物体
			3. 用moveit::planning_interface::PlanningSceneInterface向rviz发布
			4. 用moveit::planning_interface::MoveGroupInterface创建分组
			5. 用std::vector<moveit_msgs::Grasp>在move_group.pick()进行抓取
			6. 用std::vector<moveit_msgs::PlaceLocation>在move_group.place()进行放置

	
















