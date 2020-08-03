#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>

int main(int argc, char **argv)
{
	
  
  ros::init(argc, argv, "gazebo_mover");

  ros::NodeHandle n_R1, n_R2, n_R3, n_R4, n_R5, n_R6, 
                  n_L1, n_L2, n_L3, n_L4, n_L5, n_L6,
                  n_gripper;
                                                                  
  ros::Publisher joint_R1_pub = n_R1.advertise<std_msgs::Float64>("/eod_robot_description/Arm_R1_joint_position_controller/command", 10);
  ros::Publisher joint_R2_pub = n_R2.advertise<std_msgs::Float64>("/eod_robot_description/Arm_R2_joint_position_controller/command", 10);
  ros::Publisher joint_R3_pub = n_R3.advertise<std_msgs::Float64>("/eod_robot_description/Arm_R3_joint_position_controller/command", 10);
  ros::Publisher joint_R4_pub = n_R4.advertise<std_msgs::Float64>("/eod_robot_description/Arm_R4_joint_position_controller/command", 10);
  ros::Publisher joint_R5_pub = n_R5.advertise<std_msgs::Float64>("/eod_robot_description/Arm_R5_joint_position_controller/command", 10);
  ros::Publisher joint_R6_pub = n_R6.advertise<std_msgs::Float64>("/eod_robot_description/Arm_R6_joint_position_controller/command", 10);
  ros::Publisher joint_L1_pub = n_L1.advertise<std_msgs::Float64>("/eod_robot_description/Arm_L1_joint_position_controller/command", 10);
  ros::Publisher joint_L2_pub = n_L2.advertise<std_msgs::Float64>("/eod_robot_description/Arm_L2_joint_position_controller/command", 10);
  ros::Publisher joint_L3_pub = n_L3.advertise<std_msgs::Float64>("/eod_robot_description/Arm_L3_joint_position_controller/command", 10);
  ros::Publisher joint_L4_pub = n_L4.advertise<std_msgs::Float64>("/eod_robot_description/Arm_L4_joint_position_controller/command", 10);
  ros::Publisher joint_L5_pub = n_L5.advertise<std_msgs::Float64>("/eod_robot_description/Arm_L5_joint_position_controller/command", 10);
  ros::Publisher joint_L6_pub = n_L6.advertise<std_msgs::Float64>("/eod_robot_description/Arm_L6_joint_position_controller/command", 10);
  ros::Publisher joint_gripper_pub = n_gripper.advertise<trajectory_msgs::JointTrajectory>("/eod_robot_description/Gripper_finger1_joint_position_controller/command", 10);

  ros::Rate loop_rate(50);
  
  std_msgs::Float64 joint_R1_angle, joint_R2_angle, joint_R3_angle, joint_R4_angle, joint_R5_angle, joint_R6_angle,
                    joint_L1_angle, joint_L2_angle, joint_L3_angle, joint_L4_angle, joint_L5_angle, joint_L6_angle;
  
  trajectory_msgs::JointTrajectory joint_traj_gripper;

  joint_traj_gripper.header.frame_id = "base_link";
  joint_traj_gripper.joint_names.resize(1);
  joint_traj_gripper.points.resize(1);
  joint_traj_gripper.joint_names[0] = "gripper_finger1_joint";

  while (ros::ok())	
  {  
    joint_R1_angle.data = 1.5708;
    joint_R2_angle.data = -0.4363;
    joint_R3_angle.data = 0.0000;
    joint_R4_angle.data = -1.5708;
    joint_R5_angle.data = 1.5708;
    joint_R6_angle.data = -1.5708;

    joint_L1_angle.data = -1.5708;
    joint_L2_angle.data = -2.7053;
    joint_L3_angle.data = 0.0000;
    joint_L4_angle.data = -1.5708;
    joint_L5_angle.data = -1.5708;
    joint_L6_angle.data = -1.5708;

    trajectory_msgs::JointTrajectoryPoint points_n;

    points_n.positions.push_back(0.00);
    joint_traj_gripper.points[0] = points_n;
    joint_traj_gripper.points[0].time_from_start = ros::Duration(1.0);
    joint_traj_gripper.header.stamp = ros::Time::now();

    joint_R1_pub.publish(joint_R1_angle);
    joint_R2_pub.publish(joint_R2_angle);
    joint_R3_pub.publish(joint_R3_angle);
    joint_R4_pub.publish(joint_R4_angle);
    joint_R5_pub.publish(joint_R5_angle);
    joint_R6_pub.publish(joint_R6_angle);

    joint_L1_pub.publish(joint_L1_angle);
    joint_L2_pub.publish(joint_L2_angle);
    joint_L3_pub.publish(joint_L3_angle);
    joint_L4_pub.publish(joint_L4_angle);
    joint_L5_pub.publish(joint_L5_angle);
    joint_L6_pub.publish(joint_L6_angle);

    joint_gripper_pub.publish(joint_traj_gripper);
    
    loop_rate.sleep();
 
  }

  return 0;
}

// rostopic  pub /eod_robot_description/Gripper_finger1_joint_position_controller/command trajectory_msgs/JointTrajectory '{joint_names:["gripper_finger1_joint"],points: [{positions:[0.65], time_from_start: [1.0,0.0]}]}' 

// rostopic  echo /eod_robot_description/Gripper_finger1_joint_position_controller/command 
