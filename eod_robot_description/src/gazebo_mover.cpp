#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>

int main(int argc, char **argv)
{
	
  
  ros::init(argc, argv, "gazebo_mover");

  ros::NodeHandle n_R1, n_R2, n_R3, n_R4, n_R5, n_R6, n_L1, n_L2, n_L3, n_L4, n_L5, n_L6;
  
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

  ros::Rate loop_rate(50);
  
  int start_time, elapsed;
  
  while(not start_time)
  {
  	start_time = ros::Time::now().toSec();
  }
  
  
  while (ros::ok())	
  {
    
    elapsed = ros::Time::now().toSec() - start_time;
    
    std_msgs::Float64 joint_R1_angle, joint_R2_angle, joint_R3_angle, joint_R4_angle, joint_R5_angle, joint_R6_angle,
                      joint_L1_angle, joint_L2_angle, joint_L3_angle, joint_L4_angle, joint_L5_angle, joint_L6_angle;

    joint_R1_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_R2_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_R3_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_R4_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_R5_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_R6_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_L1_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_L2_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_L3_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_L4_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_L5_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);
    joint_L6_angle.data=sin(2*M_PI*0.1*elapsed)*(M_PI/2);

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
	
	loop_rate.sleep();
 
  }

  return 0;
}
