#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>
#include <Eigen/Dense>
#include <modern_robotics_lib.h>
#include "eod_robotics_lib.h"


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

    double rad2angle = 180.0 / M_PI;
    std_msgs::Float64 angle_command_right_arm[6], angle_command_left_arm[6];
    VectorXd angle_arm_left(6); VectorXd angle_arm_right(6);
    double gripper_finger1 = 0, gripper_finger1_old = 0, gripper_finger1_limit_max, gripper_finger1_limit_min;

    get_gripper_finger1_limit_max(gripper_finger1_limit_max);
    get_gripper_finger1_limit_min(gripper_finger1_limit_min);
  
    trajectory_msgs::JointTrajectory joint_traj_gripper;

    joint_traj_gripper.header.frame_id = "base_link";
    joint_traj_gripper.joint_names.resize(1);
    joint_traj_gripper.points.resize(1);
    joint_traj_gripper.joint_names[0] = "gripper_finger1_joint";

    ros::Rate loop_rate(50);

    while (ros::ok())	
    {
        // right arm
        get_arm_right_joint_angle(angle_arm_right);

        if((angle_command_right_arm[0].data != angle_arm_right[0]) || (angle_command_right_arm[1].data != angle_arm_right[1]) || 
            (angle_command_right_arm[2].data != angle_arm_right[2]) || (angle_command_right_arm[3].data != angle_arm_right[3]) || 
            (angle_command_right_arm[4].data != angle_arm_right[4]) || (angle_command_right_arm[5].data != angle_arm_right[5])) 
        {
            ROS_INFO("arm right angle = [%f, %f, %f, %f, %f, %f]", 
                angle_arm_right[0] * rad2angle, angle_arm_right[1] * rad2angle, angle_arm_right[2] * rad2angle,
                angle_arm_right[3] * rad2angle, angle_arm_right[4] * rad2angle, angle_arm_right[5] * rad2angle);
        }
    
        angle_command_right_arm[0].data = angle_arm_right[0];
        angle_command_right_arm[1].data = angle_arm_right[1];
        angle_command_right_arm[2].data = angle_arm_right[2];
        angle_command_right_arm[3].data = angle_arm_right[3];
        angle_command_right_arm[4].data = angle_arm_right[4];
        angle_command_right_arm[5].data = angle_arm_right[5];

        // left arm 
        get_arm_left_joint_angle(angle_arm_left);

        if((angle_command_left_arm[0].data != angle_arm_left[0]) || (angle_command_left_arm[1].data != angle_arm_left[1]) || 
            (angle_command_left_arm[2].data != angle_arm_left[2]) || (angle_command_left_arm[3].data != angle_arm_left[3]) || 
            (angle_command_left_arm[4].data != angle_arm_left[4]) || (angle_command_left_arm[5].data != angle_arm_left[5])) 
        {
            ROS_INFO("arm left angle = [%f, %f, %f, %f, %f, %f]", 
                angle_arm_left[0] * rad2angle, angle_arm_left[1] * rad2angle, angle_arm_left[2] * rad2angle, 
                angle_arm_left[3] * rad2angle, angle_arm_left[4] * rad2angle, angle_arm_left[5] * rad2angle);
        }

        angle_command_left_arm[0].data = angle_arm_left[0];
        angle_command_left_arm[1].data = angle_arm_left[1];
        angle_command_left_arm[2].data = angle_arm_left[2];
        angle_command_left_arm[3].data = angle_arm_left[3];
        angle_command_left_arm[4].data = angle_arm_left[4];
        angle_command_left_arm[5].data = angle_arm_left[5];        

        // gripper
        get_gripper_finger1_position(gripper_finger1);

        if(gripper_finger1 != gripper_finger1_old){
            ROS_INFO("gripper close percent = %f%%", gripper_finger1 / (gripper_finger1_limit_max - gripper_finger1_limit_min) * 100.0);
            gripper_finger1_old = gripper_finger1;
        }
            
        trajectory_msgs::JointTrajectoryPoint points_n;

        points_n.positions.push_back(gripper_finger1);
        joint_traj_gripper.points[0] = points_n;
        joint_traj_gripper.points[0].time_from_start = ros::Duration(1.0);
        joint_traj_gripper.header.stamp = ros::Time::now();

        joint_R1_pub.publish(angle_command_right_arm[0]);
        joint_R2_pub.publish(angle_command_right_arm[1]);
        joint_R3_pub.publish(angle_command_right_arm[2]);
        joint_R4_pub.publish(angle_command_right_arm[3]);
        joint_R5_pub.publish(angle_command_right_arm[4]);
        joint_R6_pub.publish(angle_command_right_arm[5]);

        joint_L1_pub.publish(angle_command_left_arm[0]);
        joint_L2_pub.publish(angle_command_left_arm[1]);
        joint_L3_pub.publish(angle_command_left_arm[2]);
        joint_L4_pub.publish(angle_command_left_arm[3]);
        joint_L5_pub.publish(angle_command_left_arm[4]);
        joint_L6_pub.publish(angle_command_left_arm[5]);

        joint_gripper_pub.publish(joint_traj_gripper);

        loop_rate.sleep();
    
    }

    return 0;
}

