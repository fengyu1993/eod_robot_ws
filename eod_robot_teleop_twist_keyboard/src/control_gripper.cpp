#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "eod_robot_msgs/Eod_robot_twist.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <modern_robotics_lib.h>
#include "eod_robotics_lib.h"

std_msgs::Float64 gripper_vel;

void gripper_vel_Callback(const eod_robot_msgs::Eod_robot_twist::ConstPtr& eod_robot_twist)
{
    gripper_vel.data = eod_robot_twist->gripper_velocity.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control gripper");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_eod_robot_vel", 100, gripper_vel_Callback);

    double dt, gripper_finger1;

    n.getParam("dt", dt); 
    get_gripper_finger1_position(gripper_finger1);

    ros::Rate loop_rate(1 / dt);

    while (ros::ok())
    {      
        gripper_finger1 += gripper_vel.data * dt;

        if (gripper_finger1 < 0.0)
            gripper_finger1 = 0.0;
        else if(gripper_finger1 > 1.0)
            gripper_finger1 = 1.0;

        set_gripper_finger1_position(gripper_finger1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

