#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "eod_robot_msgs/Eod_robot_twist.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <modern_robotics_lib.h>
#include "eod_robotics_lib.h"

double linear, angular;


void vecihle_vel_Callback(const eod_robot_msgs::Eod_robot_twist::ConstPtr& eod_robot_twist)
{
    linear = eod_robot_twist->vecihle_linear.x ;
    angular = eod_robot_twist->vecihle_angular.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control vecihle");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_eod_robot_vel", 10, vecihle_vel_Callback);

    double dt;

    n.getParam("dt", dt);

    ros::Rate loop_rate(1 / dt);
        
    while (ros::ok())
    {
        set_vecihle_velocity(linear, angular);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

