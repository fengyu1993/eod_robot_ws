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

geometry_msgs::Twist vecihle_vel;


void vecihle_vel_Callback(const eod_robot_msgs::Eod_robot_twist::ConstPtr& eod_robot_twist)
{
    vecihle_vel.linear.x = eod_robot_twist->vecihle_linear.x ;
    vecihle_vel.linear.y = eod_robot_twist->vecihle_linear.y ;
    vecihle_vel.linear.z = eod_robot_twist->vecihle_linear.z ;
    vecihle_vel.angular.x = eod_robot_twist->vecihle_angular.x;
    vecihle_vel.angular.y = eod_robot_twist->vecihle_angular.y;
    vecihle_vel.angular.z = eod_robot_twist->vecihle_angular.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control vecihle");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_eod_robot_vel", 100, vecihle_vel_Callback);

    Matrix4d vecihle_odom; Matrix3d R_vecihle_odom; Vector3d p_vecihle_odom;
    double dt, vecihle_z;

    n.getParam("dt", dt);
    n.getParam("vecihle_z", vecihle_z);
    get_vecihle_odom(vecihle_odom);

    TransToRp(vecihle_odom, R_vecihle_odom, p_vecihle_odom);

    double angle_vehicle = 0.0; 

    ros::Rate loop_rate(1 / dt);

    geometry_msgs::TransformStamped odom_trans;
        
    while (ros::ok())
    {
        p_vecihle_odom[0] += vecihle_vel.linear.x * dt * cos(angle_vehicle);
        p_vecihle_odom[1] += vecihle_vel.linear.x * dt * sin(angle_vehicle);

        angle_vehicle += vecihle_vel.angular.z * dt;
        Eigen::AngleAxisd yawAngle(angle_vehicle, Eigen::Vector3d::UnitZ());
        R_vecihle_odom = yawAngle.matrix();

        RpToTrans(R_vecihle_odom, p_vecihle_odom, vecihle_odom);

        set_vecihle_odom(vecihle_odom);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

