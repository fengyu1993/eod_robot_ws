#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "eod_robot_msgs/Eod_robot_twist.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <Eigen/Dense>
#include <modern_robotics_lib.h>
#include "eod_robotics_lib.h"

geometry_msgs::Twist arm_right_twist;
std_msgs::Bool arm_right_joint_or_task;

void arm_right_vel_Callback(const eod_robot_msgs::Eod_robot_twist::ConstPtr& eod_robot_twist)
{
    arm_right_twist.linear.x = eod_robot_twist->arm_right_linear.x ;
    arm_right_twist.linear.y = eod_robot_twist->arm_right_linear.y ;
    arm_right_twist.linear.z = eod_robot_twist->arm_right_linear.z ;
    arm_right_twist.angular.x = eod_robot_twist->arm_right_angular.x;
    arm_right_twist.angular.y = eod_robot_twist->arm_right_angular.y;
    arm_right_twist.angular.z = eod_robot_twist->arm_right_angular.z;

    arm_right_joint_or_task.data = eod_robot_twist->arm_right_joint_or_task.data;
}

double sum_twist(geometry_msgs::Twist V){
    return V.linear.x + V.linear.y + V.linear.z + V.angular.x + V.angular.y + V.angular.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control arn right");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_eod_robot_vel", 100, arm_right_vel_Callback);

    double dt;
    int arm_right_pose_num = 0, arm_right_pose_num_old = 0;
    std::string pose_string[4] = {"angle_arm_right_work", "angle_arm_right_front", "angle_arm_right_behind", "angle_arm_right_stand"};
    VectorXd angle_arm_right(6); VectorXd angle_velocity_arm_right(6);
    MatrixXd Slist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm;
    XmlRpc::XmlRpcValue param_yaml;

    n.getParam("dt", dt);
    get_arm_right_joint_angle(angle_arm_right);
    get_arm_right_param(Slist_arm_right, M_arm_right, T_base_right_arm);

    ros::Rate loop_rate(1 / dt);

    while (ros::ok())
    {
        try{
            n.getParam("arm_right_pose_num", arm_right_pose_num);
            if(arm_right_pose_num != arm_right_pose_num_old){
                n.getParam(pose_string[arm_right_pose_num - 1], param_yaml);
                for(int i =0; i < param_yaml.size(); i++)
                    angle_arm_right[i] = param_yaml[i];
                arm_right_pose_num_old = arm_right_pose_num;
            }
            else if(arm_right_joint_or_task.data == false){ // task space
                if(!NearZear(sum_twist(arm_right_twist))){
                    Matrix4d T_effect = eod_robot_FKinSpace(M_arm_right, Slist_arm_right, angle_arm_right, T_base_right_arm);
                    T_effect.block<3,3>(0,0) = Matrix3d::Identity();
                    MatrixXd Jacobian_arm_right = Adjoint(TransInv(T_effect)) * eod_robot_JacobianSpace(Slist_arm_right, angle_arm_right, T_base_right_arm);
                
                    VectorXd twist_arm_right(6);
                    twist_arm_right[0] = arm_right_twist.angular.x;
                    twist_arm_right[1] = arm_right_twist.angular.y;
                    twist_arm_right[2] = arm_right_twist.angular.z;
                    twist_arm_right[3] = arm_right_twist.linear.x;
                    twist_arm_right[4] = arm_right_twist.linear.y;
                    twist_arm_right[5] = arm_right_twist.linear.z;

                    angle_velocity_arm_right = Jacobian_arm_right.jacobiSvd(ComputeThinU | ComputeThinV).solve(twist_arm_right);
                    angle_arm_right += angle_velocity_arm_right * dt;
                }
            }
            else{ // joint space
                if(!NearZear(sum_twist(arm_right_twist))){
                    angle_velocity_arm_right[0] = arm_right_twist.linear.x;
                    angle_velocity_arm_right[1] = arm_right_twist.linear.y;
                    angle_velocity_arm_right[2] = arm_right_twist.linear.z;
                    angle_velocity_arm_right[3] = arm_right_twist.angular.x;
                    angle_velocity_arm_right[4] = arm_right_twist.angular.y;
                    angle_velocity_arm_right[5] = arm_right_twist.angular.z;

                    angle_arm_right += angle_velocity_arm_right * dt;
                }
            }

            set_arm_right_joint_angle(angle_arm_right);

            ros::spinOnce();
            loop_rate.sleep();
        }
        catch(...) {
            continue;
        }

    }

    return 0;
}

