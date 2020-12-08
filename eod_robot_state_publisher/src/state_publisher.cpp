#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include "modern_robotics_lib.h"
#include "eod_robotics_lib.h"

// #define GRIPPER "gripper_finger1_joint"
#define GRIPPER "finger_joint"

int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(50);

    double rad2angle = 180.0 / M_PI;
    VectorXd angle_arm_left(6); VectorXd angle_arm_right(6);
    double gripper_finger1, gripper_finger1_limit_max, gripper_finger1_limit_min;
    Matrix4d vecihle_odom; Matrix3d R_vecihle_odom; Vector3d p_vecihle_odom;

    get_gripper_finger1_limit_max(gripper_finger1_limit_max);
    get_gripper_finger1_limit_min(gripper_finger1_limit_min);

    geometry_msgs::TransformStamped odom_trans;

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(15);
    joint_state.position.resize(15);
    joint_state.name[0] = "Arm_R1_joint";
    joint_state.name[1] = "Arm_R2_joint";
    joint_state.name[2] = "Arm_R3_joint";
    joint_state.name[3] = "Arm_R4_joint";
    joint_state.name[4] = "Arm_R5_joint";
    joint_state.name[5] = "Arm_R6_joint";
    joint_state.name[6] = "Arm_L1_joint";
    joint_state.name[7] = "Arm_L2_joint";
    joint_state.name[8] = "Arm_L3_joint";
    joint_state.name[9] = "Arm_L4_joint";
    joint_state.name[10] = "Arm_L5_joint";
    joint_state.name[11] = "Arm_L6_joint";
    joint_state.name[12] = GRIPPER;
    joint_state.name[13] = "left_back_wheel_joint";
    joint_state.name[14] = "right_back_wheel_joint";


    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok())
    {
        // right arm
        get_arm_right_joint_angle(angle_arm_right);

        if((joint_state.position[0] != angle_arm_right[0]) || (joint_state.position[1] != angle_arm_right[1]) || 
            (joint_state.position[2] != angle_arm_right[2]) || (joint_state.position[3] != angle_arm_right[3]) || 
            (joint_state.position[4] != angle_arm_right[4]) || (joint_state.position[5] != angle_arm_right[5])) 
        {
            ROS_INFO("arm right angle = [%f, %f, %f, %f, %f, %f]", 
                angle_arm_right[0] * rad2angle, angle_arm_right[1] * rad2angle, angle_arm_right[2] * rad2angle,
                angle_arm_right[3] * rad2angle, angle_arm_right[4] * rad2angle, angle_arm_right[5] * rad2angle);
        }

        joint_state.position[0] = angle_arm_right[0];
        joint_state.position[1] = angle_arm_right[1];
        joint_state.position[2] = angle_arm_right[2];
        joint_state.position[3] = angle_arm_right[3];
        joint_state.position[4] = angle_arm_right[4];
        joint_state.position[5] = angle_arm_right[5];

        // left arm 
        get_arm_left_joint_angle(angle_arm_left);

        if((joint_state.position[6] != angle_arm_left[0]) || (joint_state.position[7] != angle_arm_left[1]) || 
            (joint_state.position[8] != angle_arm_left[2]) || (joint_state.position[9] != angle_arm_left[3]) || 
            (joint_state.position[10] != angle_arm_left[4]) || (joint_state.position[11] != angle_arm_left[5])) 
        {
            ROS_INFO("arm left angle = [%f, %f, %f, %f, %f, %f]", 
                angle_arm_left[0] * rad2angle, angle_arm_left[1] * rad2angle, angle_arm_left[2] * rad2angle, 
                angle_arm_left[3] * rad2angle, angle_arm_left[4] * rad2angle, angle_arm_left[5] * rad2angle);
        }

        joint_state.position[6] = angle_arm_left[0];
        joint_state.position[7] = angle_arm_left[1];
        joint_state.position[8] = angle_arm_left[2];
        joint_state.position[9] = angle_arm_left[3];
        joint_state.position[10] = angle_arm_left[4];
        joint_state.position[11] = angle_arm_left[5];

        // gripper
        get_gripper_finger1_position(gripper_finger1);

        if(gripper_finger1 != joint_state.position[12])
            ROS_INFO("gripper close percent = %f%%", gripper_finger1 / (gripper_finger1_limit_max - gripper_finger1_limit_min) * 100.0);

        joint_state.position[12] = gripper_finger1;

        // vehicle transform
        joint_state.position[13] = 0;
        joint_state.position[14] = 0;
        
        get_vecihle_odom(vecihle_odom);
        Matrix_T2tf_transform(vecihle_odom, odom_trans);

        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);
        
        odom_trans.header.stamp = ros::Time::now();
        broadcaster.sendTransform(odom_trans);

        loop_rate.sleep();

    }
    
}
