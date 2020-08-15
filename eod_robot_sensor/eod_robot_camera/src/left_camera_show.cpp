#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "modern_robotics_lib.h"
#include "eod_robotics_lib.h"

// Eigen::VectorXd angle(6,1);

// void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg)
// {
//   for(int i = 0; i < msg->position.size(); i++){
//     if(msg->name[i] == "Arm_L1_joint")
//       angle(0) = msg->position[i];
//     else if(msg->name[i] == "Arm_L2_joint")
//       angle(1) = msg->position[i];
//     else if(msg->name[i] == "Arm_L3_joint")
//       angle(2) = msg->position[i];
//     else if(msg->name[i] == "Arm_L4_joint")
//       angle(3) = msg->position[i];
//     else if(msg->name[i] == "Arm_L5_joint")
//       angle(4) = msg->position[i];
//     else if(msg->name[i] == "Arm_L6_joint")
//       angle(5) = msg->position[i];
//     else
//       continue;
//   }
// }

void lc_color_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat image_color = cv_ptr->image;

    cv::imshow("left_camera_image_color", image_color);

    cv::waitKey (1);

    Matrix4d T_right_effector_camera, T_left_effector_camera;

    bool flag1 = get_arm_left_T_effector_camera(T_left_effector_camera);

    bool flag2 = get_arm_right_T_effector_camera(T_right_effector_camera);

    std::cout << "flag1 " << std::endl << flag1 << std::endl;
    std::cout << "flag2 " << std::endl << flag2 << std::endl;
    std::cout << "T_left_effector_camera： " << std::endl << T_left_effector_camera << std::endl;
    std::cout << "T_right_effector_camera " << std::endl << T_right_effector_camera << std::endl;
}

void lc_depth_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    
    cv::Mat image_depth = cv_ptr->image;

    cv::imshow("left_camera_image_depth", image_depth);

    cv::waitKey (1);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "left_camera");

    ros::NodeHandle n_lc;

    // ros::Subscriber joint_pub = n_lc.subscribe("/joint_states", 10, joint_states_Callback);

    image_transport::ImageTransport it(n_lc);

    image_transport::Subscriber sub_lc_color = it.subscribe("/eod_robot_description/left_camera_ir/left_camera/color/image_raw",10, lc_color_Callback);

    image_transport::Subscriber sub_lc_depth = it.subscribe("/eod_robot_description/left_camera_ir/left_camera/depth/image_raw",10, lc_depth_Callback);

    cv::namedWindow("left_camera_image_color");

    cv::namedWindow("left_camera_image_depth");

    // tf::TransformListener listener;

    // while(n_lc.ok()){
    //     tf::StampedTransform transform;
    //     try{
    //         listener.waitForTransform("/base_link", "/left_camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0));
    //         listener.lookupTransform("/base_link", "/left_camera_depth_optical_frame", ros::Time(0), transform);
    //     }
    //     catch(tf::TransformException &ex){
    //         ROS_ERROR("%s", ex.what());
    //         ros::Duration(1.0).sleep();
    //         continue;
    //     }

    //     Eigen::Translation3d tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //     double roll, pitch, yaw;
    //     tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    //     Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
    //     Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
    //     Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());
    //     Eigen::Matrix4d T_b_c = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix(); 

    //     Eigen::Matrix4d T_b_e = eod_robot_left_arm_FKinSpace(angle);
    //     Eigen::Matrix4d T_e_c = TransInv(T_b_e) * T_b_c;

    //     std::cout << "T_b_c = " << std::endl << T_b_c << std::endl;
    //     std::cout << "T_b_e = " << std::endl << T_b_e << std::endl;
    //     std::cout << "T_e_c = " << std::endl << T_e_c << std::endl;

    //     ros::spinOnce();
    // }

    ros::spin();

    cv::destroyAllWindows();

    return 0;
}
