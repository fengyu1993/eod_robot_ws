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

Eigen::VectorXd angle(6,1);

void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int i = 0; i < msg->position.size(); i++){
    if(msg->name[i] == "Arm_R1_joint")
      angle(0) = msg->position[i];
    else if(msg->name[i] == "Arm_R2_joint")
      angle(1) = msg->position[i];
    else if(msg->name[i] == "Arm_R3_joint")
      angle(2) = msg->position[i];
    else if(msg->name[i] == "Arm_R4_joint")
      angle(3) = msg->position[i];
    else if(msg->name[i] == "Arm_R5_joint")
      angle(4) = msg->position[i];
    else if(msg->name[i] == "Arm_R6_joint")
      angle(5) = msg->position[i];
    else
      continue;
  }
}

void rc_color_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat image_color = cv_ptr->image;

    cv::imshow("right_camera_image_color", image_color);

    cv::waitKey (1);
}

void rc_depth_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    
    cv::Mat image_depth = cv_ptr->image;

    cv::imshow("right_camera_image_depth", image_depth);

    cv::waitKey (1);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "right_camera");

    ros::NodeHandle n_rc;

    ros::Subscriber joint_pub = n_rc.subscribe("/joint_states", 10, joint_states_Callback);

    image_transport::ImageTransport it(n_rc);

    image_transport::Subscriber sub_rc_color = it.subscribe("/eod_robot_description/right_camera_ir/right_camera/color/image_raw",10, rc_color_Callback);

    image_transport::Subscriber sub_rc_depth = it.subscribe("/eod_robot_description/right_camera_ir/right_camera/depth/image_raw",10, rc_depth_Callback);

    cv::namedWindow("right_camera_image_color");

    cv::namedWindow("right_camera_image_depth");

    tf::TransformListener listener;

    while(n_rc.ok()){
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("/base_link", "/right_camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/right_camera_depth_optical_frame", ros::Time(0), transform);
        }
        catch(tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        Eigen::Translation3d tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
        Eigen::AngleAxisd rot_x_btol(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y_btol(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z_btol(yaw, Eigen::Vector3d::UnitZ());
        Eigen::Matrix4d T_b_c = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix(); 
        
        Eigen::Matrix4d T_b_e = eod_robot_right_arm_FKinSpace(angle);
        Eigen::Matrix4d T_e_c = TransInv(T_b_e) * T_b_c;

        // std::cout << "T_b_c = " << std::endl << T_b_c << std::endl;
        std::cout << "T_b_e = " << std::endl << T_b_e << std::endl;
        // std::cout << "T_e_c = " << std::endl << T_e_c << std::endl;


        ros::spinOnce();
    }

    // ros::spin();

    cv::destroyAllWindows();

    return 0;
}
