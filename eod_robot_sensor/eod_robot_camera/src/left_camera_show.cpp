#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

void lc_color_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat image_color = cv_ptr->image;

    cv::imshow("left_camera_image_color", image_color);

    cv::waitKey (1);
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

    //     Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //     double roll, pitch, yaw;
    //     tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    //     Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    //     Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    //     Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    //     Eigen::Matrix4f tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix(); 
    //     std::cout << "T_b_l = " << tf_btol << std::endl;
    //     ros::spinOnce();
    // }

    ros::spin();

    cv::destroyAllWindows();

    return 0;
}
