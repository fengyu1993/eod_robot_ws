#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

void vc_color_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat image_color = cv_ptr->image;

    cv::imshow("vehicle_camera_image_color", image_color);

    cv::waitKey (1);
}

void vc_depth_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    
    cv::Mat image_depth = cv_ptr->image;

    cv::imshow("vehicle_camera_image_depth", image_depth);

    cv::waitKey (1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_camera");

    ros::NodeHandle n_vc;

    image_transport::ImageTransport it(n_vc);

    image_transport::Subscriber sub_vc_color = it.subscribe("/eod_robot_description/vehicle_camera_ir/vehicle_camera/color/image_raw",10, vc_color_Callback);

    image_transport::Subscriber sub_vc_depth = it.subscribe("/eod_robot_description/vehicle_camera_ir/vehicle_camera/depth/image_raw",10, vc_depth_Callback);

    cv::namedWindow("vehicle_camera_image_color");

    cv::namedWindow("vehicle_camera_image_depth");

    ros::spin();

    cv::destroyAllWindows();

    return 0;
}
