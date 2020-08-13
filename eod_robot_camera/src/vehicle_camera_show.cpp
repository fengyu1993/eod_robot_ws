#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::Mat image_color;

void vc_color_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    image_color = cv_ptr->image;

    cv::imshow("image_color", image_color);

    cv::waitKey (5);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_camera");

    ros::NodeHandle n_vc;

    ros::Subscriber sub_vc_color = n_vc.subscribe("/eod_robot_description/vehicle_camera_ir/vehicle_camera/color/image_raw",10, vc_color_Callback);

    ros::Rate loop_rate(50);

    cv::namedWindow("image_color");

    ros::spin();

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    cv::destroyAllWindows();

    return 0;
}