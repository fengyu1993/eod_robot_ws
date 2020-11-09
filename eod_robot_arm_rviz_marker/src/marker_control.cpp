#include <iostream>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include <string>
#include <Eigen/Dense>
#include "modern_robotics_lib.h"
#include "eod_robotics_lib.h"

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

Marker makeBox(InteractiveMarker &msg)
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.5;
    marker.scale.y = msg.scale * 0.5;
    marker.scale.z = msg.scale * 0.5;

    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}


void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  Matrix4d T; Matrix3d R; Vector3d p;

  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
        ROS_INFO_STREAM( s.str() << ": pose changed"
            << "\nposition = "
            << feedback->pose.position.x
            << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z
            << "\norientation = "
            << feedback->pose.orientation.w
            << ", " << feedback->pose.orientation.x
            << ", " << feedback->pose.orientation.y
            << ", " << feedback->pose.orientation.z
            << "\nframe: " << feedback->header.frame_id
            << " time: " << feedback->header.stamp.sec << "sec, "
            << feedback->header.stamp.nsec << " nsec" );

        MatrixXd Slist(6,6); Matrix4d M; Matrix4d T_base_arm;
        VectorXd angle_arm_left(6); VectorXd angle_arm_right(6); VectorXd thetalist0(6); 
        bool suc;

        p << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z; 
        R = Quaternion2RotationMatrix(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);
        RpToTrans(R, p, T); 

        if (feedback->marker_name == "arm left marker"){

            get_arm_left_joint_angle(thetalist0); 

            // suc = eod_robot_left_arm_IKinSpace_NR(T, thetalist0, 0.01, 0.001, angle_arm_left);
            suc = eod_robot_left_arm_IKinSpace_POE(T, thetalist0, 1, angle_arm_left);

            if(suc){
                set_arm_left_joint_angle(angle_arm_left);
                ROS_INFO("arm left IKinSpace success");
            }
            else{
                ROS_INFO("arm left IKinSpace fail");
            }         

        }
        else if (feedback->marker_name == "arm right marker"){

            get_arm_right_joint_angle(thetalist0);

            // suc = eod_robot_right_arm_IKinSpace_NR(T, thetalist0, 0.01, 0.001, angle_arm_right);  // 数值解
            suc = eod_robot_right_arm_IKinSpace_POE(T, thetalist0, 1, angle_arm_right); // 解析解

            if(suc){
                set_arm_right_joint_angle(angle_arm_right);
                std::cout << "T: " << T << std::endl;
                ROS_INFO("arm right IKinSpace success");
            }
            else{
                ROS_INFO("arm right IKinSpace fail");
            }

        }

        break;
    }

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );      
      break;
  }

  server->applyChanges();
}

void make6DofMarker(bool fixed, unsigned int interaction_mode, const Matrix4d T, std::string name, bool show_6dof)
{
    Matrix3d R; Vector3d p;

    TransToRp(T, R, p);
    Eigen::Quaterniond quater = rotationMatrix2Quaterniond(R);

    InteractiveMarker int_marker;  tf::Vector3 position;
    int_marker.header.frame_id = "base_link";
    position = tf::Vector3(p(0), p(1), p(2));

    int_marker.pose.position.x = p(0); int_marker.pose.position.y = p(1); int_marker.pose.position.z = p(2);
    int_marker.pose.orientation.x = quater.x();
    int_marker.pose.orientation.y = quater.y();
    int_marker.pose.orientation.z = quater.z();
    int_marker.pose.orientation.w = quater.w();
    int_marker.scale = 0.15;

    int_marker.name = name.c_str();
    int_marker.description = "Simple 6-DOF Control";

    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if( fixed )
    {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if(interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)           mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_" + mode_text;
        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof)
    {
        tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    if(interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply(*server, int_marker.name);
}





int main(int argc, char** argv)
{    
    ros::init(argc, argv, "marker_controls");
    ros::NodeHandle n;

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));

    ros::Duration(0.1).sleep();

    XmlRpc::XmlRpcValue param_yaml;

    VectorXd angle_arm_left(6);     
    VectorXd angle_arm_right(6); 

    get_arm_left_joint_angle(angle_arm_left);    
    get_arm_right_joint_angle(angle_arm_right);

    Matrix4d T_arm_left_effect = eod_robot_left_arm_FKinSpace(angle_arm_left);
    Matrix4d T_arm_right_effect = eod_robot_right_arm_FKinSpace(angle_arm_right);

    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::NONE, T_arm_left_effect, "arm left marker", true);
    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::NONE, T_arm_right_effect, "arm right marker", true);

    server->applyChanges();

    ros::spin();

    server.reset();
}