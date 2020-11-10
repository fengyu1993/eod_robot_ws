#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit_msgs/GetPositionIK.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "eod_robot_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle;

    ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    ros::Publisher robot_state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("tutorial_robot_state", 1);

    while (! service_client.exists())
    {
        ROS_INFO("Waiting for service");
        ros::Duration(1.0).sleep();
    }

    moveit_msgs::GetPositionIK::Request service_request;
    moveit_msgs::GetPositionIK::Response service_response;

    service_request.ik_request.group_name = "right_arm";
    service_request.ik_request.pose_stamped.header.frame_id = "base_link";
    service_request.ik_request.pose_stamped.pose.position.x = 0.249125;
    service_request.ik_request.pose_stamped.pose.position.y = -0.341856;
    service_request.ik_request.pose_stamped.pose.position.z = 0.100761;
    service_request.ik_request.pose_stamped.pose.orientation.w = 0.746917;
    service_request.ik_request.pose_stamped.pose.orientation.x = -0.283898;
    service_request.ik_request.pose_stamped.pose.orientation.y = 0.552079;
    service_request.ik_request.pose_stamped.pose.orientation.z = 0.238171;

    service_client.call(service_request, service_response);
    ROS_INFO_STREAM(
        "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                   << service_response.error_code.val);
    
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        /*输出逆运动学结果*/
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), service_response.solution.joint_state.position[i]);
    }


    service_request.ik_request.robot_state.joint_state.name = joint_model_group->getJointModelNames();

    kinematic_state->setToRandomPositions(joint_model_group);
    kinematic_state->copyJointGroupPositions(joint_model_group,
                                    service_request.ik_request.robot_state.joint_state.position);
    
    service_client.call(service_request, service_response);
    ROS_INFO_STREAM(
        "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                   << service_response.error_code.val);
    
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), service_response.solution.joint_state.position[i]);
    }


    service_request.ik_request.avoid_collisions = true;

    service_client.call(service_request, service_response); 

    ROS_INFO_STREAM(
      "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);

    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), service_response.solution.joint_state.position[i]);
    }

    moveit_msgs::DisplayRobotState msg;
    kinematic_state->setVariableValues(service_response.solution.joint_state);
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    robot_state_publisher.publish(msg);

    ros::Duration(2.0).sleep();

    ros::shutdown();
    return 0;
}