// http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/motion_planning_pipeline/motion_planning_pipeline_tutorial.html
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

/*******************Start*******************/
    /*construct a RobotModel*/
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

    /*construct a PlanningScene*/
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    /*setup the PlanningPipeline object*/
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
    
    const std::string PLANNING_GROUP_RIGHT_ARM = "right_arm";
    const robot_state::JointModelGroup* joint_model_group_right_arm = robot_state->getJointModelGroup(PLANNING_GROUP_RIGHT_ARM);

/*******************Visualization*******************/
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo",rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    ros::Duration(10).sleep();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

/*******************Pose Goal*******************/
    planning_interface::MotionPlanRequest req_right_arm;
    planning_interface::MotionPlanResponse res_right_arm;
    geometry_msgs::PoseStamped pose_right_arm;
    pose_right_arm.header.frame_id = "base_link";
    pose_right_arm.pose.position.x = 0.6;
    pose_right_arm.pose.position.y = 0.0498026;
    pose_right_arm.pose.position.z = 0.240332;
    pose_right_arm.pose.orientation.w = 0;    
    pose_right_arm.pose.orientation.x = 0; 
    pose_right_arm.pose.orientation.y = 0.216589; 
    pose_right_arm.pose.orientation.z = 0.976263; 

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    /*create the request as a constraint*/
    req_right_arm.group_name = PLANNING_GROUP_RIGHT_ARM;
    moveit_msgs::Constraints pose_goal_right_arm = kinematic_constraints::constructGoalConstraints
                                            ("Arm_R_end_effector", pose_right_arm, tolerance_pose, tolerance_angle);
    req_right_arm.goal_constraints.push_back(pose_goal_right_arm);

    /*all the pipeline and check whether planning was successful*/
    planning_pipeline->generatePlan(planning_scene, req_right_arm, res_right_arm);
    if(res_right_arm.error_code_.val != res_right_arm.error_code_.SUCCESS)
    {
        ROS_ERROR("Right arm conld not compute plan successfully");
        return 0;
    }

 /*Visualize the result*/
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>
                                                        ("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory_right_arm;

    ROS_INFO("Visualizing the right arm trajectory");
    moveit_msgs::MotionPlanResponse response_right_arm;
    res_right_arm.getMessage(response_right_arm);

    display_trajectory_right_arm.trajectory_start = response_right_arm.trajectory_start;
    display_trajectory_right_arm.trajectory.push_back(response_right_arm.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory_right_arm.trajectory.back(), joint_model_group_right_arm);
        
    visual_tools.trigger();
    display_publisher.publish(display_trajectory_right_arm);

    /*Display the goal state*/
    visual_tools.publishAxisLabeled(pose_right_arm.pose, "right arm goal_1");
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

}
