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
    
    const std::string PLANNING_GROUP_LEFT_ARM = "left_arm";
    const robot_state::JointModelGroup* joint_model_group_left_arm = robot_state->getJointModelGroup(PLANNING_GROUP_LEFT_ARM);

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

    std::vector<double> joint_values_left_arm = { 0.0, -2.7053, -0.7854, -1.5708, 2.7053, 0}; //work
    robot_state->setJointGroupPositions(joint_model_group_left_arm, joint_values_left_arm);
    planning_scene->setCurrentState(*robot_state); //将机器人的姿态设置成work姿态，因为zero状态会碰撞

/*******************Pose Goal*******************/
    planning_interface::MotionPlanRequest req_left_arm;
    planning_interface::MotionPlanResponse res_left_arm;
    geometry_msgs::PoseStamped pose_left_arm;
    pose_left_arm.header.frame_id = "base_link";
    pose_left_arm.pose.position.x = 0.45;
    pose_left_arm.pose.position.y = 0.152677;
    pose_left_arm.pose.position.z = -0.169115;
    pose_left_arm.pose.orientation.w = -0.0517946;    
    pose_left_arm.pose.orientation.x = -0.58507; 
    pose_left_arm.pose.orientation.y = 0.807652; 
    pose_left_arm.pose.orientation.z = 0.0520327; 

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    /*create the request as a constraint*/
    req_left_arm.group_name = PLANNING_GROUP_LEFT_ARM;
    moveit_msgs::Constraints pose_goal_left_arm = kinematic_constraints::constructGoalConstraints
                                            ("Arm_L_end_effector", pose_left_arm, tolerance_pose, tolerance_angle);
    req_left_arm.goal_constraints.push_back(pose_goal_left_arm);

    /*all the pipeline and check whether planning was successful*/
    planning_pipeline->generatePlan(planning_scene, req_left_arm, res_left_arm);
    if(res_left_arm.error_code_.val != res_left_arm.error_code_.SUCCESS)
    {
        ROS_ERROR("Left arm conld not compute plan successfully");
        return 0;
    }

    /*Visualize the result*/
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>
                                                        ("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory_left_arm;

    ROS_INFO("Visualizing the left arm trajectory");
    moveit_msgs::MotionPlanResponse response_left_arm;
    res_left_arm.getMessage(response_left_arm);

    display_trajectory_left_arm.trajectory_start = response_left_arm.trajectory_start;
    display_trajectory_left_arm.trajectory.push_back(response_left_arm.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory_left_arm.trajectory.back(), joint_model_group_left_arm);
        
    visual_tools.trigger();
    display_publisher.publish(display_trajectory_left_arm);

    /*Display the goal state*/
    visual_tools.publishAxisLabeled(pose_left_arm.pose, "left arm goal_1");
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

/*******************Joint Space Goals Left Arm*******************/
    planning_scene->setCurrentState(response_left_arm.trajectory_start);
    robot_state->setJointGroupPositions(joint_model_group_left_arm, response_left_arm.trajectory.joint_trajectory.points.back().positions);
   
    robot_state::RobotState goal_state_left_arm(robot_model);
    joint_values_left_arm.clear();
    joint_values_left_arm = { 1.5708,  -0.4363, 0, -1.5708, 1.5708, 0}; // front
    goal_state_left_arm.setJointGroupPositions(joint_model_group_left_arm, joint_values_left_arm);
    moveit_msgs::Constraints joint_goal_left_arm = kinematic_constraints::constructGoalConstraints(goal_state_left_arm, joint_model_group_left_arm);
   
    req_left_arm.goal_constraints.clear();
    req_left_arm.goal_constraints.push_back(joint_goal_left_arm);

    /*Call the pipeline and visualize the trajectory*/
    planning_pipeline->generatePlan(planning_scene, req_left_arm, res_left_arm);
    if(res_left_arm.error_code_.val != res_left_arm.error_code_.SUCCESS)
    {
        ROS_ERROR("Left arm conld not compute plan successfully");
        return 0;
    }

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res_left_arm.getMessage(response_left_arm);
    display_trajectory_left_arm.trajectory.push_back(response_left_arm.trajectory);

    visual_tools.publishTrajectoryLine(display_trajectory_left_arm.trajectory.back(), joint_model_group_left_arm);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory_left_arm);
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

/*******************Using a Planning Request Adapter*******************/
    planning_scene->setCurrentState(*robot_state.get());
    planning_scene->setCurrentState(response_left_arm.trajectory_start);
    robot_state->setJointGroupPositions(joint_model_group_left_arm, response_left_arm.trajectory.joint_trajectory.points.back().positions);

    /*set one of the joints slightly outside its upper limit*/
    const robot_model::JointModel* joint_model = joint_model_group_left_arm->getJointModel("Arm_L2_joint");
    const robot_model::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
    std::vector<double> tmp_values(1.0, 0.0);
    tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
    robot_state->setJointPositions(joint_model, tmp_values);
    req_left_arm.goal_constraints.clear();
    req_left_arm.goal_constraints.push_back(joint_goal_left_arm);

    /*Call the planner again and visualize the trajectories*/
    planning_pipeline->generatePlan(planning_scene, req_left_arm, res_left_arm);
    if(res_left_arm.error_code_.val != res_left_arm.error_code_.SUCCESS)
    {
        ROS_ERROR("Left arm conld not compute plan successfully");
        return 0;
    }

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res_left_arm.getMessage(response_left_arm);
    display_trajectory_left_arm.trajectory_start = response_left_arm.trajectory_start;
    display_trajectory_left_arm.trajectory.push_back(response_left_arm.trajectory);

    visual_tools.publishTrajectoryLine(display_trajectory_left_arm.trajectory.back(), joint_model_group_left_arm);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory_left_arm);
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    
      
    ROS_INFO("Done");
    ros::shutdown();
    return 0;
}
