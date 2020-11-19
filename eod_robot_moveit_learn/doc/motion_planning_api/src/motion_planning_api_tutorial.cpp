#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
    const std::string node_name = "motion_planning_tutorial";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

/*******************Start*******************/
    /*construct a RobotModel*/
    const std::string PLANNING_GROUP = "right_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    /*construct a PlanningScene*/
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    /*configure a valid robot state*/
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    /*construct a loader to load a planner*/
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    /*get the name of planning plugin, load the planner*/
    if(!node_handle.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if(!planner_instance->initialize(robot_model, node_handle.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch(pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for(std::size_t i=0; i<classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what()
                                        <<std::endl << "Available plugins: " << ss.str());
    }

/*******************Visualization*******************/
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API Demo",rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

/*******************Pose Goal*******************/
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.5591;
    pose.pose.position.y = 0.0498026;
    pose.pose.position.z = 0.240332;
    pose.pose.orientation.w = 0;    
    pose.pose.orientation.x = 0; 
    pose.pose.orientation.y = 0.216589; 
    pose.pose.orientation.z = 0.976263; 

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    /*create the request as a constraint*/
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints
                                            ("Arm_R_end_effector", pose, tolerance_pose, tolerance_angle);
    
    req.group_name = PLANNING_GROUP;
    req.goal_constraints.push_back(pose_goal);

    /*construct a planning context*/
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Conld not compute plan successfully");
        return 0;
    }

/*******************Visualize the result*******************/
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>
                                                        ("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    /*Display the goal state*/
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.publishAxisLabeled(pose.pose, "goal_1");
    visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /*Joint Space Goals*/
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values = { 4.778741,  0.001709, 3.867487, -1.588801, -5.517637, 1.247915};
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    /*Call the planner and visualize the trajectory*/
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Conld not compute plan successfully");
        return 0;
    }
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);

    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    /*Display the goal state*/
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.publishAxisLabeled(pose.pose, "goal_2");
    visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    res.getMessage(response);

    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    /*Display the goal state*/
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);  
    visual_tools.trigger();  

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

/*******************Adding Path Constraints*******************/
    pose.pose.position.x = 0.2591;
    pose.pose.position.y = -0.0498026;
    pose.pose.position.z = 0.540332;
    pose.pose.orientation.w = 0;    
    pose.pose.orientation.x = 0; 
    pose.pose.orientation.y = 0.216589; 
    pose.pose.orientation.z = 0.976263;   
    moveit_msgs::Constraints pose_goal_2 = kinematic_constraints::constructGoalConstraints
                                 ("Arm_R_end_effector", pose, tolerance_pose, tolerance_angle);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal_2);

    geometry_msgs::QuaternionStamped quaternion;
    quaternion.header.frame_id = "base_link";
    quaternion.quaternion.w = 1.0;
    req.path_constraints = kinematic_constraints::constructGoalConstraints("Arm_R_end_effector", quaternion);

    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = 
        req.workspace_parameters.min_corner.z = -2.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 
        req.workspace_parameters.max_corner.z = -2.0;

    /*Call the planner and visualize all the plans*/
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    /*Display the goal state*/
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.publishAxisLabeled(pose.pose, "goal_3");
    visual_tools.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();




    ros::shutdown();
    return 0;
}