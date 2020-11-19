// http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>
#include "modern_robotics_lib.h"
#include "eod_robotics_lib.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    static const std::string PLANNING_GROUP_RIGHT = "right_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_RIGHT);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_right;

    const robot_state::JointModelGroup* joint_model_group_right = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), 
                std::ostream_iterator<std::string>(std::cout, ", "));
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_right, joint_group_positions);

    VectorXd angle_arm_right(6);
    for (int i=0; i<6; i++)
        angle_arm_right[i] = joint_group_positions[i];

    Matrix4d T_effect = eod_robot_right_arm_FKinSpace(angle_arm_right);
    
    Matrix3d R;  Vector3d p;
    TransToRp(T_effect, R, p);
    Quaterniond q = rotationMatrix2Quaterniond(R);

    geometry_msgs::Pose target_pose1;

    target_pose1.orientation.w = q.w();
    target_pose1.orientation.x = q.x();
    target_pose1.orientation.y = q.y();
    target_pose1.orientation.z = q.z();

    target_pose1.position.x = p(0)-0.2;
    target_pose1.position.y = p(1)-0.1;
    target_pose1.position.z = p(2);

    std::cout << "target_pose1.orientation: " << std::endl << "w = " << q.w()<< std::endl \
                            << "x = " << q.x() << std::endl << "y = " << q.y() << std::endl << "z = " << q.z() ;

    std::cout << "target_pose1.position: " << std::endl << "x = " << target_pose1.position.x << std::endl \
                            << "y = " << target_pose1.position.y << std::endl << "z = " << target_pose1.position.z  << std::endl;

    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "":"FAILED");

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_right);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    joint_group_positions[0] = -1.5708;
    joint_group_positions[1] = -2.618;
    joint_group_positions[2] = 0;
    joint_group_positions[3] = -1.5708;
    joint_group_positions[4] = -1.5708;
    joint_group_positions[5] = 0;

    move_group.setJointValueTarget(joint_group_positions);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "":"FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_right);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");   
    
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "Arm_R_end_effector";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 0.052;
    start_pose2.orientation.x = 0.807;
    start_pose2.orientation.y = -0.585;
    start_pose2.orientation.z = -0.052;
    start_pose2.position.x = 0.561;
    start_pose2.position.y = -0.179;
    start_pose2.position.z = -0.216;  
    start_state.setFromIK(joint_model_group_right, start_pose2);
    move_group.setStartState(start_state);

    move_group.setPoseTarget(target_pose1);

    move_group.setPlanningTime(10.0);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");    

    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start");
    visual_tools.publishAxisLabeled(target_pose1, "goal");
    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_right);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    move_group.clearPathConstraints();

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);

    geometry_msgs::Pose target_pose3 = start_pose2;

    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3);

    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3); 

    target_pose3.position.z += 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3);  

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    collision_object.id = "box1";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.4;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = -0.2;
    box_pose.position.z = 0.6;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface_right.addCollisionObjects(collision_objects);

    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    move_group.setStartState(*move_group.getCurrentState());
    geometry_msgs::Pose another_pose;
    another_pose.orientation.w = 1.0;
    another_pose.position.x = 0.4;
    another_pose.position.y = -0.4;
    another_pose.position.z = 0.4;
    move_group.setPoseTarget(another_pose);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_right);
    visual_tools.trigger();
    visual_tools.prompt("next step");    

    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    move_group.attachObject(collision_object.id);

    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the robot");   

    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    move_group.detachObject(collision_object.id);

    visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the robot");

    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface_right.removeCollisionObjects(object_ids);

    visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

    ros::shutdown();
    return 0;
}