// http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    /* Add finger joints of eod robot. */
    posture.joint_names.resize(1);
    posture.joint_names[0] = "finger_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);    
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    /* Add finger joints of eod robot. */
    posture.joint_names.resize(1);
    posture.joint_names[0] = "finger_joint";

    /* Set them as closed */
    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.70;
    posture.points[0].time_from_start = ros::Duration(0.5); 
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI/2, -M_PI/4, -M_PI/2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0.1;
    grasps[0].grasp_pose.pose.position.z = 0.1;

    // Setting pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    //Setting post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    openGripper(grasps[0].pre_grasp_posture);

    // Setting posture of eef during grasp
    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table1
    move_group.setSupportSurfaceName("table1");

    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI/2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;

    // Setting pre-place approach
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    move_group.setSupportSurfaceName("table2");

    // Call place to place the object using the place locations given.
    move_group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0.1;
    collision_objects[0].primitive_poses[0].position.z = -0.5;

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.1;
    collision_objects[1].primitive_poses[0].position.y = 0.45;
    collision_objects[1].primitive_poses[0].position.z = -0.5;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0.1;
    collision_objects[2].primitive_poses[0].position.z = -0.2;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void prepare_pick_place()
{
    static const std::string PLANNING_GROUP_RIGHT_ARM = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_RIGHT_ARM);
    const robot_state::JointModelGroup* joint_model_group_right = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_RIGHT_ARM);

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_right, joint_group_positions);

    joint_group_positions[0] = 0;
    joint_group_positions[1] = -0.4363;
    joint_group_positions[2] = 0.7854;
    joint_group_positions[3] = -1.5708;
    joint_group_positions[4] = -2.7053;
    joint_group_positions[5] = 0;

    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success)
    {
        ROS_INFO("success plan");
        move_group.move();
    }
    else
        ROS_INFO("fail plan");   
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "eod_robot_arm_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("right_arm");
    group.setPlanningTime(45.0);

    prepare_pick_place();

    ros::WallDuration(1.0).sleep();

    addCollisionObjects(planning_scene_interface);

    ros::WallDuration(1.0).sleep();

    pick(group);

    ros::WallDuration(1.0).sleep();

    place(group);

    ros::waitForShutdown();
    return 0;
}