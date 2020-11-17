// http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_scene_ros_api_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle;

    /*Visualization*/
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    /*Advertise the required topic*/
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    /*Define the attached object message*/
    // 在环境中添加一个立方体（会检测到立方体与机器人发生碰撞）
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "right_inner_finger_pad";
    attached_object.object.header.frame_id = "right_inner_finger_pad";
    attached_object.object.id = "box";
    
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    attached_object.object.operation = attached_object.object.ADD;

    attached_object.touch_links = std::vector<std::string>
        {"right_outer_finger", "right_inner_knuckle", "right_inner_finger", "right_inner_finger_pad"}; // 避免碰撞检测

    ROS_INFO("Adding the object into the world at the location of the hand.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    /*Interlude: Synchronous vs Asynchronous updates*/
    // ros::ServiceClient planning_scene_diff_client = 
    //     node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    // planning_scene_diff_client.waitForExistence();

    // moveit_msgs::ApplyPlanningScene srv;
    // srv.request.scene = planning_scene;
    // planning_scene_diff_client.call(srv);

    /*Attach an object to the robot*/
    // 去掉上一步添加的立方体，并且在机器人连杆上添加一个立方体（该立方体是机器人连杆的一部分，所以无碰撞）
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = "box";
    remove_object.header.frame_id = "base_link";
    remove_object.operation = remove_object.REMOVE;

    ROS_INFO("Attaching the object to the hand and removing it from the world.");
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_diff_publisher.publish(planning_scene);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    /*Detach an object from the robot*/
    // 去掉上一步添加在机器人连杆上的立方体，那么该一个立方体会自动默认为在环境中
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = "box";
    detach_object.link_name = "right_inner_finger_pad";
    detach_object.object.operation = attached_object.object.REMOVE;

    ROS_INFO("Detaching the object from the robot and returning it to the world.");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene.world.collision_objects.clear();
    // planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /*Remove the object from the collision world*/
    // 移除上一步在环境中添加的立方体
    ROS_INFO("Removing the object from the world.");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene_diff_publisher.publish(planning_scene);


    ros::shutdown();
    return 0;
}