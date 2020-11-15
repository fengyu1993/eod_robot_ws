#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

bool stateFeasibilityTestExample(const robot_state::RobotState& kinematic_state, bool verbose)
{
    const double* joint_values = kinematic_state.getJointPositions("Arm_R1_joint");
    return (joint_values[0] > 0.0);
}

int main(int argc, char** argv)
{
    /*ros init*/
    ros::init(argc, argv, "panda_arm_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::size_t count = 0;

/*****************Collision Checking*****************/

    /*PlanningScene instantiate*/
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    /*Self-collision checking*/
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    /*Change the state*/
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    /*Checking for a group*/
    collision_request.group_name = "right_arm";
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    /*Getting Contact Information*/
    std::vector<double> joint_values = { 0.0, 1.0, 2.0, -2.9, 1.0, 1.4, 0.0 };
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("right_arm");
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO_STREAM("Test 4: Current state is " << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
    {
        ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }

    /*Modifying the Allowed Collision Matrix*/
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for(it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    /*Full Collision Checking*/
    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

/*****************Constraint Checking*****************/

    /*Checking Kinematic Constraints*/
    std::string end_effector_name = joint_model_group->getLinkModelNames().back();

    geometry_msgs::PoseStamped desired_pose;
    desired_pose.pose.orientation.w = 0.746917;
    desired_pose.pose.orientation.x = -0.283898;
    desired_pose.pose.orientation.y = 0.552079;
    desired_pose.pose.orientation.z = 0.238171;
    desired_pose.pose.position.x = 0.249125;
    desired_pose.pose.position.y = -0.341856;
    desired_pose.pose.position.z = 0.100761;
    desired_pose.header.frame_id = "base_link";
    moveit_msgs::Constraints goal_constraint = 
                kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

    copied_state.setToRandomPositions();
    copied_state.update();
    bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
    ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));

    kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
    kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
    bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
    ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not constrained"));

    kinematic_constraints::ConstraintEvaluationResult constraint_eval_result = kinematic_constraint_set.decide(copied_state);
    ROS_INFO_STREAM("Test 10: Random state is " << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

    planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
    bool state_feasible = planning_scene.isStateFeasible(copied_state);
    ROS_INFO_STREAM("Test 11: Random state is " << (state_feasible ? "feasible" : "not feasible"));

    bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "right_arm");
    ROS_INFO_STREAM("Test 12: Random state is " << (state_valid ? "valid" : "not valid"));
    
    ros::shutdown();
    return 0;
}