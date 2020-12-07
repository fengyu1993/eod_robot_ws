#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// PI
#include <boost/math/constants/constants.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_diaplay_tutorial");

    /* Needed for ROS_INFO commands to work */
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* Load the robot model */
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    /* Get a shared pointer to the model */
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    /* Get the configuration for the joints*/
    const robot_model::JointModelGroup* joint_model_group_right_arm = kinematic_model->getJointModelGroup("right_arm");
    const robot_model::JointModelGroup* joint_model_group_left_arm = kinematic_model->getJointModelGroup("left_arm");

    /* PUBLISH RANDOM ARM POSITIONS */
    ros::NodeHandle nh;
    ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);

    /* loop at 1 Hz */
    ros::Rate loop_rate(1);
    
    kinematic_state->setToDefaultValues();

    /* 右臂随机位置 */
    for(int cnt = 0; cnt < 5 && ros::ok(); cnt++)
    {
        kinematic_state->setToRandomPositions(joint_model_group_right_arm);

        /* get a robot state message describing the pose in kinematic_state */
        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

        /* send the message to the RobotState display */
        robot_state_publisher.publish(msg);

        /* let ROS send the message, then wait a while */
        ros::spinOnce();
        loop_rate.sleep();
    }

    /* 左臂随机位置 */
    for(int cnt = 0; cnt < 5 && ros::ok(); cnt++)
    {
        kinematic_state->setToRandomPositions(joint_model_group_left_arm);

        /* get a robot state message describing the pose in kinematic_state */
        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

        /* send the message to the RobotState display */
        robot_state_publisher.publish(msg);

        /* let ROS send the message, then wait a while */
        ros::spinOnce();
        loop_rate.sleep();
    }

    /* 右臂末端执行器画圆 */

    /* Find the default pose for the end effector */
    kinematic_state->setToDefaultValues(joint_model_group_right_arm, "work");

    const Eigen::Isometry3d end_effector_default_pose_right_arm = kinematic_state->getGlobalLinkTransform("Arm_R_end_effector");

    const double PI = boost::math::constants::pi<double>();
    const double RADIUS = 0.03;

    for(double angle = 0; angle <= 2 * PI && ros::ok(); angle += 2 * PI / 20)
    {
        /* calculate a position for the end effector */
        Eigen::Isometry3d end_effector_pose = Eigen::Translation3d(RADIUS * cos(angle), RADIUS * sin(angle), 0.0) * end_effector_default_pose_right_arm;

        ROS_INFO_STREAM("End effector position:\n" << end_effector_pose.translation());

        /* use IK to get joint angles satisfyuing the calculated position */
        bool found_ik = kinematic_state->setFromIK(joint_model_group_right_arm, end_effector_pose, 10, 0.1);
        if (!found_ik)
        {
            ROS_INFO_STREAM("Could not solve IK for pose\n" << end_effector_pose.translation());
            continue;
        }
        else
        {
            ROS_INFO_STREAM("Could solve IK for pose\n");
        }

        /* get a robot state message describing the pose in kinematic_state */
        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

        /* send the message to the RobotState display */
        robot_state_publisher.publish(msg);

        /* let ROS send the message, then wait a while */
        ros::spinOnce();
        loop_rate.sleep();
    }

   /* 左臂末端执行器画圆 */

    /* Find the default pose for the end effector */
    kinematic_state->setToDefaultValues(joint_model_group_left_arm, "work");

    const Eigen::Isometry3d end_effector_default_pose_left_arm = kinematic_state->getGlobalLinkTransform("Arm_L_end_effector");

    for(double angle = 0; angle <= 2 * PI && ros::ok(); angle += 2 * PI / 20)
    {
        /* calculate a position for the end effector */
        Eigen::Isometry3d end_effector_pose = Eigen::Translation3d(RADIUS * cos(angle), RADIUS * sin(angle), 0.0) * end_effector_default_pose_left_arm;

        ROS_INFO_STREAM("End effector position:\n" << end_effector_pose.translation());

        /* use IK to get joint angles satisfyuing the calculated position */
        bool found_ik = kinematic_state->setFromIK(joint_model_group_left_arm, end_effector_pose, 10, 0.1);
        if (!found_ik)
        {
            ROS_INFO_STREAM("Could not solve IK for pose\n" << end_effector_pose.translation());
            continue;
        }
        else
        {
            ROS_INFO_STREAM("Could solve IK for pose\n");
        }

        /* get a robot state message describing the pose in kinematic_state */
        moveit_msgs::DisplayRobotState msg;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

        /* send the message to the RobotState display */
        robot_state_publisher.publish(msg);

        /* let ROS send the message, then wait a while */
        ros::spinOnce();
        loop_rate.sleep();
    }


    ros::shutdown();
    return 0;
}