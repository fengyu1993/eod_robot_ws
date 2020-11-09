# include <ros/ros.h>

# include <moveit/robot_model_loader/robot_model_loader.h>
# include <moveit/robot_model/robot_model.h>
# include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group_right_arm = kinematic_model->getJointModelGroup("right_arm");

    const std::vector<std::string>& joint_names_right_arm = joint_model_group_right_arm->getVariableNames();

    std::vector<double> joint_values_right_arm;
    kinematic_state->copyJointGroupPositions(joint_model_group_right_arm, joint_values_right_arm);
    for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
    {
        ROS_INFO("Right Arm Joint %s: %f", joint_names_right_arm[i].c_str(), joint_values_right_arm[i]);
    }

    joint_values_right_arm[0] = 10;
    kinematic_state->setJointGroupPositions(joint_model_group_right_arm, joint_values_right_arm);

    ROS_INFO_STREAM("Right Arm Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Right Arm Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    kinematic_state->copyJointGroupPositions(joint_model_group_right_arm, joint_values_right_arm);
    for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
    {
        ROS_INFO("Right Arm Joint %s: %f", joint_names_right_arm[i].c_str(), joint_values_right_arm[i]);
    } 

    // kinematic_state->setToRandomPositions(joint_model_group_right_arm);
    joint_values_right_arm[0] = 4.778741;
    joint_values_right_arm[1] = 0.001709; 
    joint_values_right_arm[2] = 3.867487;
    joint_values_right_arm[3] = -1.588801;
    joint_values_right_arm[4] = -5.517637;
    joint_values_right_arm[5] = 1.247915;
    kinematic_state->setJointGroupPositions(joint_model_group_right_arm, joint_values_right_arm);
    const Eigen::Isometry3d& end_effector_state_right_arm = kinematic_state->getGlobalLinkTransform("Arm_R_end_effector");

    ROS_INFO_STREAM("Right Arm Translation: \n" << end_effector_state_right_arm.translation() << "\n");
    ROS_INFO_STREAM("Right Arm Rotation: \n" << end_effector_state_right_arm.rotation() << "\n");

    kinematic_state->copyJointGroupPositions(joint_model_group_right_arm, joint_values_right_arm);
    for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
    {
        ROS_INFO("Right Arm Joint %s: %f", joint_names_right_arm[i].c_str(), joint_values_right_arm[i]);
    }
    
    double timeout = 0.1;
    bool found_ik_right_arm = kinematic_state->setFromIK(joint_model_group_right_arm, end_effector_state_right_arm, timeout);

    if (found_ik_right_arm)
    {
        ROS_INFO("Find IK solution");
        kinematic_state->copyJointGroupPositions(joint_model_group_right_arm, joint_values_right_arm);
        for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
        {
            ROS_INFO("Right Arm Joint %s: %f", joint_names_right_arm[i].c_str(), joint_values_right_arm[i]);
        }
    }
    else
    {
        ROS_INFO("Right Arm Did not find IK solution");
    }

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian_right_arm;
    kinematic_state->getJacobian(joint_model_group_right_arm, \
                                 kinematic_state->getLinkModel(joint_model_group_right_arm->getLinkModelNames().back()),\
                                 reference_point_position, jacobian_right_arm);
    // jacobian_right_arm =  kinematic_state->getJacobian(joint_model_group_right_arm);
    ROS_INFO_STREAM("Right Arm Jacobian: \n" << jacobian_right_arm << "\n");

    ros::shutdown();
    return 0;
}