#ifndef EOD_ROBOTICS_LIB_H
#define EOD_ROBOTICS_LIB_H

#include <Eigen/Dense>
#include <modern_robotics_lib.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define     ANGLE_ARM_RIGHT     "angle_arm_right"
#define     ANGLE_ARM_LEFT      "angle_arm_left"
#define     GRIPPER_FINGER1     "gripper_finger1"
#define     GRIPPER_LIMIT_MIN   "gripper_finger1_limit_min"
#define     GRIPPER_LIMIT_MAX   "gripper_finger1_limit_max"
#define     VECIHLE_ODOM        "vecihle_odom"
#define     VECIHLE_LINEAR      "vecihle_linear"
#define     VECIHLE_ANGULAR     "vecihle_angular"


Matrix4d eod_robot_FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist, Matrix4d T_base_arm);

Matrix4d eod_robot_FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist, Matrix4d T_base_arm);

bool eod_robot_IKinSpace(MatrixXd Slist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist, Matrix4d T_base_arm);

bool eod_robot_IKinBody(MatrixXd Blist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist, Matrix4d T_base_arm);

Matrix4d eod_robot_FKinSpace_world(Matrix4d M, MatrixXd Slist, VectorXd thetalist, Matrix4d T_base_arm, Matrix4d vecihle_odom);

Matrix4d eod_robot_FKinBody_world(Matrix4d M, MatrixXd Blist, VectorXd thetalist, Matrix4d T_base_arm, Matrix4d vecihle_odom);

MatrixXd eod_robot_JacobianSpace(MatrixXd Slist,VectorXd thetalist, Matrix4d T_base_arm);

bool get_arm_left_param(MatrixXd& Slist_arm_left, Matrix4d& M_arm_left, Matrix4d& T_base_left_arm);

bool get_arm_right_param(MatrixXd& Slist_arm_right, Matrix4d& M_arm_right, Matrix4d& T_base_right_arm);

bool get_arm_right_joint_angle(VectorXd& angle_arm_right);

bool get_arm_left_joint_angle(VectorXd& angle_arm_left);

bool get_gripper_finger1_position(double& gripper_finger1);

bool get_gripper_finger1_limit_max(double& gripper_finger1_limit_max);

bool get_gripper_finger1_limit_min(double& gripper_finger1_limit_min);

bool get_vecihle_odom(Matrix4d& vecihle_odom);

bool get_vecihle_velocity(double& linear, double& angular);

bool set_arm_right_joint_angle(VectorXd angle_arm_right);

bool set_arm_left_joint_angle(VectorXd angle_arm_left);

bool set_gripper_finger1_position(double gripper_finger1);

bool set_vecihle_odom(Matrix4d vecihle_odom);

bool set_vecihle_velocity(double linear, double angular);

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);

Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w);

Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w);

Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R);

Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw);

Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R);

void Matrix_T2tf_transform(Matrix4d T, geometry_msgs::TransformStamped& trans);

void Matrix_R2tf_rotation(Matrix3d R, geometry_msgs::TransformStamped& trans);

void Matrix_p2tf_translation(Vector3d p, geometry_msgs::TransformStamped& trans);

void tf_transform2Matrix_T(geometry_msgs::TransformStamped trans, Matrix4d& T);

void tf_rotation2Matrix_R(geometry_msgs::TransformStamped trans, Matrix3d& R);

void tf_translation2Matrix_p(geometry_msgs::TransformStamped trans, Vector3d& p);

#endif //EOD_ROBOTICS_LIB_H