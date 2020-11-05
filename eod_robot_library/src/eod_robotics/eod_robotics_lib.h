#ifndef EOD_ROBOTICS_LIB_H
#define EOD_ROBOTICS_LIB_H

#include <Eigen/Dense>
#include <modern_robotics_lib.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define     ANGLE_ARM_RIGHT             "angle_arm_right"
#define     ANGLE_ARM_LEFT              "angle_arm_left"
#define     GRIPPER_FINGER1             "gripper_finger1"
#define     GRIPPER_LIMIT_MIN           "gripper_finger1_limit_min"
#define     GRIPPER_LIMIT_MAX           "gripper_finger1_limit_max"
#define     VECIHLE_ODOM                "vecihle_odom"
#define     VECIHLE_LINEAR              "vecihle_linear"
#define     VECIHLE_ANGULAR             "vecihle_angular"
#define     T_BASE_LEFT_ARM             "T_base_left_arm"
#define     T_BASE_RIGHT_ARM            "T_base_right_arm"
#define     SLIST_ARM_LEFT              "Slist_arm_left"
#define     M_ARM_LEFT                  "M_arm_left"
#define     SLIST_ARM_RIGHT             "Slist_arm_right"
#define     M_ARM_RIGHT                 "M_arm_right"
#define     T_LEFT_EFFECTOR_CAMERA      "T_left_effector_camera"
#define     T_RIGHT_EFFECTOR_CAMERA     "T_right_effector_camera"
#define     BLIST_ARM_LEFT              "Blist_arm_left"
#define     BLIST_ARM_RIGHT             "Blist_arm_right"

/*机器人Base坐标系正运动学：相对于base_link坐标系*/
Matrix4d eod_robot_FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist, Matrix4d T_base_arm);

/*机器人右臂Base坐标系正运动学：相对于base_link坐标系*/
Matrix4d eod_robot_right_arm_FKinSpace(VectorXd thetalist);

/*机器人左臂Base坐标系正运动学：相对于base_link坐标系*/
Matrix4d eod_robot_left_arm_FKinSpace(VectorXd thetalist);

/*机器人Body坐标系正运动学：相对于base_link坐标系*/
Matrix4d eod_robot_FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist, Matrix4d T_base_arm);

/*机器人右臂Body坐标系正运动学：相对于base_link坐标系*/
Matrix4d eod_robot_right_arm_FKinBody(VectorXd thetalist);

/*机器人左臂Body坐标系正运动学：相对于base_link坐标系*/
Matrix4d eod_robot_left_arm_FKinBody(VectorXd thetalist);

/*机器人Base坐标系逆运动学：牛顿-辛普森方法*/
bool eod_robot_IKinSpace_NR(MatrixXd Slist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist, Matrix4d T_base_arm);

/*机器人Base坐标系逆运动学：POE解析方法*/
bool eod_robot_IKinSpace_POE(MatrixXd Slist, Matrix4d M, Matrix4d T, VectorXd thetalist0, int method, VectorXd& thetalist, Matrix4d T_base_arm);

/*机器人右臂Base坐标系逆运动学：牛顿-辛普森方法*/
bool eod_robot_right_arm_IKinSpace_NR(Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist);

/*机器人右臂Base坐标系逆运动学：POE解析方法*/
bool eod_robot_right_arm_IKinSpace_POE(Matrix4d T, VectorXd thetalist0, int method, VectorXd& thetalist);

/*机器人左臂Base坐标系逆运动学：牛顿-辛普森方法*/
bool eod_robot_left_arm_IKinSpace_NR(Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist);

/*机器人左臂Base坐标系逆运动学：POE解析方法*/
bool eod_robot_left_arm_IKinSpace_POE(Matrix4d T, VectorXd thetalist0, int method, VectorXd& thetalist);

/*机器人Body坐标系逆运动学：牛顿-辛普森方法*/
bool eod_robot_IKinBody_NR(MatrixXd Blist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist, Matrix4d T_base_arm);

/*机器人右臂Body坐标系逆运动学：牛顿-辛普森方法*/
bool eod_robot_right_arm_IKinBody_NR(Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist);

/*机器人左臂Body坐标系逆运动学：牛顿-辛普森方法*/
bool eod_robot_left_arm_IKinBody_NR(Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist);

/*机器人Base坐标系正运动学：相对于世界坐标系*/
Matrix4d eod_robot_FKinSpace_world(Matrix4d M, MatrixXd Slist, VectorXd thetalist, Matrix4d T_base_arm, Matrix4d vecihle_odom);

/*机器人右臂Base坐标系正运动学：相对于世界坐标系*/
Matrix4d eod_robot_right_arm_FKinSpace_world(VectorXd thetalist);

/*机器人左臂Base坐标系正运动学：相对于世界坐标系*/
Matrix4d eod_robot_left_arm_FKinSpace_world(VectorXd thetalist);

/*机器人Body坐标系正运动学：相对于世界坐标系*/
Matrix4d eod_robot_FKinBody_world(Matrix4d M, MatrixXd Blist, VectorXd thetalist, Matrix4d T_base_arm, Matrix4d vecihle_odom);

/*机器人右臂Body坐标系正运动学：相对于世界坐标系*/
Matrix4d eod_robot_right_arm_FKinBody_world(VectorXd thetalist);

/*机器人左臂Body坐标系正运动学：相对于世界坐标系*/
Matrix4d eod_robot_left_arm_FKinBody_world(VectorXd thetalist);

/*机器人Base坐标系雅克比矩阵*/
MatrixXd eod_robot_JacobianSpace(MatrixXd Slist,VectorXd thetalist, Matrix4d T_base_arm);

/*机器人右臂Base坐标系雅克比矩阵*/
MatrixXd eod_robot_right_arm_JacobianSpace(VectorXd thetalist);

/*机器人左臂Base坐标系雅克比矩阵*/
MatrixXd eod_robot_left_arm_JacobianSpace(VectorXd thetalist);

/*机器人Body坐标系雅克比矩阵*/
MatrixXd eod_robot_JacobianBody(MatrixXd Blist,VectorXd thetalist);

/*机器人右臂Body坐标系雅克比矩阵*/
MatrixXd eod_robot_right_arm_JacobianBody(VectorXd thetalist);

/*机器人左臂Body坐标系雅克比矩阵*/
MatrixXd eod_robot_left_arm_JacobianBody(VectorXd thetalist);

/*获取左臂Base坐标系POE参数*/
bool get_arm_left_ParamSpace(MatrixXd& Slist_arm_left, Matrix4d& M_arm_left, Matrix4d& T_base_left_arm);

/*获取右臂Base坐标系POE参数*/
bool get_arm_right_ParamSpace(MatrixXd& Slist_arm_right, Matrix4d& M_arm_right, Matrix4d& T_base_right_arm);

/*获取左臂Body坐标系POE参数*/
bool get_arm_left_ParamBody(MatrixXd& Blist_arm_left, Matrix4d& M_arm_left, Matrix4d& T_base_left_arm);

/*获取右臂Body坐标系POE参数*/
bool get_arm_right_ParamBody(MatrixXd& Blist_arm_right, Matrix4d& M_arm_right, Matrix4d& T_base_right_arm);

/*获取右臂期望关节角度*/
bool get_arm_right_joint_angle(VectorXd& angle_arm_right);

/*获取左臂期望关节角度*/
bool get_arm_left_joint_angle(VectorXd& angle_arm_left);

/*获取左臂摄像头相对于末端执行器的变换矩阵*/
bool get_arm_left_T_effector_camera(Matrix4d& T_effector_camera);

/*获取右臂摄像头相对于末端执行器的变换矩阵*/
bool get_arm_right_T_effector_camera(Matrix4d& T_effector_camera);

/*获取夹爪期望位置*/
bool get_gripper_finger1_position(double& gripper_finger1);

/*获取夹爪最大限位*/
bool get_gripper_finger1_limit_max(double& gripper_finger1_limit_max);

/*获取夹爪最小限位*/
bool get_gripper_finger1_limit_min(double& gripper_finger1_limit_min);

/*获取车体相对于世界坐标系的变换矩阵*/
bool get_vecihle_odom(Matrix4d& vecihle_odom);

/*获取车体线速度和角速度*/
bool get_vecihle_velocity(double& linear, double& angular);

/*设置右臂期望关节角度*/
bool set_arm_right_joint_angle(VectorXd angle_arm_right);

/*设置左臂期望关节角度*/
bool set_arm_left_joint_angle(VectorXd angle_arm_left);

/*设置夹爪期望位置*/
bool set_gripper_finger1_position(double gripper_finger1);

/*设置车体相对于世界坐标系的变换矩阵*/
bool set_vecihle_odom(Matrix4d vecihle_odom);

/*设置车体线速度和角速度*/
bool set_vecihle_velocity(double linear, double angular);

/*欧拉角转四元数*/
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);

/*四元数转欧拉角*/
Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w);

/*四元数转旋转矩阵*/
Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w);

/*旋转矩阵转四元数*/
Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R);

/*欧拉角转旋转矩阵*/
Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw);

/*旋转矩阵转欧拉角*/
Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R);

/*Eigen的变换矩阵转tf的变换矩阵*/
void Matrix_T2tf_transform(Matrix4d T, geometry_msgs::TransformStamped& trans);

/*Eigen的旋转矩阵转tf的旋转矩阵*/
void Matrix_R2tf_rotation(Matrix3d R, geometry_msgs::TransformStamped& trans);

/*Eigen的位置向量转tf的位置向量*/
void Matrix_p2tf_translation(Vector3d p, geometry_msgs::TransformStamped& trans);

/*tf的变换矩阵转Eigen的变换矩阵*/
void tf_transform2Matrix_T(geometry_msgs::TransformStamped trans, Matrix4d& T);

/*tf的旋转矩阵转Eigen的旋转矩阵*/
void tf_rotation2Matrix_R(geometry_msgs::TransformStamped trans, Matrix3d& R);

/*tf的位置向量转Eigen的位置向量*/
void tf_translation2Matrix_p(geometry_msgs::TransformStamped trans, Vector3d& p);




#endif //EOD_ROBOTICS_LIB_H