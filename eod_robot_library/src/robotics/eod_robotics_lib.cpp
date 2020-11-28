#include "eod_robotics_lib.h"

Matrix4d eod_robot_FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist, Matrix4d T_base_arm)
{
    return T_base_arm * FKinSpace(M, Slist, thetalist);
}

Matrix4d eod_robot_right_arm_FKinSpace(VectorXd thetalist)
{
    MatrixXd Slist_arm_right(6,6);  Matrix4d M_arm_right; Matrix4d T_base_right_arm;

    get_arm_right_ParamSpace(Slist_arm_right, M_arm_right, T_base_right_arm);

    return eod_robot_FKinSpace(M_arm_right, Slist_arm_right, thetalist, T_base_right_arm);
}

Matrix4d eod_robot_left_arm_FKinSpace(VectorXd thetalist)
{
    MatrixXd Slist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm;

    get_arm_left_ParamSpace(Slist_arm_left, M_arm_left, T_base_left_arm);

    return eod_robot_FKinSpace(M_arm_left, Slist_arm_left, thetalist, T_base_left_arm);
}

Matrix4d eod_robot_FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist, Matrix4d T_base_arm)
{
    return T_base_arm * FKinBody(M, Blist, thetalist);
}

Matrix4d eod_robot_right_arm_FKinBody(VectorXd thetalist)
{
    MatrixXd Blist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm;

    get_arm_right_ParamBody(Blist_arm_right, M_arm_right, T_base_right_arm);

    return eod_robot_FKinBody(M_arm_right, Blist_arm_right, thetalist, T_base_right_arm);    
}

Matrix4d eod_robot_left_arm_FKinBody(VectorXd thetalist)
{
    MatrixXd Blist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm;

    get_arm_left_ParamSpace(Blist_arm_left, M_arm_left, T_base_left_arm);

    return eod_robot_FKinSpace(M_arm_left, Blist_arm_left, thetalist, T_base_left_arm);
}

bool eod_robot_IKinSpace_NR(MatrixXd Slist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist, Matrix4d T_base_arm)
{
    Matrix4d T_temp  = TransInv(T_base_arm) * T;

    return IKinSpace(Slist, M, T_temp, thetalist0, eomg, ev, thetalist);
}

bool eod_robot_IKinSpace_POE(MatrixXd Slist, Matrix4d M, Matrix4d T, VectorXd thetalist0, int method, VectorXd& thetalist, Matrix4d T_base_arm)
{
    Matrix4d T_temp = TransInv(T_base_arm) * T;

    return IKinSpace_POE(Slist, M, T_temp, thetalist0, method, thetalist);  
}

bool eod_robot_right_arm_IKinSpace_NR(Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist)
{
    MatrixXd Slist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm;

    get_arm_right_ParamSpace(Slist_arm_right, M_arm_right, T_base_right_arm);    

    return eod_robot_IKinSpace_NR(Slist_arm_right, M_arm_right, T, thetalist0, eomg, ev, thetalist, T_base_right_arm);
}

bool eod_robot_right_arm_IKinSpace_POE(Matrix4d T, VectorXd thetalist0, int method, VectorXd& thetalist)
{
    MatrixXd Slist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm;

    get_arm_right_ParamSpace(Slist_arm_right, M_arm_right, T_base_right_arm);    

    return eod_robot_IKinSpace_POE(Slist_arm_right, M_arm_right, T, thetalist0, method, thetalist, T_base_right_arm);    
}

bool eod_robot_left_arm_IKinSpace_NR(Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist)
{
    MatrixXd Slist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm;

    get_arm_left_ParamSpace(Slist_arm_left, M_arm_left, T_base_left_arm);  

    return eod_robot_IKinSpace_NR(Slist_arm_left, M_arm_left, T, thetalist0, eomg, ev, thetalist, T_base_left_arm);  
}

bool eod_robot_left_arm_IKinSpace_POE(Matrix4d T, VectorXd thetalist0, int method, VectorXd& thetalist)
{
    MatrixXd Slist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm;

    get_arm_left_ParamSpace(Slist_arm_left, M_arm_left, T_base_left_arm);  

    return eod_robot_IKinSpace_POE(Slist_arm_left, M_arm_left, T, thetalist0, method, thetalist, T_base_left_arm);     
}

bool eod_robot_IKinBody_NR(MatrixXd Blist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist, Matrix4d T_base_arm)
{
    Matrix4d T_temp;

    T_temp = TransInv(T_base_arm) * T;

    return IKinBody(Blist, M, T_temp, thetalist0, eomg, ev, thetalist);

}

bool eod_robot_right_arm_IKinBody_NR(Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist)
{
    MatrixXd Blist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm;

    get_arm_right_ParamBody(Blist_arm_right, M_arm_right, T_base_right_arm);    

    return eod_robot_IKinBody_NR(Blist_arm_right, M_arm_right, T, thetalist0, eomg, ev, thetalist, T_base_right_arm);
}

bool eod_robot_left_arm_IKinBody_NR(Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist)
{
    MatrixXd Blist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm;

    get_arm_left_ParamBody(Blist_arm_left, M_arm_left, T_base_left_arm);  

    return eod_robot_IKinBody_NR(Blist_arm_left, M_arm_left, T, thetalist0, eomg, ev, thetalist, T_base_left_arm); 
}


Matrix4d eod_robot_FKinSpace_world(Matrix4d M, MatrixXd Slist, VectorXd thetalist, Matrix4d T_base_arm, Matrix4d vecihle_odom)
{
    return vecihle_odom * eod_robot_FKinSpace(M, Slist, thetalist, T_base_arm);
}

Matrix4d eod_robot_right_arm_FKinSpace_world(VectorXd thetalist)
{
    MatrixXd Slist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm; Matrix4d vecihle_odom;

    get_arm_right_ParamSpace(Slist_arm_right, M_arm_right, T_base_right_arm);

    get_vecihle_odom(vecihle_odom);

    return eod_robot_FKinSpace_world(M_arm_right, Slist_arm_right, thetalist, T_base_right_arm, vecihle_odom);
}

Matrix4d eod_robot_left_arm_FKinSpace_world(VectorXd thetalist)
{
    MatrixXd Slist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm; Matrix4d vecihle_odom;

    get_arm_left_ParamSpace(Slist_arm_left, M_arm_left, T_base_left_arm); 

    get_vecihle_odom(vecihle_odom);

    return eod_robot_FKinSpace_world(M_arm_left, Slist_arm_left, thetalist, T_base_left_arm, vecihle_odom);
}

Matrix4d eod_robot_FKinBody_world(Matrix4d M, MatrixXd Blist, VectorXd thetalist, Matrix4d T_base_arm, Matrix4d vecihle_odom)
{
    return vecihle_odom * eod_robot_FKinBody(M, Blist, thetalist, T_base_arm);
}

Matrix4d eod_robot_right_arm_FKinBody_world(VectorXd thetalist)
{
    MatrixXd Blist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm; Matrix4d vecihle_odom;

    get_arm_right_ParamBody(Blist_arm_right, M_arm_right, T_base_right_arm);

    get_vecihle_odom(vecihle_odom);

    return eod_robot_FKinBody_world(M_arm_right, Blist_arm_right, thetalist, T_base_right_arm, vecihle_odom);
}

Matrix4d eod_robot_left_arm_FKinBody_world(VectorXd thetalist)
{
    MatrixXd Blist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm; Matrix4d vecihle_odom;

    get_arm_left_ParamBody(Blist_arm_left, M_arm_left, T_base_left_arm); 

    get_vecihle_odom(vecihle_odom);

    return eod_robot_FKinBody_world(M_arm_left, Blist_arm_left, thetalist, T_base_left_arm, vecihle_odom);
}

MatrixXd eod_robot_JacobianSpace(MatrixXd Slist,VectorXd thetalist, Matrix4d T_base_arm)
{
    return Adjoint(T_base_arm) * JacobianSpace(Slist, thetalist);
}

MatrixXd eod_robot_right_arm_JacobianSpace(VectorXd thetalist)
{
    MatrixXd Slist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm; 

    get_arm_right_ParamSpace(Slist_arm_right, M_arm_right, T_base_right_arm);

    return eod_robot_JacobianSpace(Slist_arm_right, thetalist, T_base_right_arm);
}

MatrixXd eod_robot_left_arm_JacobianSpace(VectorXd thetalist)
{
    MatrixXd Slist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm; 

    get_arm_left_ParamSpace(Slist_arm_left, M_arm_left, T_base_left_arm); 

    return eod_robot_JacobianSpace(Slist_arm_left, thetalist, T_base_left_arm);
}

MatrixXd eod_robot_JacobianBody(MatrixXd Blist,VectorXd thetalist)
{
    return JacobianBody(Blist, thetalist);
}

MatrixXd eod_robot_right_arm_JacobianBody(VectorXd thetalist)
{
    MatrixXd Blist_arm_right(6,6); Matrix4d M_arm_right; Matrix4d T_base_right_arm; 

    get_arm_right_ParamBody(Blist_arm_right, M_arm_right, T_base_right_arm);

    return eod_robot_JacobianBody(Blist_arm_right, thetalist);
}

MatrixXd eod_robot_left_arm_JacobianBody(VectorXd thetalist)
{
    MatrixXd Blist_arm_left(6,6); Matrix4d M_arm_left; Matrix4d T_base_left_arm; 

    get_arm_left_ParamBody(Blist_arm_left, M_arm_left, T_base_left_arm); 

    return eod_robot_JacobianBody(Blist_arm_left, thetalist);
}

bool get_arm_left_ParamSpace(MatrixXd& Slist_arm_left, Matrix4d& M_arm_left, Matrix4d& T_base_left_arm)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget1 = ros::param::get(SLIST_ARM_LEFT, param_yaml);
        if(ifget1){
            for(int i = 0; i < 6; i++){
                std::stringstream S;
                S << "S" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    Slist_arm_left(j, i) = param_yaml[i][S.str()][j];
            }
        }

        bool ifget2 = ros::param::get(M_ARM_LEFT, param_yaml);
        if(ifget2){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    M_arm_left(i, j) = param_yaml[i][S.str()][j];
            }
        }

        bool ifget3 = ros::param::get(T_BASE_LEFT_ARM, param_yaml);
        if(ifget3){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    T_base_left_arm(i, j) = param_yaml[i][S.str()][j];
            }
        }

        return ifget1 && ifget2 && ifget3;
    }
    catch(...) {
        return false;
    }

}

bool get_arm_right_ParamSpace(MatrixXd& Slist_arm_right, Matrix4d& M_arm_right, Matrix4d& T_base_right_arm)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget1 = ros::param::get(SLIST_ARM_RIGHT, param_yaml);
        if(ifget1){
            for(int i = 0; i < 6; i++){
                std::stringstream S;
                S << "S" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    Slist_arm_right(j, i) = param_yaml[i][S.str()][j];
            }
        }

        bool ifget2 = ros::param::get(M_ARM_RIGHT, param_yaml);
        if(ifget2){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    M_arm_right(i, j) = param_yaml[i][S.str()][j];
            }
        }

        bool ifget3 = ros::param::get(T_BASE_RIGHT_ARM, param_yaml);
        if(ifget3){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    T_base_right_arm(i, j) = param_yaml[i][S.str()][j];
            } 
        }
       
        return ifget1 && ifget2 && ifget3;
    }
    catch(...) {
        return false;
    }

}

bool get_arm_left_ParamBody(MatrixXd& Blist_arm_left, Matrix4d& M_arm_left, Matrix4d& T_base_left_arm)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget1 = ros::param::get(BLIST_ARM_LEFT, param_yaml);
        if(ifget1){
            for(int i = 0; i < 6; i++){
                std::stringstream S;
                S << "S" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    Blist_arm_left(j, i) = param_yaml[i][S.str()][j];
            }
        }

        bool ifget2 = ros::param::get(M_ARM_LEFT, param_yaml);
        if(ifget2){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    M_arm_left(i, j) = param_yaml[i][S.str()][j];
            }
        }

        bool ifget3 = ros::param::get(T_BASE_LEFT_ARM, param_yaml);
        if(ifget3){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    T_base_left_arm(i, j) = param_yaml[i][S.str()][j];
            }
        }

        return ifget1 && ifget2 && ifget3;
    }
    catch(...) {
        return false;
    }

}

bool get_arm_right_ParamBody(MatrixXd& Blist_arm_right, Matrix4d& M_arm_right, Matrix4d& T_base_right_arm)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget1 = ros::param::get(BLIST_ARM_RIGHT, param_yaml);
        if(ifget1){
            for(int i = 0; i < 6; i++){
                std::stringstream S;
                S << "S" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    Blist_arm_right(j, i) = param_yaml[i][S.str()][j];
            }
        }

        bool ifget2 = ros::param::get(M_ARM_RIGHT, param_yaml);
        if(ifget2){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    M_arm_right(i, j) = param_yaml[i][S.str()][j];
            }
        }

        bool ifget3 = ros::param::get(T_BASE_RIGHT_ARM, param_yaml);
        if(ifget3){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    T_base_right_arm(i, j) = param_yaml[i][S.str()][j];
            } 
        }
       
        return ifget1 && ifget2 && ifget3;
    }
    catch(...) {
        return false;
    }
}

bool get_arm_right_joint_angle(VectorXd& angle_arm_right)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget = ros::param::get(ANGLE_ARM_RIGHT, param_yaml);
        if(ifget){
            for(int i =0; i < param_yaml.size(); i++)
                angle_arm_right[i] = param_yaml[i];
        }

        return ifget;
    }
    catch(...) {
        return false;
    }
}

bool get_arm_left_joint_angle(VectorXd& angle_arm_left)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget = ros::param::get(ANGLE_ARM_LEFT, param_yaml);
        if(ifget){
            for(int i =0; i < param_yaml.size(); i++)
                angle_arm_left[i] = param_yaml[i];
        }

        return ifget;
    }
    catch(...) {
        return false;
    }
}

bool get_arm_left_T_effector_camera(Matrix4d& T_effector_camera)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget = ros::param::get(T_LEFT_EFFECTOR_CAMERA, param_yaml);
        if(ifget){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    T_effector_camera(i, j) = param_yaml[i][S.str()][j];
            } 
        }
        return ifget;
    }
    catch(...) {
        return false;
    }
}

bool get_arm_right_T_effector_camera(Matrix4d& T_effector_camera)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget = ros::param::get(T_RIGHT_EFFECTOR_CAMERA, param_yaml);
        if(ifget){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    T_effector_camera(i, j) = param_yaml[i][S.str()][j];
            } 
        }
        return ifget;
    }
    catch(...) {
        return false;
    }    
}

bool get_gripper_finger1_position(double& gripper_finger1)
{
    try {
        bool ifget = ros::param::get(GRIPPER_FINGER1, gripper_finger1);
        return ifget;
    }
    catch(...) {
        return false;
    }
}

bool get_gripper_finger1_limit_max(double& gripper_finger1_limit_max)
{
    try {
        bool ifget = ros::param::get(GRIPPER_LIMIT_MAX, gripper_finger1_limit_max);
        return ifget;
    }
    catch(...) {
        return false;
    }
}

bool get_gripper_finger1_limit_min(double& gripper_finger1_limit_min)
{
    try {
        bool ifget = ros::param::get(GRIPPER_LIMIT_MIN, gripper_finger1_limit_min);
        return ifget;
    }
    catch(...) {
        return false;
    }
}

bool get_vecihle_odom(Matrix4d& vecihle_odom)
{
    try {
        XmlRpc::XmlRpcValue param_yaml;

        bool ifget = ros::param::get(VECIHLE_ODOM, param_yaml);
        if(ifget){
            for(int i = 0; i < 4; i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < param_yaml[i][S.str()].size(); j++)
                    vecihle_odom(i, j) = param_yaml[i][S.str()][j];
            }
        }

        return ifget;
    }
    catch(...) {
        return false;
    }
}

bool get_vecihle_velocity(double& linear, double& angular){
    try {
        bool ifget1 = ros::param::get(VECIHLE_LINEAR, linear);
        bool ifget2 = ros::param::get(VECIHLE_ANGULAR, angular);
        return ifget1 && ifget2;
    }
    catch(...) {
        return false;
    }
}

bool set_arm_right_joint_angle(VectorXd angle_arm_right)
{
    try {
        if(ros::param::has(ANGLE_ARM_RIGHT)){
            XmlRpc::XmlRpcValue param_yaml;

            for(int i =0; i < angle_arm_right.size(); i++)
                param_yaml[i] = angle_arm_right[i];

            ros::param::set(ANGLE_ARM_RIGHT, param_yaml);

            return true;
        }
        else
            return false;   
    }
    catch(...) {
        return false;
    }
}

bool set_arm_left_joint_angle(VectorXd angle_arm_left)
{
    try {
        if(ros::param::has(ANGLE_ARM_LEFT)){
            XmlRpc::XmlRpcValue param_yaml;

            for(int i =0; i < angle_arm_left.size(); i++)
                param_yaml[i] = angle_arm_left[i];

            ros::param::set(ANGLE_ARM_LEFT, param_yaml);

            return true;
        }
        else
            return false;
    }
    catch(...) {
        return false;
    }
}

bool set_gripper_finger1_position(double gripper_finger1)
{
    try {
        if(ros::param::has(GRIPPER_FINGER1)){
            double temp, limit_min, limit_max;

            ros::param::get(GRIPPER_LIMIT_MAX, limit_max);
            ros::param::get(GRIPPER_LIMIT_MIN, limit_min);

            temp = gripper_finger1 * (limit_max - limit_min) + limit_min;

            ros::param::set(GRIPPER_FINGER1, temp); 
            return true;
        }
        else
            return false;
    }
    catch(...) {
        return false;
    }
}

bool set_vecihle_odom(Matrix4d vecihle_odom)
{
    try {
        if(ros::param::has(VECIHLE_ODOM)){
            XmlRpc::XmlRpcValue param_yaml;

            for(int i = 0; i < vecihle_odom.rows(); i++){
                std::stringstream S;
                S << "row" << i+1;
                for(int j = 0; j < vecihle_odom.cols(); j++)
                    param_yaml[i][S.str()][j] = vecihle_odom(i, j);
            }

            ros::param::set(VECIHLE_ODOM, param_yaml);
            return true;
        }
        else
            return false;
    }
    catch(...) {
        return false;
    }
}

bool set_vecihle_velocity(double linear, double angular){
    try {
        if(ros::param::has(VECIHLE_LINEAR) && ros::param::has(VECIHLE_ANGULAR)){
            ros::param::set(VECIHLE_LINEAR, linear);
            ros::param::set(VECIHLE_ANGULAR, angular);
            return true;
        }
        else
            return false;
    }
    catch(...) {
        return false;
    }    
}

 
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd yamAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollchAngle(roll, Eigen::Vector3d::UnitX());
 
    Eigen::Quaterniond q = yamAngle * pitchAngle * rollchAngle;
    return q;
}

Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);

    return euler;
}

Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    return R;
}

Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    q.normalize();

    return q;
}

Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd yamAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollchAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = yamAngle * pitchAngle * rollchAngle;
    Eigen::Matrix3d R = q.matrix();

    return R;
}

Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R)
{
    Eigen::Matrix3d m;
    m = R;
    Eigen::Vector3d euler = m.eulerAngles(2, 1, 0);

    return euler;
}

void Matrix_T2tf_transform(Matrix4d T, geometry_msgs::TransformStamped& trans)
{
    Matrix3d R; Vector3d p;
    TransToRp(T, R, p);
    Matrix_R2tf_rotation(R, trans);
    Matrix_p2tf_translation(p, trans);
}

void Matrix_R2tf_rotation(Matrix3d R, geometry_msgs::TransformStamped& trans)
{
        Eigen::Quaterniond quater = rotationMatrix2Quaterniond(R);
        trans.transform.rotation.x = quater.x();
        trans.transform.rotation.y = quater.y();
        trans.transform.rotation.z = quater.z();
        trans.transform.rotation.w = quater.w();
}

void Matrix_p2tf_translation(Vector3d p, geometry_msgs::TransformStamped& trans)
{
        trans.transform.translation.x = p[0];
        trans.transform.translation.y = p[1];
        trans.transform.translation.z = p[2];
}

void tf_transform2Matrix_T(geometry_msgs::TransformStamped trans, Matrix4d& T)
{
    Matrix3d R; Vector3d p;
    tf_rotation2Matrix_R(trans, R);
    tf_translation2Matrix_p(trans, p);
    RpToTrans(R, p, T);
}

void tf_rotation2Matrix_R(geometry_msgs::TransformStamped trans, Matrix3d& R)
{
    R = Quaternion2RotationMatrix(trans.transform.rotation.x, 
                                    trans.transform.rotation.y, 
                                    trans.transform.rotation.z, 
                                    trans.transform.rotation.w);
}

void tf_translation2Matrix_p(geometry_msgs::TransformStamped trans, Vector3d& p)
{
        p[0] = trans.transform.translation.x;
        p[1] = trans.transform.translation.y;
        p[2] = trans.transform.translation.z;
}

