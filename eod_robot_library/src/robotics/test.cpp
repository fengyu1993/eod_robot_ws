#include <iostream>
#include "modern_robotics_lib.h"
#include "eod_robotics_lib.h"
#include "math.h"


using namespace std; 

int main(int argc, char** argv)
{ 
    MatrixXd R_Slist_T(6,6);
    R_Slist_T << 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.4247, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.8091, 0.0000, 0.0000, -1.0000, -0.1110, 0.8091, 0.0000, 0.0000, 1.0000, 0.0000, 0.0296, 0.0000, 0.8091;
    MatrixXd R_Slist = R_Slist_T.transpose();

    MatrixXd L_Slist_T(6,6);
    L_Slist_T <<  0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0703, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0703, 0.0000, 0.2459, 0.0000, 1.0000, 0.0000, -0.0703, 0.0000, 0.5013, 0.0000, 0.0000, -1.0000, -0.0786, 0.5013, 0.0000, 0.0000, 1.0000, 0.0000, 0.0398, 0.0000, 0.5013;
    MatrixXd L_Slist = L_Slist_T.transpose();

    Matrix4d R_M(4,4);
    R_M << -1.0000, 0, 0, 0.8091, 0, 0, 1.0000, 0.3790, 0, 1.0000, 0, -0.0296, 0, 0, 0, 1.0000;

    Matrix4d L_M(4,4);
    L_M << -1.0000, 0.0000, 0.0000, 0.5013, 0.0000, 0.0000, 1.0000, 0.2510, 0.0000, 1.0000, 0.0000, -0.0398, 0.0000, 0.0000, 0.0000, 1.0000;

    Matrix4d T_base_right_arm(4,4);
    T_base_right_arm << 1.0000, 0.0000, 0.0000, -0.0500, 0.0000, 0.4229, -0.9062, -0.1373, \
                        0.0000, 0.9062, 0.4229, -0.0906, 0.0000, 0.0000, 0.0000, 1.0000;

    Matrix4d T_base_left_arm(4,4);
    T_base_left_arm << -1.0000, 0.0000, 0.0000, 0.0500, 0.0000, -0.4229, 0.9062, 0.1373, 0.0000, \
                        0.9062, 0.4229, -0.0906, 0.0000, 0.0000, 0.0000, 1.0000;

    VectorXd thetalist_right_arm(6),  thetalist_result_right_arm(6), \
                thetalist_left_arm(6),  thetalist_result_left_arm(6);

    // thetalist_right_arm << 4.778741,  0.001709, 3.867487, -1.588801, -5.517637, 1.247915;
    // thetalist_right_arm << 0,  -0.4363, 0.7854, -1.5708, -2.7053, 0; // work pose
    thetalist_right_arm << 0,  0, 0,  0, 0, 0; // zero pose
    // thetalist_left_arm << 4.778741,  0.001709, 3.867487, -1.588801, -5.517637, 1.247915;
    // thetalist_left_arm << 0,  -2.7053, -0.7854, -1.5708, 2.7053, 0; // work pose
    thetalist_left_arm << 0,  0, 0,  0, 0, 0; // zero pose

    Matrix4d T_eef_right_arm = T_base_right_arm * FKinSpace(R_M, R_Slist, thetalist_right_arm);
    Matrix4d T_eef_left_arm = T_base_left_arm * FKinSpace(L_M, L_Slist, thetalist_left_arm);

    std::cout << "T_eef_right_arm = "<< std::endl << T_eef_right_arm << std::endl;
    std::cout << "T_eef_left_arm = "<< std::endl << T_eef_left_arm << std::endl;

    Matrix3d R_right_arm, R_left_arm;  Vector3d p_right_arm, p_left_arm;

    TransToRp(T_eef_right_arm, R_right_arm, p_right_arm);

    TransToRp(T_eef_left_arm, R_left_arm, p_left_arm);

        std::cout << "p_ave = "<< std::endl << (p_left_arm + p_right_arm) / 2 << std::endl;

    Quaterniond q_right_arm = rotationMatrix2Quaterniond(R_right_arm);

    Quaterniond q_left_arm = rotationMatrix2Quaterniond(R_left_arm);
    
    std::cout << "q_right_arm = "<< std::endl\
              << q_right_arm.w() << ", "<< q_right_arm.x() << ", "<< q_right_arm.y() << ", "<< q_right_arm.z() << std::endl;

    std::cout << "q_left_arm = "<< std::endl\
              << q_left_arm.w() << ", "<< q_left_arm.x() << ", "<< q_left_arm.y() << ", "<< q_left_arm.z() << std::endl;

    bool suc_right_arm = eod_robot_IKinSpace_POE(R_Slist, R_M, T_eef_right_arm, thetalist_right_arm, 0, thetalist_result_right_arm, T_base_right_arm);
    bool suc_left_arm = eod_robot_IKinSpace_POE(L_Slist, L_M, T_eef_left_arm, thetalist_left_arm, 0, thetalist_result_left_arm, T_base_left_arm);

    std::cout << "thetalist_right_arm = "<< std::endl << thetalist_right_arm << std::endl;
    std::cout << "thetalist_right_arm_error = "<< std::endl << thetalist_result_right_arm - thetalist_right_arm<< std::endl;
    std::cout << "thetalist_left_arm = "<< std::endl << thetalist_left_arm << std::endl;
    std::cout << "thetalist_right_arm_error = "<< std::endl << thetalist_result_left_arm - thetalist_left_arm<< std::endl; 
   
    MatrixXd jacobian_space_right_arm = Adjoint(T_base_right_arm) *  JacobianSpace(R_Slist, thetalist_right_arm);
    MatrixXd jacobian_space_left_arm = Adjoint(T_base_left_arm) *  JacobianSpace(L_Slist, thetalist_left_arm);

    std::cout << "jacobian_space_right_arm = "<< std::endl << jacobian_space_right_arm << std::endl;
    std::cout << "jacobian_space_left_arm = "<< std::endl << jacobian_space_left_arm << std::endl;

    Eigen::Matrix3d R_1 = euler2RotationMatrix(-1.5708, -1.4008, 0);
    Eigen::Matrix3d R_2 =  euler2RotationMatrix(0, 0, 3.14159);
    Eigen::Matrix3d R_3 =  R_1 * R_2;
    Eigen::Vector3d rpy = RotationMatrix2euler(R_3);


    std::cout << "R_1 = " << std::endl << R_1 << std::endl ;
    std::cout << "R_2 = " << std::endl << R_2 << std::endl ;
    std::cout << "R_3 = " << std::endl << R_3 << std::endl ;
    std::cout << "rpy = " << std::endl << rpy << std::endl ;

}

    /* left arm*/
    // Vector3d l_p_b_c; l_p_b_c << -0.39555, 0.048624, 0.044706;
    // Matrix3d l_R_b_c = Quaternion2RotationMatrix(0.154052, -0.152216, 0.68617, 0.694456);
    // Matrix4d l_T_b_c;  RpToTrans(l_R_b_c, l_p_b_c, l_T_b_c);

    // Vector3d l_p_b_e; l_p_b_e << -0.4513, -0.00491466, 0.120025;
    // Matrix3d l_R_b_e(3,3); l_R_b_e << 1, 0, 0, 0, 0.9062, -0.4229, 0, 0.4229, 0.9062;
    // Matrix4d l_T_b_e;  RpToTrans(l_R_b_e, l_p_b_e, l_T_b_e);

    // Matrix4d l_T_e_c = TransInv(l_T_b_e) * l_T_b_c;

    // Matrix4d l_T_b_e_2(4,4);  
    // l_T_b_e_2 <<  0.00194287,0.999968,0.00780416,-0.0302846,0.000283611,-0.00780489,0.999991,0.376542,1.00002,-0.00194064,-0.000298766,0.550085,0,0,0,1;
    // Vector3d l_p_b_c_2; l_p_b_c_2 << -0.0142292, 0.285539, 0.60571;
    // Matrix3d l_R_b_c_2 = Quaternion2RotationMatrix(-0.707228, -0.000796242, -0.00631859, 0.706957);
    // Matrix4d l_T_b_c_2;  RpToTrans(l_R_b_c_2, l_p_b_c_2, l_T_b_c_2);

    // Matrix4d l_T_e_c_2 = TransInv(l_T_b_e_2) * l_T_b_c_2;

    // cout << "l_T_e_c: " << endl << l_T_e_c << endl;
    // cout << "l_T_e_c_2: " << endl << l_T_e_c_2 << endl;
    // cout << "l_T_e_c_ave: " << endl << (l_T_e_c + l_T_e_c_2) / 2 << endl;

    /* right arm*/
    // Vector3d r_p_b_c; r_p_b_c << 0.703928, -0.0454149, 0.0789382;
    // Matrix3d r_R_b_c = Quaternion2RotationMatrix(-0.153962, -0.152308, -0.686586, 0.694044);
    // Matrix4d r_T_b_c;  RpToTrans(r_R_b_c, r_p_b_c, r_T_b_c);
    // Vector3d r_p_b_c_2; r_p_b_c_2 << -0.183374, -0.310726, 0.915141;
    // Matrix3d r_R_b_c_2 = Quaternion2RotationMatrix(0.00532469, 0.715353, -0.698661, 0.0107353);
    // Matrix4d r_T_b_c_2;  RpToTrans(r_R_b_c_2, r_p_b_c_2, r_T_b_c_2);

    // Matrix4d r_T_b_e(4,4);  
    // r_T_b_e << -1, 0,  0, 0.7591, 0, -0.9062, 0.4229, 0.0498026, 0, 0.4229, 0.9062, 0.240332, 0, 0, 0, 1;
    // Matrix4d r_T_b_e_2(4,4);  
    // r_T_b_e_2 <<   -0.0118243, -0.999899, 0.00791875, -0.163403, -0.0235799, -0.00763844, -0.999714, -0.49562, 0.999673, -0.0120076, -0.0234872, 0.855644, 0, 0, 0, 1;

    // Matrix4d r_T_e_c = TransInv(r_T_b_e) * r_T_b_c;
    // Matrix4d r_T_e_c_2 = TransInv(r_T_b_e_2) * r_T_b_c_2;

    // cout << "r_T_e_c: " << endl << r_T_e_c << endl;
    // cout << "r_T_e_c_2: " << endl << r_T_e_c_2 << endl;
    // cout << "r_T_e_c_ave: " << endl << (r_T_e_c + r_T_e_c_2) / 2 << endl;

        // Matrix4d T(4,4);
    // T << 0.0795304,  -0.986297,  -0.144544,   0.767886,-0.99676, -0.0769389, -0.0234398,  -0.233008, 0.0119975,    0.14594,  -0.989221,  -0.216464,0, 0, 0, 1;
    
    // VectorXd thetalist0(6), thetalist(6);
    // thetalist0 << 0.0022273, -0.310563,  0.570518, -1.47717, -2.70454, 0.243157;


    // double eomg = 0.01, ev = 0.001;

    // bool suc = eod_robot_IKinSpace(R_Slist, R_M, T, thetalist0, 0.01, 0.001, thetalist, T_base_R);

    // Matrix4d T_check = eod_robot_FKinSpace(R_M, R_Slist, thetalist, T_base_R);

    // cout << "suc = " << suc << endl;
    // cout << "thetalist0 = " << thetalist0 << endl;
    // cout << "thetalist = " << thetalist << endl;
    // cout << "T_error: " << T_check - T << endl;

    // Matrix3d R(3,3);
    // R << 1.00002,  6.32013e-06,  2.22446e-06, 6.53443e-06, 1,  -2.28938e-06, -3.48345e-07,  -3.59979e-06, 1.00002;
    // cout << R << endl;
    // cout << MatrixLog3(R) << endl;


    // Matrix4d T_norm;
    // Vector3d V_norm;
    // V_norm << 1,1,1;
    // T_norm = Matrix4d::Identity();

    // cout << "T_norm: " << T_norm << endl; 
    // cout << "T_norm.norm(): " << T_norm.maxCoeff() << endl; 
    
    // cout << "V_norm: " << T_norm << endl; 
    // cout << "V_norm.norm(): " << V_norm.norm() << endl; 

    // double roll, pitch, yaw;
    // roll = 1.2; pitch = -0.3; yaw = 2.1;

    // Eigen::Quaterniond q = euler2Quaternion(roll, pitch, yaw);
    // cout << "Quaterniond: " << q.x() << ", " << q.y() << ", " << q.z() << ", " <<  q.w() << endl;

    // Eigen::Vector3d rpy = Quaterniond2Euler(q.x(), q.y(), q.z(), q.w());
    // cout << "Euler: " << rpy << endl;

    // Eigen::Matrix3d R = Quaternion2RotationMatrix(q.x(), q.y(), q.z(), q.w());
    // cout << "R: " << R << endl;

    // Eigen::Quaterniond q2 = rotationMatrix2Quaterniond(R);
    // cout << "Quaterniond 2: " << q2.x() << ", " << q2.y() << ", " << q2.z() << ", " <<  q2.w() << endl;

    // Eigen::Matrix3d R2 = euler2RotationMatrix(roll, pitch, yaw);
    // cout << "R 2: " << R2 << endl;

    // Eigen::Vector3d rpy2 = RotationMatrix2euler(R2);
    // cout << "Euler 2: " << rpy2 << endl;

    // Eigen::Matrix3d R3 = euler2RotationMatrix(rpy2[2], rpy2[1], rpy2[0]);
    // cout << "R 3: " << R3 << endl;

        // VectorXd thetalist(6,1); thetalist << 1.25, 2.1, 0.5, 0.2, 1.2, 0.9;
    
    

    // MatrixXd L_Blist(6,6);
    // for (int i = 0; i < 6; i++){
    //     L_Blist.col(i) = Adjoint(TransInv(L_M)) * L_Slist.col(i);
    // }

    // Matrix4d L_T_Space = eod_robot_FKinSpace(L_M, L_Slist, thetalist, T_base_L);
    // Matrix4d L_T_Body = eod_robot_FKinBody(L_M, L_Blist, thetalist, T_base_L);

    // cout << "L_T_Space:" << endl << L_T_Space << endl;
    // cout << "L_T_Body:" << endl << L_T_Body << endl;
    // cout << "L_Slist':"  << endl << L_Slist.transpose() << endl ;
    // cout << "L_Blist':"  << endl << L_Blist.transpose() << endl ; 

    // MatrixXd R_Slist_T(6,6);
    // R_Slist_T << 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.4247, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.8091, 0.0000, 0.0000, -1.0000, -0.1110, 0.8091, 0.0000, 0.0000, 1.0000, 0.0000, 0.0296, 0.0000, 0.8091;
    // MatrixXd R_Slist = R_Slist_T.transpose();

    // Matrix4d R_M(4,4);
    // R_M << -1.0000, 0, 0, 0.8091, 0, 0, 1.0000, 0.3790, 0, 1.0000, 0, -0.0296, 0, 0, 0, 1.0000;

    // Matrix4d T_base_R(4,4);
    // T_base_R <<  1.0000, 0,  0, -0.0500, 0, 0.4229, -0.9062, -0.1373, 0, 0.9062, 0.4229, -0.0906, 0, 0, 0, 1.0000;

    // MatrixXd R_Blist(6,6);
    // for (int i = 0; i < 6; i++){
    //     R_Blist.col(i) = Adjoint(TransInv(R_M)) * R_Slist.col(i);
    // }

    // Matrix4d R_T_Space = eod_robot_FKinSpace(R_M, R_Slist,thetalist, T_base_R);
    // Matrix4d R_T_Body = eod_robot_FKinBody(R_M, R_Blist,thetalist, T_base_R);

    // cout << "R_T_Space:" << endl << R_T_Space << endl;
    // cout << "R_T_Body:" << endl << R_T_Body << endl;
    // cout << "R_Slist':"  << endl << R_Slist.transpose() << endl ;
    // cout << "R_Blist':"  << endl << R_Blist.transpose() << endl ;

        // double y1 = 0, p1 = 0.192, r1 = 0;

    // Eigen::Matrix3d R1 = euler2RotationMatrix(r1, p1, y1);

    // double y2 = 0, p2 = 1.5708, r2 = 0;

    // Eigen::Matrix3d R2 = euler2RotationMatrix(r2, p2, y2);

    // Eigen::Matrix3d R = R2 * R1;

    // Eigen::Vector3d  rpy = RotationMatrix2euler(R);

    // double y3 = -1.5708, p3 = 0.00, r3 = 0.0;

    // Eigen::Matrix3d R3 = euler2RotationMatrix(r3, p3, y3);

    // R = R3 * R;

    // rpy = RotationMatrix2euler(R);

    // cout << "r = " << rpy[2]<< " p = " << rpy[1] << " y = " << rpy[0] << endl;

    // cout << rpy[0] << " " << rpy[1] << " " << rpy[2] << endl;

    // Eigen::Vector3d rpy = RotationMatrix2euler(R);
    
    // Eigen::Matrix3d R2 = euler2RotationMatrix(rpy[0], rpy[1], rpy[2]);

    // cout << "R: " << R << endl;
    // cout << "Euler: " << rpy << endl;
    // cout << "R2: " << R2 << endl;

    //     MatrixXd R_Slist_T(6,6);
    // R_Slist_T << 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.4247, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.8091, 0.0000, 0.0000, -1.0000, -0.1110, 0.8091, 0.0000, 0.0000, 1.0000, 0.0000, 0.0296, 0.0000, 0.8091;
    // MatrixXd R_Slist = R_Slist_T.transpose();

    // Matrix4d R_M(4,4);
    // R_M << -1.0000, 0, 0, 0.8091, 0, 0, 1.0000, 0.3790, 0, 1.0000, 0, -0.0296, 0, 0, 0, 1.0000;

    // VectorXd thetalist(6), thetalist0(6), thetalist_result(6);

    // thetalist << M_PI/6, -M_PI/2-M_PI/15, -M_PI/3, -M_PI/2-M_PI/5, -M_PI/6, 0;
    // // thetalist  = 2*M_PI*MatrixXd::Random(6,1) - MatrixXd::Ones(6,1) * M_PI;

    // Matrix4d T_eef = FKinSpace(R_M, R_Slist, thetalist);
    // // T_eef(0,3) = 2;

    // int method = 2;

    // bool flag = IKinSpace_POE(R_Slist, R_M, T_eef, thetalist, method, thetalist_result);

    // Matrix4d T_result = FKinSpace(R_M, R_Slist, thetalist_result);

    // cout << "flag = " << endl << flag << endl;

    // cout << "thetalist = " << endl << thetalist << endl;

    // cout << "thetalist_result = " << endl << thetalist_result << endl;

    // cout << "T_err = " << endl << T_result - T_eef << endl;


    //     MatrixXd L_Slist_T(6,6);
    // L_Slist_T <<  0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0703, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0703, 0.0000, 0.2459, 0.0000, 1.0000, 0.0000, -0.0703, 0.0000, 0.5013, 0.0000, 0.0000, -1.0000, -0.0786, 0.5013, 0.0000, 0.0000, 1.0000, 0.0000, 0.0398, 0.0000, 0.5013;
    // MatrixXd L_Slist = L_Slist_T.transpose();

    // Matrix4d L_M(4,4);
    // L_M << -1.0000, 0.0000, 0.0000, 0.5013, 0.0000, 0.0000, 1.0000, 0.2510, 0.0000, 1.0000, 0.0000, -0.0398, 0.0000, 0.0000, 0.0000, 1.0000;


    // VectorXd thetalist(6), thetalist0(6), thetalist_result(6);

    // thetalist << M_PI/6, -M_PI/2-M_PI/15, -M_PI/3, -M_PI/2-M_PI/5, -M_PI/6, 0;
    // // thetalist  = 2*M_PI*MatrixXd::Random(6,1) - MatrixXd::Ones(6,1) * M_PI;

    // Matrix4d T_eef = FKinSpace(L_M, L_Slist, thetalist);

    // int method = 1;

    // bool flag = IKinSpace_POE(L_Slist, L_M, T_eef, thetalist, method, thetalist_result);

    // Matrix4d T_result = FKinSpace(L_M, L_Slist, thetalist_result);

    // cout << "flag = " << endl << flag << endl;

    // cout << "thetalist = " << endl << thetalist << endl;

    // cout << "thetalist_result = " << endl << thetalist_result << endl;

    // cout << "T_err = " << endl << T_result - T_eef << endl;


    // MatrixXd jacobian_body = eod_robot_JacobianBody(R_Blist, thetalist);

    // std::cout << "jacobian_body = "<< std::endl << jacobian_body << std::endl;


    // MatrixXd R_Blist = SlistToBlist(R_Slist, R_M);