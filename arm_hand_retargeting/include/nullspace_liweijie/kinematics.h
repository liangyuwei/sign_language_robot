#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#include "config_nullspace.h"
#include "transform_matrix_helper.h"
#include "static_matrix_config.h"
// using namespace cfg;

namespace kinematics {
    struct Result
    {
        Vector3d l_elbow_pos;
        Vector3d r_elbow_pos;
        Vector3d l_wrist_pos;
        Vector3d r_wrist_pos;
        Quaterniond l_elbow_quat;
        Quaterniond r_elbow_quat;
        Quaterniond l_wrist_quat;
        Quaterniond r_wrist_quat;
    };
    Result yumi_forward_kinematics(Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle);

    void Result2x(Result result, Eigen::Matrix<double,6,1>&  x_left, Eigen::Matrix<double,6,1>&  x_right);

    Result yumi_forward_kinematics(Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle) {
            // Create dynamic rotation matrices
            AngleAxis<double> left_rv1(joint_angle[0],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> left_rot1 = left_rv1.toRotationMatrix(); 
            AngleAxis<double> left_rv2(joint_angle[1],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> left_rot2 = left_rv2.toRotationMatrix();
            AngleAxis<double> left_rv3(joint_angle[2],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> left_rot3 = left_rv3.toRotationMatrix();
            AngleAxis<double> left_rv4(joint_angle[3],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> left_rot4 = left_rv4.toRotationMatrix();
            AngleAxis<double> left_rv5(joint_angle[4],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> left_rot5 = left_rv5.toRotationMatrix();
            AngleAxis<double> left_rv6(joint_angle[5],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> left_rot6 = left_rv6.toRotationMatrix();
            AngleAxis<double> left_rv7(joint_angle[6],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> left_rot7 = left_rv7.toRotationMatrix();
            AngleAxis<double> right_rv1(joint_angle[7],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> right_rot1 = right_rv1.toRotationMatrix();
            AngleAxis<double> right_rv2(joint_angle[8],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> right_rot2 = right_rv2.toRotationMatrix();
            AngleAxis<double> right_rv3(joint_angle[9],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> right_rot3 = right_rv3.toRotationMatrix();
            AngleAxis<double> right_rv4(joint_angle[10],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> right_rot4 = right_rv4.toRotationMatrix();
            AngleAxis<double> right_rv5(joint_angle[11],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> right_rot5 = right_rv5.toRotationMatrix();
            AngleAxis<double> right_rv6(joint_angle[12],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> right_rot6 = right_rv6.toRotationMatrix();
            AngleAxis<double> right_rv7(joint_angle[13],Eigen::Matrix<double,3,1>(double(0),double(0),double(1)));  Eigen::Matrix<double,3,3> right_rot7 = right_rv7.toRotationMatrix();
            // Forward Kinematics
            Eigen::Matrix<double,3,1> l_elbow_pos_cur = Rl1*left_rot1*(Rl2*left_rot2*(Rl3*left_rot3*Tl4+Tl3)+Tl2)+Tl1;
            Eigen::Matrix<double,3,1> r_elbow_pos_cur = Rr1*right_rot1*(Rr2*right_rot2*(Rr3*right_rot3*Tr4+Tr3)+Tr2)+Tr1;
            Eigen::Matrix<double,3,1> l_wrist_pos_cur = Rl1*left_rot1*(Rl2*left_rot2*(Rl3*left_rot3*(Rl4*left_rot4*(Rl5*left_rot5*(Rl6*left_rot6*Tl7+Tl6)+Tl5)+Tl4)+Tl3)+Tl2)+Tl1;
            Eigen::Matrix<double,3,1> r_wrist_pos_cur = Rr1*right_rot1*(Rr2*right_rot2*(Rr3*right_rot3*(Rr4*right_rot4*(Rr5*right_rot5*(Rr6*right_rot6*Tr7+Tr6)+Tr5)+Tr4)+Tr3)+Tr2)+Tr1;
            Eigen::Matrix<double,3,3> l_elbow_rot_cur = Rl1*left_rot1*Rl2*left_rot2*Rl3*left_rot3*Rl4*left_rot4;
            Eigen::Matrix<double,3,3> r_elbow_rot_cur = Rr1*right_rot1*Rr2*right_rot2*Rr3*right_rot3*Rr4*right_rot4;
            Eigen::Matrix<double,3,3> l_wrist_rot_cur = l_elbow_rot_cur*Rl5*left_rot5*Rl6*left_rot6*Rl7*left_rot7;
            Eigen::Matrix<double,3,3> r_wrist_rot_cur = r_elbow_rot_cur*Rr5*right_rot5*Rr6*right_rot6*Rr7*right_rot7;
        Result res;
        res.l_elbow_pos = l_elbow_pos_cur;
        res.r_elbow_pos = r_elbow_pos_cur;
        res.l_wrist_pos = l_wrist_pos_cur;
        res.r_wrist_pos = r_wrist_pos_cur;
        res.l_elbow_quat = Quaternion<double>(l_elbow_rot_cur);
        res.r_elbow_quat = Quaternion<double>(r_elbow_rot_cur);
        res.l_wrist_quat = Quaternion<double>(l_wrist_rot_cur);
        res.r_wrist_quat = Quaternion<double>(r_wrist_rot_cur);
        return res;
    }

    void Result2x(const Result result, Eigen::Matrix<double,WRIST_DOF,1>&  x_left, Eigen::Matrix<double,WRIST_DOF,1>&  x_right) {
        Vector3d l_wrist_pos = result.l_wrist_pos;
        Vector3d r_wrist_pos = result.r_wrist_pos;
        Quaterniond l_wrist_quat = result.l_wrist_quat;
        Quaterniond r_wrist_quat = result.r_wrist_quat;
        x_left.block(0,0,3,1) = l_wrist_pos;
        x_left.block(3,0,3,1) = l_wrist_quat.matrix().eulerAngles(0,1,2);
        // x_left.block(3,0,3,1) = Eigen::Matrix<double,3,1>::Zero();
        x_right.block(0,0,3,1) = r_wrist_pos;
        x_right.block(3,0,3,1) = r_wrist_quat.matrix().eulerAngles(0,1,2);
        // x_right.block(3,0,3,1) = Eigen::Matrix<double,3,1>::Zero();
    }

     void Result2xe(const Result result, Eigen::Matrix<double,ELBOW_DOF,1>&  x_left, Eigen::Matrix<double,ELBOW_DOF,1>&  x_right) {
        Vector3d l_elbow_pos = result.l_elbow_pos;
        Vector3d r_elbow_pos = result.r_elbow_pos;
        x_left.block(0,0,3,1) = l_elbow_pos;
        x_right.block(0,0,3,1) = r_elbow_pos;
    }

}

#endif