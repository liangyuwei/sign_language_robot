#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#include "config.h"
#include "transform_matrix_helper.h"
#include "static_matrix_config.h"
using namespace cfg;

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
    Eigen::AngleAxisd left_rv1(joint_angle[0],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d left_rot1 = left_rv1.toRotationMatrix(); 
    Eigen::AngleAxisd left_rv2(joint_angle[1],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d left_rot2 = left_rv2.toRotationMatrix();
    Eigen::AngleAxisd left_rv3(joint_angle[2],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d left_rot3 = left_rv3.toRotationMatrix();
    Eigen::AngleAxisd left_rv4(joint_angle[3],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d left_rot4 = left_rv4.toRotationMatrix();
    Eigen::AngleAxisd left_rv5(joint_angle[4],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d left_rot5 = left_rv5.toRotationMatrix();
    Eigen::AngleAxisd left_rv6(joint_angle[5],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d left_rot6 = left_rv6.toRotationMatrix();
    Eigen::AngleAxisd left_rv7(joint_angle[6],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d left_rot7 = left_rv7.toRotationMatrix();
    Eigen::AngleAxisd right_rv1(joint_angle[7],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d right_rot1 = right_rv1.toRotationMatrix();
    Eigen::AngleAxisd right_rv2(joint_angle[8],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d right_rot2 = right_rv2.toRotationMatrix();
    Eigen::AngleAxisd right_rv3(joint_angle[9],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d right_rot3 = right_rv3.toRotationMatrix();
    Eigen::AngleAxisd right_rv4(joint_angle[10],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d right_rot4 = right_rv4.toRotationMatrix();
    Eigen::AngleAxisd right_rv5(joint_angle[11],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d right_rot5 = right_rv5.toRotationMatrix();
    Eigen::AngleAxisd right_rv6(joint_angle[12],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d right_rot6 = right_rv6.toRotationMatrix();
    Eigen::AngleAxisd right_rv7(joint_angle[13],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d right_rot7 = right_rv7.toRotationMatrix();
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
    res.l_elbow_quat = Eigen::Quaternion<double>(l_elbow_rot_cur);
    res.r_elbow_quat = Eigen::Quaternion<double>(r_elbow_rot_cur);
    res.l_wrist_quat = Eigen::Quaternion<double>(l_wrist_rot_cur);
    res.r_wrist_quat = Eigen::Quaternion<double>(r_wrist_rot_cur);
    return res;
}

Eigen::Vector3d get_pos_of_specific_link(Eigen::Matrix<double,cfg::NUM_OF_JOINTS,1> joint_angle,
        std::string joint_name) {
    // Create dynamic rotation matrices
    Eigen::AngleAxisd left_rv1(joint_angle[0],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l1 = left_rv1.toRotationMatrix(); 
    Eigen::AngleAxisd left_rv2(joint_angle[1],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l2 = left_rv2.toRotationMatrix();
    Eigen::AngleAxisd left_rv3(joint_angle[2],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l3 = left_rv3.toRotationMatrix();
    Eigen::AngleAxisd left_rv4(joint_angle[3],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l4 = left_rv4.toRotationMatrix();
    Eigen::AngleAxisd left_rv5(joint_angle[4],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l5 = left_rv5.toRotationMatrix();
    Eigen::AngleAxisd left_rv6(joint_angle[5],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l6 = left_rv6.toRotationMatrix();
    Eigen::AngleAxisd left_rv7(joint_angle[6],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l7 = left_rv7.toRotationMatrix();
    Eigen::AngleAxisd right_rv1(joint_angle[7],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r1 = right_rv1.toRotationMatrix();
    Eigen::AngleAxisd right_rv2(joint_angle[8],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r2 = right_rv2.toRotationMatrix();
    Eigen::AngleAxisd right_rv3(joint_angle[9],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r3 = right_rv3.toRotationMatrix();
    Eigen::AngleAxisd right_rv4(joint_angle[10],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r4 = right_rv4.toRotationMatrix();
    Eigen::AngleAxisd right_rv5(joint_angle[11],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r5 = right_rv5.toRotationMatrix();
    Eigen::AngleAxisd right_rv6(joint_angle[12],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r6 = right_rv6.toRotationMatrix();
    Eigen::AngleAxisd right_rv7(joint_angle[13],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r7 = right_rv7.toRotationMatrix();
    // Forward kinematics
    Eigen::Vector3d pos;
    if (joint_name=="yumi_link_7_l") {
        pos = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Tl6 +
                Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*Rq_l2*Tl3 + Rl1*Rq_l1*Tl2 + Tl1;
    }
    else if (joint_name=="yumi_link_6_l") {
        pos = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Tl6 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 
                                + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*Rq_l2*Tl3 + Rl1*Rq_l1*Tl2 + Tl1;
    }
    else if (joint_name=="yumi_link_5_l") {
        pos = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 
                    + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*Rq_l2*Tl3 + Rl1*Rq_l1*Tl2 + Tl1;
    }
    else if (joint_name=="yumi_link_4_l") {
        pos = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*Rq_l2*Tl3 + Rl1*Rq_l1*Tl2 + Tl1;
    }
    else if (joint_name=="yumi_link_3_l") {
        pos = Rl1*Rq_l1*Rl2*Rq_l2*Tl3 + Rl1*Rq_l1*Tl2 + Tl1;
    }
    else if (joint_name=="yumi_link_2_l") {
        pos = Rl1*Rq_l1*Tl2 + Tl1;
    }
    else if (joint_name=="yumi_link_1_l") {
        pos = Tl1;
    }

    else if (joint_name=="yumi_link_7_r") {
        pos = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Tr6 +
                Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*Rq_r2*Tr3 + Rr1*Rq_r1*Tr2 + Tr1;
    }
    else if (joint_name=="yumi_link_6_r") {
        pos = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Tr6 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 
                                + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*Rq_r2*Tr3 + Rr1*Rq_r1*Tr2 + Tr1;
    }
    else if (joint_name=="yumi_link_5_r") {
        pos = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 
                    + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*Rq_r2*Tr3 + Rr1*Rq_r1*Tr2 + Tr1;
    }
    else if (joint_name=="yumi_link_4_r") {
        pos = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*Rq_r2*Tr3 + Rr1*Rq_r1*Tr2 + Tr1;
    }
    else if (joint_name=="yumi_link_3_r") {
        pos = Rr1*Rq_r1*Rr2*Rq_r2*Tr3 + Rr1*Rq_r1*Tr2 + Tr1;
    }
    else if (joint_name=="yumi_link_2_r") {
        pos = Rr1*Rq_r1*Tr2 + Tr1;
    }
    else if (joint_name=="yumi_link_1_r") {
        pos = Tr1;
    }
    return pos;
}

Eigen::Quaterniond get_ori_of_specific_link(Eigen::Matrix<double,cfg::NUM_OF_JOINTS,1> joint_angle,
        std::string joint_name) {
    // Create dynamic rotation matrices
    Eigen::AngleAxisd left_rv1(joint_angle[0],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l1 = left_rv1.toRotationMatrix(); 
    Eigen::AngleAxisd left_rv2(joint_angle[1],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l2 = left_rv2.toRotationMatrix();
    Eigen::AngleAxisd left_rv3(joint_angle[2],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l3 = left_rv3.toRotationMatrix();
    Eigen::AngleAxisd left_rv4(joint_angle[3],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l4 = left_rv4.toRotationMatrix();
    Eigen::AngleAxisd left_rv5(joint_angle[4],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l5 = left_rv5.toRotationMatrix();
    Eigen::AngleAxisd left_rv6(joint_angle[5],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l6 = left_rv6.toRotationMatrix();
    Eigen::AngleAxisd left_rv7(joint_angle[6],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_l7 = left_rv7.toRotationMatrix();
    Eigen::AngleAxisd right_rv1(joint_angle[7],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r1 = right_rv1.toRotationMatrix();
    Eigen::AngleAxisd right_rv2(joint_angle[8],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r2 = right_rv2.toRotationMatrix();
    Eigen::AngleAxisd right_rv3(joint_angle[9],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r3 = right_rv3.toRotationMatrix();
    Eigen::AngleAxisd right_rv4(joint_angle[10],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r4 = right_rv4.toRotationMatrix();
    Eigen::AngleAxisd right_rv5(joint_angle[11],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r5 = right_rv5.toRotationMatrix();
    Eigen::AngleAxisd right_rv6(joint_angle[12],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r6 = right_rv6.toRotationMatrix();
    Eigen::AngleAxisd right_rv7(joint_angle[13],Eigen::Vector3d(0,0,1));  Eigen::Matrix3d Rq_r7 = right_rv7.toRotationMatrix();
    // Forward kinematics
    Eigen::Matrix3d rot;
    if (joint_name=="yumi_link_7_l") {
        rot = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Rl7*Rq_l7;
    }
    else if (joint_name=="yumi_link_6_l") {
        rot = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6;
    }
    else if (joint_name=="yumi_link_5_l") {
        rot = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5;
    }
    else if (joint_name=="yumi_link_4_l") {
        rot = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3;
    }
    else if (joint_name=="yumi_link_3_l") {
        rot = Rl1*Rq_l1*Rl2*Rq_l2;
    }
    else if (joint_name=="yumi_link_2_l") {
        rot = Rl1*Rq_l1;
    }

    if (joint_name=="yumi_link_7_r") {
        rot = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Rr7*Rq_r7;
    }
    else if (joint_name=="yumi_link_6_r") {
        rot = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6;
    }
    else if (joint_name=="yumi_link_5_r") {
        rot = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5;
    }
    else if (joint_name=="yumi_link_4_r") {
        rot = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3;
    }
    else if (joint_name=="yumi_link_3_r") {
        rot = Rr1*Rq_r1*Rr2*Rq_r2;
    }
    else if (joint_name=="yumi_link_2_r") {
        rot = Rr1*Rq_r1;
    }
    Eigen::Quaterniond ori(rot);
    return ori;
}

void Result2x(const Result result, Pose6d&  x_left, Pose6d&  x_right) {
    Vector3d l_wrist_pos = result.l_wrist_pos;
    Vector3d r_wrist_pos = result.r_wrist_pos;
    Quaterniond l_wrist_quat = result.l_wrist_quat;
    Quaterniond r_wrist_quat = result.r_wrist_quat;
    x_left.t = l_wrist_pos;
    x_left.R= l_wrist_quat;
    x_right.t = r_wrist_pos;
    x_right.R= r_wrist_quat;
}

void Result2xe(const Result result, Eigen::Matrix<double,ELBOW_DOF,1>&  x_left, Eigen::Matrix<double,ELBOW_DOF,1>&  x_right) {
    Vector3d l_elbow_pos = result.l_elbow_pos;
    Vector3d r_elbow_pos = result.r_elbow_pos;
    x_left.block(0,0,3,1) = l_elbow_pos;
    x_right.block(0,0,3,1) = r_elbow_pos;
}

} // namespace kinematics

#endif