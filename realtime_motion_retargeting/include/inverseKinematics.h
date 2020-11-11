#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_
#include "config.h"
#include "transform_matrix_helper.h"
#include "static_matrix_config.h"
#include "kinematics.h"
// using namespace cfg;
using namespace matrix_helper;
using namespace cfg;

namespace inverseKinematics{

void calculate_jacobian(const Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle, 
                                                Eigen::Matrix<double,WRIST_DOF,NUM_OF_JOINTS/2> & Jacobian_left,
                                                Eigen::Matrix<double,WRIST_DOF,NUM_OF_JOINTS/2> & Jacobian_right) {
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
    Eigen::Matrix<double,3,3> dRq_l1 = dRzc(joint_angle(0,0)); 
    Eigen::Matrix<double,3,3> dRq_l2 = dRzc(joint_angle(1,0)); 
    Eigen::Matrix<double,3,3> dRq_l3 = dRzc(joint_angle(2,0)); 
    Eigen::Matrix<double,3,3> dRq_l4 = dRzc(joint_angle(3,0)); 
    Eigen::Matrix<double,3,3> dRq_l5 = dRzc(joint_angle(4,0)); 
    Eigen::Matrix<double,3,3> dRq_l6 = dRzc(joint_angle(5,0)); 
    Eigen::Matrix<double,3,3> dRq_r1 = dRzc(joint_angle(7,0)); 
    Eigen::Matrix<double,3,3> dRq_r2 = dRzc(joint_angle(8,0)); 
    Eigen::Matrix<double,3,3> dRq_r3 = dRzc(joint_angle(9,0)); 
    Eigen::Matrix<double,3,3> dRq_r4 = dRzc(joint_angle(10,0)); 
    Eigen::Matrix<double,3,3> dRq_r5 = dRzc(joint_angle(11,0)); 
    Eigen::Matrix<double,3,3> dRq_r6 = dRzc(joint_angle(12,0)); 
    // Left arm
    // The fisrt column of pos
    Jacobian_left.block(0,0,3,1) =  Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Tl6 +
                                                                        Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 + Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*dRq_l1*Rl2*Rq_l2*Tl3 + Rl1*dRq_l1*Tl2;
    // The second column of pos
    Jacobian_left.block(0,1,3,1) = Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Tl6 +
                                                                        Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 + Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*dRq_l2*Tl3;
    // The third column of pos
    Jacobian_left.block(0,2,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Tl6 +
                                                                        Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Rl4*Rq_l4*Tl5 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Tl4;
    // The fourth column of pos
    Jacobian_left.block(0,3,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*dRq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*dRq_l4*Rl5*Rq_l5*Tl6 +
                                                                        Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*dRq_l4*Tl5;
    // The fifth column of pos
    Jacobian_left.block(0,4,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*dRq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*dRq_l5*Tl6;
    // The sixth column of pos
    Jacobian_left.block(0,5,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*dRq_l6*Tl7;
    // The seventh column of pos
    Jacobian_left.block(0,6,3,1) = Eigen::Matrix<double,3,1>::Zero();
    // The first column of ori
    Jacobian_left.block(3,0,3,1) = Rl1.block(0,2,3,1);
    // The second column of ori
    Jacobian_left.block(3,1,3,1) = (Rl1*Rq_l1*Rl2).block(0,2,3,1);
    // The third column of ori
    Jacobian_left.block(3,2,3,1) = (Rl1*Rq_l1*Rl2*Rq_l2*Rl3).block(0,2,3,1);
    // The fourth column of ori
    Jacobian_left.block(3,3,3,1) = (Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4).block(0,2,3,1);
    // The fifth column of ori
    Jacobian_left.block(3,4,3,1) = (Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5).block(0,2,3,1);
    // The sixth column of ori
    Jacobian_left.block(3,5,3,1) = (Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6).block(0,2,3,1);
    // The seventh column of ori
    Jacobian_left.block(3,6,3,1) = (Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Rl7).block(0,2,3,1);

    // Right arm
    // The fisrt column of pos
    Jacobian_right.block(0,0,3,1) =  Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Tr6 +
                                                                        Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 + Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*dRq_r1*Rr2*Rq_r2*Tr3 + Rr1*dRq_r1*Tr2;
    // The second column of pos
    Jacobian_right.block(0,1,3,1) = Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Tr6 +
                                                                        Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 + Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*dRq_r2*Tr3;
    // The third column of pos
    Jacobian_right.block(0,2,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Tr6 +
                                                                        Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Rr4*Rq_r4*Tr5 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Tr4;
    // The fourth column of pos
    Jacobian_right.block(0,3,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*dRq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*dRq_r4*Rr5*Rq_r5*Tr6 +
                                                                        Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*dRq_r4*Tr5;
    // The fifth column of pos
    Jacobian_right.block(0,4,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*dRq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*dRq_r5*Tr6;
    // The sixth column of pos
    Jacobian_right.block(0,5,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*dRq_r6*Tr7;
    // The seventh column of pos
    Jacobian_right.block(0,6,3,1) = Eigen::Matrix<double,3,1>::Zero();
    // The first column of ori
    Jacobian_right.block(3,0,3,1) = Rr1.block(0,2,3,1);
    // The second column of ori
    Jacobian_right.block(3,1,3,1) = (Rr1*Rq_r1*Rr2).block(0,2,3,1);
    // The third column of ori
    Jacobian_right.block(3,2,3,1) = (Rr1*Rq_r1*Rr2*Rq_r2*Rr3).block(0,2,3,1);
    // The fourth column of ori
    Jacobian_right.block(3,3,3,1) = (Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4).block(0,2,3,1);
    // The fifth column of ori
    Jacobian_right.block(3,4,3,1) = (Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5).block(0,2,3,1);
    // The sixth column of ori
    Jacobian_right.block(3,5,3,1) = (Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6).block(0,2,3,1);
    // The seventh column of ori
    Jacobian_right.block(3,6,3,1) = (Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Rr7).block(0,2,3,1);
}

void calculate_jacobian_for_elbow(Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle, 
                                                Eigen::Matrix<double,ELBOW_DOF,NUM_OF_JOINTS/2> & Jacobian_left,
                                                Eigen::Matrix<double,ELBOW_DOF,NUM_OF_JOINTS/2> & Jacobian_right) {
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
    Eigen::Matrix<double,3,3> dRq_l1 = dRzc(joint_angle(0,0)); 
    Eigen::Matrix<double,3,3> dRq_l2 = dRzc(joint_angle(1,0)); 
    Eigen::Matrix<double,3,3> dRq_l3 = dRzc(joint_angle(2,0)); 
    Eigen::Matrix<double,3,3> dRq_r1 = dRzc(joint_angle(7,0)); 
    Eigen::Matrix<double,3,3> dRq_r2 = dRzc(joint_angle(8,0)); 
    Eigen::Matrix<double,3,3> dRq_r3 = dRzc(joint_angle(9,0)); 
    // Left arm
    // The fisrt column of pos
    Jacobian_left.block(0,0,3,1) =  Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*dRq_l1*Rl2*Rq_l2*Tl3 + Rl1*dRq_l1*Tl2;
    // The second column of pos
    Jacobian_left.block(0,1,3,1) = Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*dRq_l2*Tl3;
    // The third column of pos
    Jacobian_left.block(0,2,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Tl4;
    // The fourth column of pos
    Jacobian_left.block(0,3,3,1) = Eigen::Matrix<double,3,1>::Zero();
    // The fifth column of pos
    Jacobian_left.block(0,4,3,1) = Eigen::Matrix<double,3,1>::Zero();
    // The sixth column of pos
    Jacobian_left.block(0,5,3,1) = Eigen::Matrix<double,3,1>::Zero();
    // The seventh column of pos
    Jacobian_left.block(0,6,3,1) = Eigen::Matrix<double,3,1>::Zero();

    // Right arm
    // The fisrt column of pos
    Jacobian_right.block(0,0,3,1) =  Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*dRq_r1*Rr2*Rq_r2*Tr3 + Rr1*dRq_r1*Tr2;
    // The second column of pos
    Jacobian_right.block(0,1,3,1) = Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*dRq_r2*Tr3;
    // The third column of pos
    Jacobian_right.block(0,2,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Tr4;
    // The fourth column of pos
    Jacobian_right.block(0,3,3,1) = Eigen::Matrix<double,3,1>::Zero();
    // The fifth column of pos
    Jacobian_right.block(0,4,3,1) = Eigen::Matrix<double,3,1>::Zero();
    // The sixth column of pos
    Jacobian_right.block(0,5,3,1) = Eigen::Matrix<double,3,1>::Zero();
    // The seventh column of pos
    Jacobian_right.block(0,6,3,1) = Eigen::Matrix<double,3,1>::Zero();
}

void calculate_jacobian_for_specific_link(Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle, 
                                            std::string link_name,
                                            Eigen::Matrix<double,3,NUM_OF_JOINTS/2> & Jacobian
                                            ) {
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
    Eigen::Matrix<double,3,3> dRq_l1 = dRzc(joint_angle(0,0)); 
    Eigen::Matrix<double,3,3> dRq_l2 = dRzc(joint_angle(1,0)); 
    Eigen::Matrix<double,3,3> dRq_l3 = dRzc(joint_angle(2,0)); 
    Eigen::Matrix<double,3,3> dRq_l4 = dRzc(joint_angle(3,0)); 
    Eigen::Matrix<double,3,3> dRq_l5 = dRzc(joint_angle(4,0)); 
    Eigen::Matrix<double,3,3> dRq_l6 = dRzc(joint_angle(5,0)); 
    Eigen::Matrix<double,3,3> dRq_r1 = dRzc(joint_angle(7,0)); 
    Eigen::Matrix<double,3,3> dRq_r2 = dRzc(joint_angle(8,0)); 
    Eigen::Matrix<double,3,3> dRq_r3 = dRzc(joint_angle(9,0)); 
    Eigen::Matrix<double,3,3> dRq_r4 = dRzc(joint_angle(10,0)); 
    Eigen::Matrix<double,3,3> dRq_r5 = dRzc(joint_angle(11,0)); 
    Eigen::Matrix<double,3,3> dRq_r6 = dRzc(joint_angle(12,0)); 
    // Initialize Jacobian
    Jacobian = Eigen::Matrix<double,3,NUM_OF_JOINTS/2>::Zero();
    // Calculate Jacobian
    if(link_name=="yumi_link_7_l") {
        // The fisrt column of pos
        Jacobian.block(0,0,3,1) =  Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Tl6 +
                                                                            Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 + Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*dRq_l1*Rl2*Rq_l2*Tl3 + Rl1*dRq_l1*Tl2;
        // The second column of pos
        Jacobian.block(0,1,3,1) = Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Tl6 +
                                                                            Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 + Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*dRq_l2*Tl3;
        // The third column of pos
        Jacobian.block(0,2,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Tl6 +
                                                                            Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Rl4*Rq_l4*Tl5 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Tl4;
        // The fourth column of pos
        Jacobian.block(0,3,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*dRq_l4*Rl5*Rq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*dRq_l4*Rl5*Rq_l5*Tl6 +
                                                                            Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*dRq_l4*Tl5;
        // The fifth column of pos
        Jacobian.block(0,4,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*dRq_l5*Rl6*Rq_l6*Tl7 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*dRq_l5*Tl6;
        // The sixth column of pos
        Jacobian.block(0,5,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Rl5*Rq_l5*Rl6*dRq_l6*Tl7;
    }
    else if(link_name=="yumi_link_7_r") {
        // The fisrt column of pos
        Jacobian.block(0,0,3,1) =  Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Tr6 +
                                                                            Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 + Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*dRq_r1*Rr2*Rq_r2*Tr3 + Rr1*dRq_r1*Tr2;
        // The second column of pos
        Jacobian.block(0,1,3,1) = Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Tr6 +
                                                                            Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 + Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*dRq_r2*Tr3;
        // The third column of pos
        Jacobian.block(0,2,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Tr6 +
                                                                            Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Rr4*Rq_r4*Tr5 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Tr4;
        // The fourth column of pos
        Jacobian.block(0,3,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*dRq_r4*Rr5*Rq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*dRq_r4*Rr5*Rq_r5*Tr6 +
                                                                            Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*dRq_r4*Tr5;
        // The fifth column of pos
        Jacobian.block(0,4,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*dRq_r5*Rr6*Rq_r6*Tr7 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*dRq_r5*Tr6;
        // The sixth column of pos
        Jacobian.block(0,5,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Rr5*Rq_r5*Rr6*dRq_r6*Tr7;
    }
    else if(link_name=="yumi_link_5_l") {
        // The fisrt column of pos
        Jacobian.block(0,0,3,1) = Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 + Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*dRq_l1*Rl2*Rq_l2*Tl3 + Rl1*dRq_l1*Tl2;
        // The second column of pos
        Jacobian.block(0,1,3,1) = Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Rl4*Rq_l4*Tl5 + Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*dRq_l2*Tl3;
        // The third column of pos
        Jacobian.block(0,2,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Rl4*Rq_l4*Tl5 + Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Tl4;
        // The fourth column of pos
        Jacobian.block(0,3,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Rl4*dRq_l4*Tl5;
    }
    else if(link_name=="yumi_link_5_r") {
        // The fisrt column of pos
        Jacobian.block(0,0,3,1) = Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 + Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*dRq_r1*Rr2*Rq_r2*Tr3 + Rr1*dRq_r1*Tr2;
        // The second column of pos
        Jacobian.block(0,1,3,1) = Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Rr4*Rq_r4*Tr5 + Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*dRq_r2*Tr3;
        // The third column of pos
        Jacobian.block(0,2,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Rr4*Rq_r4*Tr5 + Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Tr4;
        // The fourth column of pos
        Jacobian.block(0,3,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Rr4*dRq_r4*Tr5;
    }
    else if(link_name=="yumi_link_4_l") {
        // The fisrt column of pos
        Jacobian.block(0,0,3,1) = Rl1*dRq_l1*Rl2*Rq_l2*Rl3*Rq_l3*Tl4 + Rl1*dRq_l1*Rl2*Rq_l2*Tl3 + Rl1*dRq_l1*Tl2;
        // The second column of pos
        Jacobian.block(0,1,3,1) = Rl1*Rq_l1*Rl2*dRq_l2*Rl3*Rq_l3*Tl4 + Rl1*Rq_l1*Rl2*dRq_l2*Tl3;
        // The third column of pos
        Jacobian.block(0,2,3,1) = Rl1*Rq_l1*Rl2*Rq_l2*Rl3*dRq_l3*Tl4;
    }
    else if(link_name=="yumi_link_4_r") {
        // The fisrt column of pos
        Jacobian.block(0,0,3,1) = Rr1*dRq_r1*Rr2*Rq_r2*Rr3*Rq_r3*Tr4 + Rr1*dRq_r1*Rr2*Rq_r2*Tr3 + Rr1*dRq_r1*Tr2;
        // The second column of pos
        Jacobian.block(0,1,3,1) = Rr1*Rq_r1*Rr2*dRq_r2*Rr3*Rq_r3*Tr4 + Rr1*Rq_r1*Rr2*dRq_r2*Tr3;
        // The third column of pos
        Jacobian.block(0,2,3,1) = Rr1*Rq_r1*Rr2*Rq_r2*Rr3*dRq_r3*Tr4;
    }
    else if(link_name=="yumi_link_3_l") {
        // The fisrt column of pos
        Jacobian.block(0,0,3,1) = Rl1*dRq_l1*Rl2*Rq_l2*Tl3 + Rl1*dRq_l1*Tl2;
        // The second column of pos
        Jacobian.block(0,1,3,1) = Rl1*Rq_l1*Rl2*dRq_l2*Tl3;
    }
    else if(link_name=="yumi_link_3_r") {
        // The fisrt column of pos
        Jacobian.block(0,0,3,1) = Rr1*dRq_r1*Rr2*Rq_r2*Tr3 + Rr1*dRq_r1*Tr2;
        // The second column of pos
        Jacobian.block(0,1,3,1) = Rr1*Rq_r1*Rr2*dRq_r2*Tr3;
    }
}

Eigen::MatrixXd pinv(Eigen::MatrixXd  A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
    const static double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) 
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*

    return X;

}

Eigen::MatrixXd plus_inverse(Eigen::MatrixXd J, const double lambda) {
    const int r = J.rows();

    // // Decide if J is nearly singular by its rank
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
    // const static double  pinvtoler = 1.e-8; //tolerance
    // Eigen::MatrixXd singularValues = svd.singularValues();//奇异值
    // int rank = 0;
    // for (long i = 0; i<r; ++i) {
    //     if (singularValues(i) > pinvtoler)
    //         rank += 1;
    // }

    // // Output for debug
    // std::cout<<"Lambda: " << lambda <<std::endl;
    // std::cout<<"Jacobian: " << (J*J.transpose() + lambda * identity) <<std::endl;
    // std::cout<< "Inverse:" << (J*J.transpose() + lambda * identity).inverse() <<std::endl;

    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(r,r);
    return J.transpose() * (J*J.transpose() + lambda * identity).inverse();
}

Eigen::Matrix<double,NUM_OF_JOINTS/2,NUM_OF_JOINTS/2> null_space_projection(Eigen::Matrix<double,WRIST_DOF,NUM_OF_JOINTS/2> J, Eigen::Matrix<double,NUM_OF_JOINTS/2,WRIST_DOF> J_plus) {
    Eigen::Matrix<double,NUM_OF_JOINTS/2,NUM_OF_JOINTS/2> I = Eigen::Matrix<double,NUM_OF_JOINTS/2,NUM_OF_JOINTS/2>::Identity();
    return I - J_plus*J;
}

void calculate_dx(Eigen::Matrix<double,WRIST_DOF,1> &dx, const Pose6d x0, const Pose6d x1) {
    dx.block(0,0,3,1) = x1.t - x0.t;
    // cout<<"dx: "<<(x1-x0).block(0,0,3,1).transpose()<<endl;
    Eigen::Quaterniond quat0 = x0.R;
    Eigen::Quaterniond quat1 = x1.R;
    // Eigen::Matrix3d drot = euler2rotmat(ea0).transpose()*euler2rotmat(ea1);
    Eigen::Matrix3d drot = quat1.toRotationMatrix()*(quat0.toRotationMatrix().transpose())- Eigen::Matrix3d::Identity();
    // Calculate dtheta
    Eigen::Vector3d dtheta;
    dtheta[0] = (drot(2,1)-drot(1,2))/2;
    dtheta[1] = (drot(0,2)-drot(2,0))/2;
    dtheta[2] = (drot(1,0)-drot(0,1))/2;
    // (2) Output some information for debug
    // cout<<"drot: "<<drot.transpose()<<endl;
    // cout<<"dtheta: "<<dtheta.transpose()<<endl;
    dx.block(3,0,3,1) = dtheta;
}

Pose6d interpolate_between_pose6d(const Pose6d x0, const Pose6d x1, const double t) {
    Pose6d xt;
    xt.t = t * (x1.t - x0.t) + x0.t;
    xt.R = x0.R.slerp(t, x1.R);
    return xt;
}

Eigen::Vector3d interpolate_between_pos3d(const Eigen::Vector3d x0, const Eigen::Vector3d x1, const double t) {
    Eigen::Vector3d xt;
    xt = t * (x1 - x0) + x0;
    return xt;
}

void calculate_dxe(Eigen::Matrix<double,ELBOW_DOF,1> &dx, const Eigen::Matrix<double,ELBOW_DOF,1> x0, const Eigen::Matrix<double,ELBOW_DOF,1> x1) {
    dx= (x1-x0);
}

} // namespace inverseKinematics

#endif