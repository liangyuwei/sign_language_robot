#ifndef UTIL_FUNCTIONS_H_
#define UTIL_FUNCTIONS_H_
#include <iostream>
#include <vector>
#include "config.h"
// For Eigen
#include <Eigen/Core>
#include <Eigen/Geometry> 
// #include <Eigen/Dense>
// #include <Eigen/QR>

using namespace cfg;

double util_clamp(double x, const double min_val, const double max_val) {
    x = std::min(max_val,x);
    x = std::max(min_val,x);
    return x;
}

double linear_map(const double x,const double min_i,const double max_i,const double min_o,const double max_o){
    return (x-min_i)/(max_i-min_i)*(max_o-min_o) + min_o;
}

std::vector<double>  convert_glove_angle(std::vector<double> q_finger_human){
    /*
        Direct mapping and linear scaling
        @param:q_finger_human            double[14]
        @param:q_finger_robot_goal    double[12]
    */
    std::vector<double> q_finger_robot_goal(NUM_OF_FINGER_JOINTS/2);
    q_finger_robot_goal[0] = linear_map(q_finger_human[3], WISEGLOVE_LB[3], WISEGLOVE_UB[3], ROBOTHAND_LB[0], ROBOTHAND_UB[0]);
    q_finger_robot_goal[1] = linear_map(q_finger_human[4], WISEGLOVE_LB[4], WISEGLOVE_UB[4], ROBOTHAND_LB[1], ROBOTHAND_UB[1]);
    q_finger_robot_goal[2] = linear_map(q_finger_human[6], WISEGLOVE_LB[6], WISEGLOVE_UB[6], ROBOTHAND_LB[2], ROBOTHAND_UB[2]);
    q_finger_robot_goal[3] = linear_map(q_finger_human[7], WISEGLOVE_LB[7], WISEGLOVE_UB[7], ROBOTHAND_LB[3], ROBOTHAND_UB[3]);
    q_finger_robot_goal[4] = linear_map(q_finger_human[9], WISEGLOVE_LB[9], WISEGLOVE_UB[9], ROBOTHAND_LB[4], ROBOTHAND_UB[4]);
    q_finger_robot_goal[5] = linear_map(q_finger_human[10], WISEGLOVE_LB[10], WISEGLOVE_UB[10], ROBOTHAND_LB[5], ROBOTHAND_UB[5]);
    q_finger_robot_goal[6] = linear_map(q_finger_human[12], WISEGLOVE_LB[12], WISEGLOVE_UB[12], ROBOTHAND_LB[6], ROBOTHAND_UB[6]);
    q_finger_robot_goal[7] = linear_map(q_finger_human[13], WISEGLOVE_LB[13], WISEGLOVE_UB[13], ROBOTHAND_LB[7], ROBOTHAND_UB[7]);
    q_finger_robot_goal[8] = 0.1*((ROBOTHAND_LB[8] + ROBOTHAND_UB[8]) / 2.0);
    q_finger_robot_goal[9] = 0.1*linear_map(q_finger_human[2], WISEGLOVE_LB[2], WISEGLOVE_UB[2], ROBOTHAND_LB[9], ROBOTHAND_UB[9]);
    q_finger_robot_goal[10] = linear_map(q_finger_human[0], WISEGLOVE_LB[0], WISEGLOVE_UB[0], ROBOTHAND_LB[10], ROBOTHAND_UB[10]);
    q_finger_robot_goal[11] = linear_map(q_finger_human[1], WISEGLOVE_LB[1], WISEGLOVE_UB[1], ROBOTHAND_LB[11], ROBOTHAND_UB[11]); 
    return q_finger_robot_goal;
}

std::vector<double>  convert_glove_angle_dof15(std::vector<double> q_finger_human){
    /*
        Direct mapping and linear scaling
        @param:q_finger_human            double[15]
        @param:q_finger_robot_goal       double[12]
    */
    std::vector<double> q_finger_robot_goal(NUM_OF_FINGER_JOINTS/2);
    q_finger_robot_goal[0] = linear_map(q_finger_human[3], WISEGLOVE_LB[3], WISEGLOVE_UB[3], ROBOTHAND_LB[0], ROBOTHAND_UB[0]);
    q_finger_robot_goal[1] = linear_map(q_finger_human[4], WISEGLOVE_LB[4], WISEGLOVE_UB[4], ROBOTHAND_LB[1], ROBOTHAND_UB[1]);
    q_finger_robot_goal[2] = linear_map(q_finger_human[6], WISEGLOVE_LB[6], WISEGLOVE_UB[6], ROBOTHAND_LB[2], ROBOTHAND_UB[2]);
    q_finger_robot_goal[3] = linear_map(q_finger_human[7], WISEGLOVE_LB[7], WISEGLOVE_UB[7], ROBOTHAND_LB[3], ROBOTHAND_UB[3]);
    q_finger_robot_goal[4] = linear_map(q_finger_human[9], WISEGLOVE_LB[9], WISEGLOVE_UB[9], ROBOTHAND_LB[4], ROBOTHAND_UB[4]);
    q_finger_robot_goal[5] = linear_map(q_finger_human[10], WISEGLOVE_LB[10], WISEGLOVE_UB[10], ROBOTHAND_LB[5], ROBOTHAND_UB[5]);
    q_finger_robot_goal[6] = linear_map(q_finger_human[12], WISEGLOVE_LB[12], WISEGLOVE_UB[12], ROBOTHAND_LB[6], ROBOTHAND_UB[6]);
    q_finger_robot_goal[7] = linear_map(q_finger_human[13], WISEGLOVE_LB[13], WISEGLOVE_UB[13], ROBOTHAND_LB[7], ROBOTHAND_UB[7]);
    q_finger_robot_goal[8] = linear_map(q_finger_human[14], WISEGLOVE_LB[14], WISEGLOVE_UB[14], ROBOTHAND_LB[8], ROBOTHAND_UB[8]);
    q_finger_robot_goal[9] = linear_map(q_finger_human[2], WISEGLOVE_LB[2], WISEGLOVE_UB[2], ROBOTHAND_LB[9], ROBOTHAND_UB[9]);
    q_finger_robot_goal[10] = linear_map(q_finger_human[0], WISEGLOVE_LB[0], WISEGLOVE_UB[0], ROBOTHAND_LB[10], ROBOTHAND_UB[10]);
    q_finger_robot_goal[11] = linear_map(q_finger_human[1], WISEGLOVE_LB[1], WISEGLOVE_UB[1], ROBOTHAND_LB[11], ROBOTHAND_UB[11]);

    q_finger_robot_goal[0] = util_clamp(q_finger_robot_goal[0],ROBOTHAND_UB[0],ROBOTHAND_LB[0]);
    q_finger_robot_goal[1] = util_clamp(q_finger_robot_goal[1],ROBOTHAND_UB[1],ROBOTHAND_LB[1]);
    q_finger_robot_goal[2] = util_clamp(q_finger_robot_goal[2],ROBOTHAND_UB[2],ROBOTHAND_LB[2]);
    q_finger_robot_goal[3] = util_clamp(q_finger_robot_goal[3],ROBOTHAND_UB[3],ROBOTHAND_LB[3]);
    q_finger_robot_goal[4] = util_clamp(q_finger_robot_goal[4],ROBOTHAND_UB[4],ROBOTHAND_LB[4]);
    q_finger_robot_goal[5] = util_clamp(q_finger_robot_goal[5],ROBOTHAND_UB[5],ROBOTHAND_LB[5]);
    q_finger_robot_goal[6] = util_clamp(q_finger_robot_goal[6],ROBOTHAND_UB[6],ROBOTHAND_LB[6]);
    q_finger_robot_goal[7] = util_clamp(q_finger_robot_goal[7],ROBOTHAND_UB[7],ROBOTHAND_LB[7]);
    q_finger_robot_goal[8] = util_clamp(q_finger_robot_goal[8],ROBOTHAND_UB[8],ROBOTHAND_LB[8]);
    q_finger_robot_goal[9] = util_clamp(q_finger_robot_goal[9],ROBOTHAND_UB[9],ROBOTHAND_LB[9]);
    q_finger_robot_goal[10] = util_clamp(q_finger_robot_goal[10],ROBOTHAND_UB[10],ROBOTHAND_LB[10]);
    q_finger_robot_goal[11] = util_clamp(q_finger_robot_goal[11],ROBOTHAND_UB[11],ROBOTHAND_LB[11]);

    return q_finger_robot_goal;
}

Eigen::Matrix<double,NUM_OF_FINGER_JOINTS/2,1>  convert_glove_angle_matrix_form(Eigen::Matrix<double,NUM_OF_GLOVE_ANGLES,1> q_finger_human){
    /*
        Direct mapping and linear scaling
        @param:q_finger_human            Matrix<double,14,1>
        @param:q_finger_robot_goal    Matrix<double,12,1>
    */
    Eigen::Matrix<double,NUM_OF_FINGER_JOINTS/2,1> q_finger_robot_goal;
    q_finger_robot_goal[0] = linear_map(q_finger_human[3], WISEGLOVE_LB[3], WISEGLOVE_UB[3], ROBOTHAND_LB[0], ROBOTHAND_UB[0]);
    q_finger_robot_goal[1] = linear_map(q_finger_human[4], WISEGLOVE_LB[4], WISEGLOVE_UB[4], ROBOTHAND_LB[1], ROBOTHAND_UB[1]);
    q_finger_robot_goal[2] = linear_map(q_finger_human[6], WISEGLOVE_LB[6], WISEGLOVE_UB[6], ROBOTHAND_LB[2], ROBOTHAND_UB[2]);
    q_finger_robot_goal[3] = linear_map(q_finger_human[7], WISEGLOVE_LB[7], WISEGLOVE_UB[7], ROBOTHAND_LB[3], ROBOTHAND_UB[3]);
    q_finger_robot_goal[4] = linear_map(q_finger_human[9], WISEGLOVE_LB[9], WISEGLOVE_UB[9], ROBOTHAND_LB[4], ROBOTHAND_UB[4]);
    q_finger_robot_goal[5] = linear_map(q_finger_human[10], WISEGLOVE_LB[10], WISEGLOVE_UB[10], ROBOTHAND_LB[5], ROBOTHAND_UB[5]);
    q_finger_robot_goal[6] = linear_map(q_finger_human[12], WISEGLOVE_LB[12], WISEGLOVE_UB[12], ROBOTHAND_LB[6], ROBOTHAND_UB[6]);
    q_finger_robot_goal[7] = linear_map(q_finger_human[13], WISEGLOVE_LB[13], WISEGLOVE_UB[13], ROBOTHAND_LB[7], ROBOTHAND_UB[7]);
    q_finger_robot_goal[8] = 0.1*((ROBOTHAND_LB[8] + ROBOTHAND_UB[8]) / 2.0);
    q_finger_robot_goal[9] = 0.1*linear_map(q_finger_human[2], WISEGLOVE_LB[2], WISEGLOVE_UB[2], ROBOTHAND_LB[9], ROBOTHAND_UB[9]);
    q_finger_robot_goal[10] = linear_map(q_finger_human[0], WISEGLOVE_LB[0], WISEGLOVE_UB[0], ROBOTHAND_LB[10], ROBOTHAND_UB[10]);
    q_finger_robot_goal[11] = linear_map(q_finger_human[1], WISEGLOVE_LB[1], WISEGLOVE_UB[1], ROBOTHAND_LB[11], ROBOTHAND_UB[11]); 
    return q_finger_robot_goal;
}

std::vector<std::vector<double>> convert_glove_angle_traj(std::vector<std::vector<double>> glove_angle_traj) {
    std::vector<std::vector<double>> convertion_result;
    for (auto glove_angle : glove_angle_traj)
    {
        std::vector<double> robot_finger_goal = convert_glove_angle(glove_angle);
        convertion_result.push_back(robot_finger_goal);
    }
    return convertion_result;
}

std::vector<Matrix<double,NUM_OF_FINGER_JOINTS/2,1>> convert_glove_angle_traj_matrix_form(std::vector<std::vector<double>> glove_angle_traj) {
    std::vector<Matrix<double,NUM_OF_FINGER_JOINTS/2,1>> convertion_result;
    for (auto glove_angle : glove_angle_traj)
    {
        Eigen::Matrix<double,NUM_OF_FINGER_JOINTS/2,1> robot_finger_goal = convert_glove_angle_matrix_form(Map<MatrixXd>(glove_angle.data(), NUM_OF_GLOVE_ANGLES, 1));
        convertion_result.push_back(robot_finger_goal);
    }
    return convertion_result;
}

#endif