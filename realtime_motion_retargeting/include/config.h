#ifndef _CONFIG_H
#define _CONFIG_H

// For Eigen
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <iostream>
#include <string>
#include <cmath>
using namespace std;

// Parameter configs & some declarations of structs
namespace cfg{
    const static int LENGTH = 50;
    const static int NUM_DATAPOINTS = 50;
    const static int DENSE_DATAPOINTS = 1000;
    const static int NUM_OF_JOINTS = 14;
    const static int NUM_OF_FINGER_JOINTS = 24;
    const static int NUM_OF_GLOVE_ANGLES = 14;
    const static int TOTAL_NUM_OF_JOINTS = 38;
    const static int JOINT_DOF = 38;  // 2*(12+7)
    const static int DMPPOINTS_DOF = 24;  // 2(start & goal)*4*3
    const static int WRIST_DOF = 6;
    const static int ELBOW_DOF = 3;
    const static string IN_H5_NAME = "/home/liweijie/projects/motion-retargeting-optimize/mocap_data_YuMi_affine_interpolated.h5";
    const static string OUT_H5_NAME = "/home/liweijie/projects/motion-retargeting-optimize/mocap_data_YuMi_optimize_results.h5";
    const static string GROUP_NAME = "fengren.bag";

    // Hyper params
    static double K_ELBOW_FK_COST = 0.0;
    static double K_WRIST_FK_COST = 100.0;
    static double K_CONSTRAINTS_COST = 1.0;
    static double K_NULLSPACE_ELBOW = 0.4;
    static double K_COL = 1.0;
    static double K_SMOOTHNESS = 0.01;
    static double K_MAINTAIN = 0.01;
    static double K_DMPSTARTSGOALS = 1.0;
    static double K_DMPSCALEMARGIN = 1.0;
    static double K_DMPRELCHANGE = 1.0;
    static int NUM_ROUNDS = 10;
    static int NUM_ROUNDS_2 = 100;
    static int NUM_TO_ITERATE_DMP = 5;
    static int NUM_TO_ITERATE_COL = 10;
    static double lambda = 0.1;
    static int num_check = 10;
    static bool isDamped = false;
    static int nbInterval = 10;

    const static double trans_l1[3] =  {0.05355, 0.0725, 0.51492};
    const static double trans_l2[3] =  {0.03, 0.0, 0.1};
    const static double trans_l3[3] =  {-0.03, 0.17283, 0.0};
    const static double trans_l4[3] =  {-0.04188, 0.0, 0.07873};
    const static double trans_l5[3] =  {0.0405, 0.16461, 0.0};
    const static double trans_l6[3] =  {-0.027, 0, 0.10039};
    const static double trans_l7[3] =  {0.027, 0.029, 0.0};
    const static double trans_r1[3] =  {0.05355, -0.0725, 0.51492};
    const static double trans_r2[3] =  {0.03, 0.0, 0.1};
    const static double trans_r3[3] =  {-0.03, 0.17283, 0.0};
    const static double trans_r4[3] =  {-0.04188, 0.0, 0.07873};
    const static double trans_r5[3] =  {0.0405, 0.16461, 0.0};
    const static double trans_r6[3] =  {-0.027, 0, 0.10039};
    const static double trans_r7[3] =  {0.027, 0.029, 0.0};

    const static double rot_l1[3] = {0.9781,  -0.5716,   2.3180};
    const static double rot_l2[3] = {1.57079632679, 0, 0};
    const static double rot_l3[3] = {-1.57079632679, 0, 0};
    const static double rot_l4[3] = {1.57079632679, -1.57079632679, 0};
    const static double rot_l5[3] = {-1.57079632679, 0, 0};
    const static double rot_l6[3] = {1.57079632679, 0, 0};
    const static double rot_l7[3] = {-1.57079632679, 0, 0};
    const static double rot_r1[3] = {-0.9795,  -0.5682,   -2.3155};
    const static double rot_r2[3] = {1.57079632679, 0, 0};
    const static double rot_r3[3] = {-1.57079632679, 0, 0};
    const static double rot_r4[3] = {1.57079632679, -1.57079632679, 0};
    const static double rot_r5[3] = {-1.57079632679, 0, 0};
    const static double rot_r6[3] = {1.57079632679, 0, 0};
    const static double rot_r7[3] = {-1.57079632679, 0, 0};

     // Variables' bounds
    // const static double YUMI_LOWER_LIMITS[NUM_OF_JOINTS] = 
    //     {-2.8, -2.49, -1.2, -1.7, -2.0, -1.5, -2.0, -0.5, -2.49, -2.2, -1.7, -2.0, -1.5, -2.0}; 
    // const static double YUMI_UPPER_LIMITS[NUM_OF_JOINTS] = 
    //     {0.5, 0.75, 2.2, 1.4, 1.578, 2.1, 1.578, 2.8, 0.75, 1.2, 1.4, 1.578, 2.1, 1.578}; 

    const static double YUMI_LOWER_LIMITS[NUM_OF_JOINTS] = {
        -2.94,-2.50,-2.94,-2.15,-5.06,-1.53,-3.99,
        -2.94,-2.50,-2.94,-2.15,-5.06,-1.53,-3.99
    };
    const static double YUMI_UPPER_LIMITS[NUM_OF_JOINTS] = {
        2.94,0.75,2.94,1.39,5.06,2.40,3.99,
        2.94,0.75,2.94,1.39,5.06,2.40,3.99
    };

    const std::vector<double> q_l_arm_lb = {-2.8, -2.49, -1.2, -1.7, -2.0, -1.5, -2.0};
    const std::vector<double> q_l_arm_ub = {0.5, 0.75, 2.2, 1.4, 1.578, 2.1, 1.578};//{2.94, 0.76, 2.94, 1.4, 5.06, 2.41, 4.0};
    const std::vector<double> q_r_arm_lb = {-0.5, -2.49, -2.2, -1.7, -2.0, -1.5, -2.0}; // modified on 2020/07/20
    const std::vector<double> q_r_arm_ub = {2.8, 0.75, 1.2, 1.4, 1.578, 2.1, 1.578}; // modified on 2020/07/20
    const Eigen::Matrix<double,7,1> q_l_arm_lb_mat(q_l_arm_lb.data());
    const Eigen::Matrix<double,7,1> q_l_arm_ub_mat(q_l_arm_ub.data());
    const Eigen::Matrix<double,7,1> q_r_arm_lb_mat(q_r_arm_lb.data());
    const Eigen::Matrix<double,7,1> q_r_arm_ub_mat(q_r_arm_ub.data());

    const static double yumi_velocity_lb[NUM_OF_JOINTS/2] = {
        -0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10
    };
    const static double yumi_velocity_ub[NUM_OF_JOINTS/2] = {
         0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10
    };
    
    // remember to keep consistent with _robot_finger_start and _robot_finger_goal
    const std::vector<double> q_l_finger_lb = {-1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -0.75, 0.0, -0.2, -0.15};
                                                //{-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0};
    const std::vector<double> q_l_finger_ub = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.1,  0.0,  0.0};
                                                //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0};
    const std::vector<double> q_r_finger_lb = {-1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.0, 0.0, -0.2, -0.15};
                                                //{-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0};
    const std::vector<double> q_r_finger_ub = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3, 0.1,  0.0,  0.0}; 
                                                //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0};

    const static double ROBOTHAND_LB[NUM_OF_FINGER_JOINTS/2] = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.1,  0.0,  0.0};
    const static double ROBOTHAND_UB[NUM_OF_FINGER_JOINTS/2] = {-1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.0, 0.0, -0.2, -0.15};
    // onst static double ROBOTHAND_LB[NUM_OF_FINGER_JOINTS/2] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0};
    // const static double ROBOTHAND_UB[NUM_OF_FINGER_JOINTS/2] = {-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0};
    
    const static double WISEGLOVE_LB[NUM_OF_GLOVE_ANGLES] = {0,  0,   53,  0,   0,   22,  0,   0,   22,  0,  0,   35,  0,  0};
    const static double WISEGLOVE_UB[NUM_OF_GLOVE_ANGLES] = {45, 100, 0,   90,  120, 0,   90,  120, 0,   90, 120, 0,   90, 120};
    const static double q_pos_lb[JOINT_DOF] = {
        -2.94,-2.50,-2.94,-2.15,-5.06,-1.53,-3.99,
        -2.94,-2.50,-2.94,-2.15,-5.06,-1.53,-3.99,
        -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -0.75, 0.0, -0.2, -0.15,
        -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.0, 0.0, -0.2, -0.15
    };
    const static double q_pos_ub[JOINT_DOF] = {
        2.94,0.75,2.94,1.39,5.06,2.40,3.99,
        2.94,0.75,2.94,1.39,5.06,2.40,3.99,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.1,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.1,  0.0,  0.0
    };

    // Error bound
    const static double linear_bound = 0.02;
    const static double linear_bound_elbow = 0.1;
    const static double tracking_pos_err_bound = sqrt(3*linear_bound*linear_bound);
    const static double tracking_pos_err_bound_elbow = sqrt(3*linear_bound_elbow*linear_bound_elbow);
    const static double tracking_ori_err_bound = 1.0*M_PI/180.0;

    // Robot description file
    const static std::string urdf_file_name =  "/home/liweijie/projects/motion-retargeting-optimize/yumi_description/yumi_with_hands.urdf";
    const static std::string srdf_file_name = "/home/liweijie/projects/motion-retargeting-optimize/yumi_description/yumi.srdf";

}

namespace cfg{
    // Struct for accounting the tracking err of arms
    struct ArmCost
    {
        double l_wrist_pos_err;
        double r_wrist_pos_err;
        double l_wrist_ori_err;
        double r_wrist_ori_err;
        double l_elbow_pos_err;
        double r_elbow_pos_err;
        double return_total_arm_cost() {
            return l_wrist_pos_err + r_wrist_pos_err + l_wrist_ori_err + r_wrist_ori_err +
                            l_elbow_pos_err + r_elbow_pos_err;
        }
    };

    // Struct as the reference pose of wrist, elbow & fingers
    struct ReferencePose 
    {
        Eigen::Vector3d y_lw;
        Eigen::Vector3d y_rw;
        Eigen::Vector3d y_le;
        Eigen::Vector3d y_re;
        Eigen::Quaterniond quat_lw;
        Eigen::Quaterniond quat_rw;
        Eigen::Matrix<double,NUM_OF_FINGER_JOINTS/2,1> q_finger_robot_goal_l;
        Eigen::Matrix<double,NUM_OF_FINGER_JOINTS/2,1> q_finger_robot_goal_r;
    };

    struct DMP_trajs
    {
        // size is 3 x N
        Eigen::MatrixXd y_lrw;
        Eigen::MatrixXd y_lew;
        Eigen::MatrixXd y_rew;
        Eigen::MatrixXd y_rw; // right wrist
        
        Eigen::MatrixXd y_re; // right elbow
        Eigen::MatrixXd y_lw; // left wrist
        Eigen::MatrixXd y_le; // left elbow

    };
}

#endif