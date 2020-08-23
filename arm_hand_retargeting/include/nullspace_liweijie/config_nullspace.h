#ifndef CONFIG_NULLSPACE_H
#define CONFIG_NULLSPACE_H
#include <iostream>
#include <string>
#include <cmath>
using namespace std;

// namespace cfg{
    const static int LENGTH = 50;
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
    const static double K_ELBOW_FK_COST = 0.0;
    const static double K_WRIST_FK_COST = 100.0;
    const static double K_CONSTRAINTS_COST = 1.0;

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

    const static double YUMI_LOWER_LIMITS[NUM_OF_JOINTS] = 
            {
                -2.94087978961,
                -2.50454747661,
                -2.94087978961,
                -2.15548162621,
                -5.06145483078,
                -1.53588974176,
                -3.99680398707,
                -2.94087978961,
                -2.50454747661,
                -2.94087978961,
                -2.15548162621,
                -5.06145483078,
                -1.53588974176,
                -3.99680398707
            };
    const static double YUMI_UPPER_LIMITS[NUM_OF_JOINTS] = 
        {
            2.94087978961,
            0.759218224618,
            2.94087978961,
            1.3962634016,
            5.06145483078,
            2.40855436775,
            3.99680398707,
            2.94087978961,
            0.759218224618,
            2.94087978961,
            1.3962634016,
            5.06145483078,
            2.40855436775,
            3.99680398707
        };
        const static double ROBOTHAND_LB[NUM_OF_FINGER_JOINTS/2] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0};
        const static double ROBOTHAND_UB[NUM_OF_FINGER_JOINTS/2] = {-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0};
        const static double WISEGLOVE_LB[NUM_OF_GLOVE_ANGLES] = {0, 0, 53, 0, 0, 22, 0, 0, 22, 0, 0, 35, 0, 0};
        const static double WISEGLOVE_UB[NUM_OF_GLOVE_ANGLES] = {45, 100, 0, 90, 120, 0, 90, 120, 0, 90, 120, 0, 90, 120};

        // Error bound
        const static double linear_bound = 0.02;
        const static double linear_bound_elbow = 0.1;
        const static double tracking_pos_err_bound = 3*linear_bound*linear_bound;
        const static double tracking_pos_err_bound_elbow = 3*linear_bound_elbow*linear_bound_elbow;
        const static double tracking_ori_err_bound = 1.0*M_PI/180.0;

        // Robot description file
        const static std::string urdf_file_name =  "/home/liweijie/sign_language_robot_ws/yumi_with_hands.urdf";
        const static std::string srdf_file_name = "/home/liweijie/sign_language_robot_ws/src/yumi_sign_language_robot_moveit_config/config/yumi.srdf";
// }

#endif