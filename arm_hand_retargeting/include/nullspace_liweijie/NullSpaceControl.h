
// My header files
#include "inverseKinematics.h"
// #include "h5_io.h"
// #include "util_functions.h"
#include "config_nullspace.h"

// Vertices
// #include "vertices/DualArmDualHandVertex.h"
#include <vector>

using namespace std;

void get_hand_pos(const vector<vector<double>> hd_pos, Eigen::Matrix<double,WRIST_DOF,1> &x, int i) {
    vector<double> hd_pos_i = hd_pos[i];
    x(0,0) = hd_pos_i[0];
    x(1,0) = hd_pos_i[1];
    x(2,0) = hd_pos_i[2];
}

void get_hand_ori(const vector<vector<double>> hd_quat, Eigen::Matrix<double,WRIST_DOF,1> &x, int i) {
    vector<double> hd_quat_i = hd_quat[i];
    Eigen::Quaternion<double> quat(Vector4d(hd_quat_i[1],hd_quat_i[2],hd_quat_i[3],hd_quat_i[0])); // x,y,z,w
    x.block(3,0,3,1) = quat.matrix().eulerAngles(0,1,2);
}

void get_elbow_pos(const vector<vector<double>> elbow_pos, Eigen::Matrix<double,ELBOW_DOF,1> &x, int i) {
    vector<double> elbow_pos_i = elbow_pos[i];
    x(0,0) = elbow_pos_i[0];
    x(1,0) = elbow_pos_i[1];
    x(2,0) = elbow_pos_i[2];
}

class NullSpaceControl
{
        // Member variables
        private:
            // (1) q
            double ql_data[NUM_OF_JOINTS/2] = {
                -1.5,-1.5,1.5,0.0,0.0,0.0,0.0
            };
            double qr_data[NUM_OF_JOINTS/2] = {
                1.5,-1.5,-1.5,0.0,0.0,0.0,0.0
            };
            Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,1> dql;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,1> dqr;
            // (2) x
            Eigen::Matrix<double,WRIST_DOF,1> xl0;  // Start
            Eigen::Matrix<double,WRIST_DOF,1> xr0;
            Eigen::Matrix<double,WRIST_DOF,1> xl1;  // Goal
            Eigen::Matrix<double,WRIST_DOF,1> xr1;
            Eigen::Matrix<double,WRIST_DOF,1> xl;   // Current
            Eigen::Matrix<double,WRIST_DOF,1> xr;
            Eigen::Matrix<double,WRIST_DOF,1> dxl;  // Derivative
            Eigen::Matrix<double,WRIST_DOF,1> dxr;
            // (3) xe
            Eigen::Matrix<double,ELBOW_DOF,1> xel0;
            Eigen::Matrix<double,ELBOW_DOF,1> xer0;
            Eigen::Matrix<double,ELBOW_DOF,1> xel1;
            Eigen::Matrix<double,ELBOW_DOF,1> xer1;
            Eigen::Matrix<double,ELBOW_DOF,1> xel;
            Eigen::Matrix<double,ELBOW_DOF,1> xer;
            Eigen::Matrix<double,ELBOW_DOF,1> dxel;
            Eigen::Matrix<double,ELBOW_DOF,1> dxer;
            // (5) J
            Eigen::Matrix<double,WRIST_DOF,NUM_OF_JOINTS/2> Jl;
            Eigen::Matrix<double,WRIST_DOF,NUM_OF_JOINTS/2> Jr;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,WRIST_DOF> Jl_plus;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,WRIST_DOF> Jr_plus;
            Eigen::Matrix<double,ELBOW_DOF,NUM_OF_JOINTS/2> Jel;
            Eigen::Matrix<double,ELBOW_DOF,NUM_OF_JOINTS/2> Jer;
            Eigen::Matrix<double,ELBOW_DOF,NUM_OF_JOINTS/2> Jel_tilda;
            Eigen::Matrix<double,ELBOW_DOF,NUM_OF_JOINTS/2> Jer_tilda;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,ELBOW_DOF> Jel_plus;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,ELBOW_DOF> Jer_plus;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,NUM_OF_JOINTS/2> Nl;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,NUM_OF_JOINTS/2> Nr;
            // (6) joint angle traj
            std::vector<Eigen::Matrix<double,NUM_OF_JOINTS,1> > joint_angle_traj;
            // (7) the variable for total error
            double total_wrist_pos_err = 0;
            double total_wrist_ori_err = 0;
            double total_elbow_pos_err = 0;
            // Param setting
            const int MIN_ITER = 1;
            const int MAX_ITER = 10;

    public:
        std::vector<Eigen::Matrix<double,NUM_OF_JOINTS,1>> solve_joint_trajectories(vector<vector<double>> l_hd_pos,vector<vector<double>> r_hd_pos,vector<vector<double>> l_hd_quat,vector<vector<double>> r_hd_quat,
                                                vector<vector<double>> l_fr_pos,vector<vector<double>> r_fr_pos,Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_in,Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_in) 
        {
            std::vector<Eigen::Matrix<double,NUM_OF_JOINTS,1> > joint_angle_traj_tmp(LENGTH);
            joint_angle_traj = joint_angle_traj_tmp;

            // Init ql & qr
            ql = ql_in;
            qr = qr_in;
            joint_angle_traj[0] = concat_joint_angles(ql,qr);
            // Init x & xe
            kinematics::Result res = kinematics::yumi_forward_kinematics(concat_joint_angles(ql,qr));
            kinematics::Result2x(res,xl,xr);
            kinematics::Result2xe(res,xel,xer);
            std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
            for (int i = 1; i < LENGTH; i++)
            {
                // Fetch the target x
                get_hand_pos(l_hd_pos,xl1,i);
                get_hand_pos(r_hd_pos,xr1,i);
                get_hand_ori(l_hd_quat,xl1,i);
                get_hand_ori(r_hd_quat,xr1,i);
                get_elbow_pos(l_fr_pos,xel1,i);
                get_elbow_pos(r_fr_pos,xer1,i);
                // Execute inverse kinematics 
                std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
                for (int j = 0; (j < MAX_ITER) & ( (j < MIN_ITER) || (tracking_pos_err(xl.block(0,0,3,1),xl1.block(0,0,3,1))>tracking_pos_err_bound) || (tracking_ori_err_with_ea(xl.block(3,0,3,1),xl1.block(3,0,3,1))>tracking_ori_err_bound) 
                        || (tracking_pos_err(xr.block(0,0,3,1),xr1.block(0,0,3,1))>tracking_pos_err_bound) || (tracking_ori_err_with_ea(xr.block(3,0,3,1),xr1.block(3,0,3,1))>tracking_ori_err_bound) 
                        || (tracking_pos_err(xel,xel1)>tracking_pos_err_bound_elbow) || (tracking_pos_err(xer,xer1)>tracking_pos_err_bound_elbow) ); j++)
                {
                    // 1. Calculate dx & dxe
                    inverseKinematics::calculate_dx(dxl,xl,xl1);
                    inverseKinematics::calculate_dx(dxr,xr,xr1);
                    inverseKinematics::calculate_dxe(dxel,xel,xel1);
                    inverseKinematics::calculate_dxe(dxer,xer,xer1);
                    // 2. Scale dx
                    // double scale = (0.001*j>1?1:0.001*j);
                    double scale = 1;
                    dxl = scale*dxl;
                    dxr = scale*dxr;
                    // 3. Calculate the J & J+ & other intermediate variables
                    inverseKinematics::calculate_jacobian(concat_joint_angles(ql,qr),Jl,Jr);
                    inverseKinematics::calculate_jacobian_for_elbow(concat_joint_angles(ql,qr),Jel,Jer);
                    Jl_plus = inverseKinematics::pinv(Jl);
                    Jr_plus = inverseKinematics::pinv(Jr);
                    Nl = inverseKinematics::null_space_projection(Jl,Jl_plus);
                    Nr = inverseKinematics::null_space_projection(Jr,Jr_plus);
                    Jel_tilda = Jel * Nl;
                    Jer_tilda = Jer * Nr;
                    Jel_plus = inverseKinematics::pinv(Jel_tilda);
                    Jer_plus = inverseKinematics::pinv(Jer_tilda);
                    // 4. Calculate dq & dx
                    // (1) Single task
                    // dql = Jl_plus*dxl;
                    // dqr = Jr_plus*dxr;
                    // (2) Based on task priority
                    double elbow_scale = 0.01;// * 0.01 * 0.1; // scale down the importance of elbow tracking, yet it's still not good as LM - 2020/08/23, LYW
                    dql = Jl_plus*dxl + elbow_scale * Nl * Jel_plus * (dxel - Jel*Jl_plus*dxl);
                    dqr = Jr_plus*dxr + elbow_scale * Nr * Jer_plus * (dxer - Jer*Jr_plus*dxr);
                    // 5. Update q
                    ql = ql + dql;
                    qr = qr + dqr;
                    inverseKinematics::clamp_to_joint_limits(ql,YUMI_LOWER_LIMITS,YUMI_UPPER_LIMITS);
                    inverseKinematics::clamp_to_joint_limits(qr,YUMI_LOWER_LIMITS,YUMI_UPPER_LIMITS);
                    // 6. Run forward kinematics & transform result to x
                    res = kinematics::yumi_forward_kinematics(concat_joint_angles(ql,qr));
                    kinematics::Result2x(res,xl,xr);
                    kinematics::Result2xe(res,xel,xer);
                }
                // Print segment line
                // cout<<"/////////////////////////////////////---------Solving Finished----------///////////////////////////////"<<endl;
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
                // std::cout << "Point " << i << "\tInverse kinematics spent " << duration.count()  << " s." << std::endl; 
                // Print the position after iteration
                // cout<< "Point " << i <<"\t Current wrist pose: "<< xl.transpose() <<endl;
                // cout<< "Point " << i << "\t Target wrist pose: "<< xl1.transpose() <<endl;
                // cout<< "Point " << i <<"\t Current elbow position: "<< xel.transpose() <<endl;
                // cout<< "Point " << i << "\t Target elbow position: "<< xel1.transpose() <<endl;
                // cout<< "Point " << i <<"\t Current joint angles: "<< concat_joint_angles(ql,qr).transpose() <<endl;
                // Print tracking error
                // cout<<"tracking wrist pos err:"<<tracking_pos_err(xl.block(0,0,3,1),xl1.block(0,0,3,1))<<endl;
                // cout<<"tracking wrist ori err:"<<tracking_ori_err_with_ea(xl.block(3,0,3,1),xl1.block(3,0,3,1))<<endl;
                // cout<<"tracking elbow pos err:"<<tracking_pos_err(xel,xel1)<<endl;
                // Save the solution result of joint angles
                joint_angle_traj[i] = concat_joint_angles(ql,qr);
                // Account the total tracking error
                total_wrist_pos_err += tracking_pos_err(xl.block(0,0,3,1),xl1.block(0,0,3,1));
                total_wrist_ori_err += tracking_ori_err_with_ea(xl.block(3,0,3,1),xl1.block(3,0,3,1));
                total_elbow_pos_err += tracking_pos_err(xel,xel1);
            }
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
            // std::cout <<  "The whole null space control process spent " << duration.count()  << " s." << std::endl; 
            // std::cout <<  "The total tracking error of wrist position: " << total_wrist_pos_err  << std::endl; 
            // std::cout <<  "The total tracking error of wrist orientation: " << total_wrist_ori_err  << std::endl; 
            // std::cout <<  "The total tracking error of elbow position: " << total_elbow_pos_err  << std::endl; 
            return joint_angle_traj;
        }
};