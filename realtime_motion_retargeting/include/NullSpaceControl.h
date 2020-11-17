#ifndef  NULLSPACE_CONTROL_H_
#define  NULLSPACE_CONTROL_H_
// My header files
#include "config.h"
#include "inverseKinematics.h"
#include "transform_matrix_helper.h"

using namespace Eigen;
using namespace matrix_helper;
using namespace cfg;

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
            Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_initial;
            Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_initial;
            // (2) x
            matrix_helper::Pose6d xl1;  // Goal
            matrix_helper::Pose6d xr1;
            matrix_helper::Pose6d xl;   // Current
            matrix_helper::Pose6d xr;
            Eigen::Matrix<double,WRIST_DOF,1> dxl;  // Derivative
            Eigen::Matrix<double,WRIST_DOF,1> dxr;
            // (3) xe
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
            std::vector<Eigen::Matrix<double,NUM_OF_JOINTS,1>> joint_angle_traj;
            // (7) the variable for total error
            double total_wrist_pos_err = 0;
            double total_wrist_ori_err = 0;
            double total_elbow_pos_err = 0;
            // Param setting
            const int MIN_ITER = 1;
            const int MAX_ITER = 10;

    public:
        NullSpaceControl() {
            joint_angle_traj.resize(NUM_DATAPOINTS);
            left_joint_angle_traj.resize(NUM_DATAPOINTS);
            right_joint_angle_traj.resize(NUM_DATAPOINTS);
        }
        void get_hand_pos(Matrix<double,3,NUM_DATAPOINTS> hd_pos, Pose6d &x, int i);
        void get_hand_ori(Matrix<double,NUM_DATAPOINTS,4> hd_quat, Pose6d &x, int i);
        void get_elbow_pos(Matrix<double,3,NUM_DATAPOINTS> elbow_pos, Eigen::Matrix<double,ELBOW_DOF,1> &x, int i);
        Eigen::Matrix<double,NUM_OF_JOINTS,1> solve_one_step(
                                    Eigen::Vector3d l_wrist_pos_tgy,Eigen::Vector3d r_wrist_pos_tgy,
                                    Eigen::Quaterniond l_wrist_ori_tgt,Eigen::Quaterniond r_wrist_ori_tgt,
                                    Eigen::Vector3d l_elbow_pos_tgt,Eigen::Vector3d r_elbow_pos_tgt,
                                    Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_in,Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_in);
        std::vector<std::vector<double>> left_joint_angle_traj;
        std::vector<std::vector<double>> right_joint_angle_traj;
};

Eigen::Matrix<double,NUM_OF_JOINTS,1> NullSpaceControl::solve_one_step(
                                    Eigen::Vector3d l_wrist_pos_tgt,Eigen::Vector3d r_wrist_pos_tgt,
                                    Eigen::Quaterniond l_wrist_ori_tgt,Eigen::Quaterniond r_wrist_ori_tgt,
                                    Eigen::Vector3d l_elbow_pos_tgt,Eigen::Vector3d r_elbow_pos_tgt,
                                    Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_in,Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_in) 
{
    // Init ql & qr
    ql = ql_in;
    qr = qr_in;
    // Init x & xe
    kinematics::Result res = kinematics::yumi_forward_kinematics(concat_joint_angles(ql,qr));
    kinematics::Result2x(res,xl,xr);
    kinematics::Result2xe(res,xel,xer);

    Pose6d xls;
    Pose6d xrs;
    Eigen::Vector3d xels;
    Eigen::Vector3d xers;
    Pose6d xlt;
    Pose6d xrt;
    Eigen::Vector3d xelt;
    Eigen::Vector3d xert;

    // Fetch the start x
    xls = xl;
    xrs = xr;
    xels = xel;
    xers = xer;

    // Fetch the target x
    xlt.t = l_wrist_pos_tgt;
    xlt.R = l_wrist_ori_tgt;
    xrt.t = r_wrist_pos_tgt;
    xrt.R = r_wrist_ori_tgt;
    xelt = l_elbow_pos_tgt;
    xert = r_elbow_pos_tgt;
    
    // Execute inverse kinematics 
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    for (int n = 0; n < cfg::nbInterval; n++)
    {
        double t = double(n+1) / double(cfg::nbInterval);
        Pose6d xl0 = xl;
        Pose6d xr0 = xr;
        Eigen::Vector3d xel0 = xel;
        Eigen::Vector3d xer0 = xer;

        xl1 = inverseKinematics::interpolate_between_pose6d(xls,xlt,t);
        xr1 = inverseKinematics::interpolate_between_pose6d(xrs,xrt,t);
        xel1 = inverseKinematics::interpolate_between_pos3d(xels,xelt,t);
        xer1 = inverseKinematics::interpolate_between_pos3d(xers,xert,t);

        for (int j = 0; (j < MAX_ITER) & ( (j < MIN_ITER) || (tracking_pos_err(xl.t,xl1.t)>tracking_pos_err_bound) || (tracking_ori_err_with_quat(xl.R,xl1.R)>tracking_ori_err_bound) 
                || (tracking_pos_err(xr.t,xr1.t)>tracking_pos_err_bound) || (tracking_ori_err_with_quat(xr.R,xr1.R)>tracking_ori_err_bound) 
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
            if(cfg::isDamped) {
                Jl_plus = inverseKinematics::plus_inverse(Jl,cfg::lambda);
                Jr_plus = inverseKinematics::plus_inverse(Jr,cfg::lambda);
            }
            else {
                Jl_plus = inverseKinematics::pinv(Jl);
                Jr_plus = inverseKinematics::pinv(Jr);
            }
            Nl = inverseKinematics::null_space_projection(Jl,Jl_plus);
            Nr = inverseKinematics::null_space_projection(Jr,Jr_plus);
            Jel_tilda = Jel * Nl;
            Jer_tilda = Jer * Nr;
            if(cfg::isDamped) {
                Jel_plus = inverseKinematics::plus_inverse(Jel_tilda,cfg::lambda);
                Jer_plus = inverseKinematics::plus_inverse(Jer_tilda,cfg::lambda);
            }
            else {
                Jel_plus = inverseKinematics::pinv(Jel_tilda);
                Jer_plus = inverseKinematics::pinv(Jer_tilda);
            }
            // 4. Calculate dq & dx
            // (2) Based on task priority
            dql = Jl_plus*dxl + K_NULLSPACE_ELBOW * Nl * Jel_plus * (dxel - Jel*Jl_plus*dxl);
            dqr = Jr_plus*dxr + K_NULLSPACE_ELBOW * Nr * Jer_plus * (dxer - Jer*Jr_plus*dxr);
            // 5. Update q
            ql = ql + dql;
            qr = qr + dqr;
            matrix_helper::clamp_to_joint_limits(ql,YUMI_LOWER_LIMITS,YUMI_UPPER_LIMITS);
            matrix_helper::clamp_to_joint_limits(qr,YUMI_LOWER_LIMITS,YUMI_UPPER_LIMITS);
            // 6. Run forward kinematics & transform result to x
            res = kinematics::yumi_forward_kinematics(concat_joint_angles(ql,qr));
            kinematics::Result2x(res,xl,xr);
            kinematics::Result2xe(res,xel,xer);
        }
    }
    if((tracking_pos_err(xl.t,xl1.t)>tracking_pos_err_bound) || (tracking_ori_err_with_quat(xl.R,xl1.R)>tracking_ori_err_bound) 
                || (tracking_pos_err(xel,xel1)>tracking_pos_err_bound_elbow))
    {
        ql = ql_in;
    }
    if((tracking_pos_err(xr.t,xr1.t)>tracking_pos_err_bound) || (tracking_ori_err_with_quat(xr.R,xr1.R)>tracking_ori_err_bound) 
                || (tracking_pos_err(xer,xer1)>tracking_pos_err_bound_elbow))
    {
        qr = qr_in;
    }
    else return concat_joint_angles(ql,qr);
}

void NullSpaceControl::get_hand_pos(Matrix<double,3,NUM_DATAPOINTS> hd_pos, Pose6d &x, int i) {
    // x.t = hd_pos.block(0,i,3,1);
    Vector3d pos = hd_pos.block(0,i,3,1);
    x.t(0,0) = pos(0,0);
    x.t(1,0) = pos(1,0);
    x.t(2,0) = pos(2,0);
}

void NullSpaceControl::get_hand_ori(Matrix<double,NUM_DATAPOINTS,4> hd_quat, Pose6d &x, int i) {
    Vector4d hd_quat_i = hd_quat.block(i,0,1,4).transpose();
    Eigen::Quaternion<double> quat(Vector4d(hd_quat_i[1],hd_quat_i[2],hd_quat_i[3],hd_quat_i[0]));
    x.R = quat;
}

void NullSpaceControl::get_elbow_pos(Matrix<double,3,NUM_DATAPOINTS> elbow_pos, Eigen::Matrix<double,ELBOW_DOF,1> &x, int i) {
    x = elbow_pos.block(0,i,3,1);
}

#endif