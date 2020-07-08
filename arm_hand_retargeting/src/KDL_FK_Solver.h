// ROS
#include <ros/ros.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

// For KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Common
#include <vector>
#include <string>

using namespace Eigen;


class KDL_FK_Solver
{
  public:
    KDL_FK_Solver();
    ~KDL_FK_Solver(){};
    KDL::ChainFkSolverPos_recursive setup_left_kdl();
    KDL::ChainFkSolverPos_recursive setup_right_kdl();

    // Perform FK
    Vector3d return_wrist_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, bool left_or_right);
    Vector3d return_elbow_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, bool left_or_right);
    Vector3d return_shoulder_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, bool left_or_right);
    Matrix3d return_wrist_ori(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, bool left_or_right);

  private:

    // params
    const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";

    // Link IDs
    unsigned int l_num_wrist_seg = 0;
    unsigned int l_num_elbow_seg = 0;
    unsigned int l_num_shoulder_seg = 0;    

    unsigned int r_num_wrist_seg = 0;
    unsigned int r_num_elbow_seg = 0;
    unsigned int r_num_shoulder_seg = 0;

    // FK solvers
    //KDL::ChainFkSolverPos_recursive left_fk_solver;
    //KDL::ChainFkSolverPos_recursive right_fk_solver;


};