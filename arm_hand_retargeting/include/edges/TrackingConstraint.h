#ifndef TRACKING_CONSTRAINT_H
#define TRACKING_CONSTRAINT_H

// Common
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

// For acos, fabs, pow
#include <cmath>

// G2O: infrastructure
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/QR>

// For KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// G2O Vertices
#include "vertices/DMPStartsGoalsVertex.h" 
#include "vertices/DualArmDualHandVertex.h"

// Self-defined helper variables
#include "config.h"

using namespace g2o;
using namespace Eigen;


/** 
 * @brief Define constraint for computing tracking error.
 * 
 * To reduce the number of queries of DMP trajectory generator, we combine all q vertices in a TrackingConstraint edge.
 * Try using vector error for tracking cost. wrist pos (3 x 2) + wrist ori (3 x 2) + elbow pos (3 x 2) + finger (1 x 2) = 20. \n
 * Error vector is in the order: l wrist pos -> l wrist ori -> l elbow pos -> r wrist pos -> r wrist ori 
 * -> r elbow pos -> l finger (1d) -> r finger (1d).
 * /
/* addVertex rule: connect DMP starts and goals vertex first, then datapoints */
class TrackingConstraint : public BaseBinaryEdge<20, double, DMPStartsGoalsVertex, DualArmDualHandVertex> // <D, E>, dimension and measurement datatype
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor of TrackingConstraint. \n
     * Initialize with left and right KDL FK solver, collision checker and DMP trajectory generator. \n
     * Use list initialization for reference member.
     * @param[in]     point_id      Indicate the ID of the current path point. (starts from 1 and ends at NUM_DATAPOINTS)
     */
    TrackingConstraint(boost::shared_ptr<DMPTrajectoryGenerator> &_trajectory_generator_ptr, 
                      KDL::ChainFkSolverPos_recursive &_left_fk_solver, 
                      KDL::ChainFkSolverPos_recursive &_right_fk_solver, 
                      boost::shared_ptr<DualArmDualHandCollision> &_dual_arm_dual_hand_collision_ptr,
                      int point_id) : trajectory_generator_ptr(_trajectory_generator_ptr), 
                                                     left_fk_solver(_left_fk_solver), 
                                                     right_fk_solver(_right_fk_solver), 
                                                     dual_arm_dual_hand_collision_ptr(_dual_arm_dual_hand_collision_ptr)
    {
      this->point_id = point_id;
    }

    /// Pass in user data, e.g. wrist ID for KDL chain
    void set_user_data(my_constraint_struct _fdata)
    {
      this->fdata = _fdata;
    }

    // functions for error vector computation
    Vector3d compute_wrist_pos_error(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, bool left_or_right, my_constraint_struct &fdata);
    Vector3d compute_wrist_ori_error(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, bool left_or_right, my_constraint_struct &fdata);
    Vector3d compute_elbow_pos_error(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);

    void computeError();

    // functions for returning cost values (overloaded to provide easy access)
    double return_wrist_pos_cost(bool left_or_right, bool both);
    double return_wrist_ori_cost(bool left_or_right, bool both);
    double return_elbow_pos_cost(bool left_or_right, bool both);
    double return_finger_cost(bool left_or_right, bool both);

    // provide information for manual manipulation of DMP starts and goals during in-turn optimization (overloaded to provide easy access)   
    Vector3d return_wrist_pos_offset(bool left_or_right);
    Vector3d return_elbow_pos_offset(bool left_or_right);

    // return the wrist, elbow positions of the current q joint values, for recording Cartesian trajectories (overloaded to provide easy access)
    Vector3d return_wrist_pos(bool left_or_right);
    Vector3d return_elbow_pos(bool left_or_right);
  
    // return Jacobians results for debug
    MatrixXd output_dmp_jacobian();
    Matrix<double, 20, JOINT_DOF> output_q_jacobian();

    /// Re-implement numeric differentiation
    virtual void linearizeOplus();

    // Directly map human joint angle data to robot hand
    static Matrix<double, 12, 1> map_finger_joint_values(Matrix<double, 14, 1> q_finger_human, bool left_or_right, my_constraint_struct fdata);

    /// Read from disk, leave blank
    virtual bool read( std::istream& in ) {return true;}
    
    /// Write to disk, leave blank
    virtual bool write( std::ostream& out ) const {return true;}


  private:
    /// (DMP) Trajectory generator
    boost::shared_ptr<DMPTrajectoryGenerator> &trajectory_generator_ptr;

    // KDL solvers
    KDL::ChainFkSolverPos_recursive &left_fk_solver;      ///< KDL FK solver for left arm.
    KDL::ChainFkSolverPos_recursive &right_fk_solver;     ///< KDL FK solver for right arm.

    /// Collision Checker, which constains RobotState that can be used to compute jacobians, global link transforms, etc.
    boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr;

    /// Helper function. Perform linear mapping based on upper and lower bounds
    static double linear_map(double x_, double min_, double max_, double min_hat, double max_hat);

    // numbers
    int point_id; ///< Internal variable, indicates the ID of the current path point, for locating the corresponding goal positions in DMP results.
    my_constraint_struct fdata;   ///< User data, pass in user-defined data, such as goal orientation and goal finger angles.

    // Overloaded functions for calculating tracking costs which are used for evaluation.
    double return_wrist_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    double return_wrist_ori_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    double return_elbow_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);

    // Overloaded functions for obtaining the actual positions of wrists and elbows.
    Vector3d return_wrist_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    Vector3d return_elbow_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
  
    /// Helper function to compute finger cost (used in computeErro(), linearizeOplus() and return_finger_cost()).
    double compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct &fdata);

    // Provide information for manually moving DMP starts and goals  
    Vector3d return_wrist_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    Vector3d return_elbow_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);

    // For storing jacobian information 
    Matrix<double, 20, DMPPOINTS_DOF> jacobians_for_dmp;
    Matrix<double, 20, JOINT_DOF> jacobians_for_q;

    // internal variables for storing robot jacobians for left and right wrists as well as elbows
    MatrixXd J_lw, J_le, J_rw, J_re;

    /// Scaling factor for tracking jacobians for DMP
    double dmp_jacobian_scale = 0.0; // do not use tracking constraint to update DMP... since the error term is actually modified nullspace formulation

};


/**
 * Do linear mapping on finger joints for use in computing finger position cost.
 */
double TrackingConstraint::linear_map(double x_, double min_, double max_, double min_hat, double max_hat)
{
  return (x_ - min_) / (max_ - min_) * (max_hat - min_hat) + min_hat;
}


/**
 * @brief Directly map human finger data to robot hands. 
 * 
 * Result is used as an initial setup for finger part.
 */
Matrix<double, 12, 1> TrackingConstraint::map_finger_joint_values(Matrix<double, 14, 1> q_finger_human, bool left_or_right, my_constraint_struct fdata)
{
  // Obtain required data and parameter settings
  Matrix<double, 14, 1> human_finger_start = fdata.glove_start;
  Matrix<double, 14, 1> human_finger_final = fdata.glove_final;
  Matrix<double, 12, 1> robot_finger_start, robot_finger_final;
  if (left_or_right)
  {
    // Get bounds
    robot_finger_start = fdata.l_robot_finger_start;
    robot_finger_final = fdata.l_robot_finger_final;
  }
  else
  {
    // Get bounds
    robot_finger_start = fdata.r_robot_finger_start;
    robot_finger_final = fdata.r_robot_finger_final;
  }

  // Direct mapping and linear scaling
  Matrix<double, 12, 1> q_finger_robot_goal;
  q_finger_robot_goal[0] = linear_map(q_finger_human[3], human_finger_start[3], human_finger_final[3], robot_finger_start[0], robot_finger_final[0]);
  q_finger_robot_goal[1] = linear_map(q_finger_human[4], human_finger_start[4], human_finger_final[4], robot_finger_start[1], robot_finger_final[1]);
  q_finger_robot_goal[2] = linear_map(q_finger_human[6], human_finger_start[6], human_finger_final[6], robot_finger_start[2], robot_finger_final[2]);
  q_finger_robot_goal[3] = linear_map(q_finger_human[7], human_finger_start[7], human_finger_final[7], robot_finger_start[3], robot_finger_final[3]);
  q_finger_robot_goal[4] = linear_map(q_finger_human[9], human_finger_start[9], human_finger_final[9], robot_finger_start[4], robot_finger_final[4]);
  q_finger_robot_goal[5] = linear_map(q_finger_human[10], human_finger_start[10], human_finger_final[10], robot_finger_start[5], robot_finger_final[5]);
  q_finger_robot_goal[6] = linear_map(q_finger_human[12], human_finger_start[12], human_finger_final[12], robot_finger_start[6], robot_finger_final[6]);
  q_finger_robot_goal[7] = linear_map(q_finger_human[13], human_finger_start[13], human_finger_final[13], robot_finger_start[7], robot_finger_final[7]);
  q_finger_robot_goal[8] = (robot_finger_start[8] + robot_finger_final[8]) / 2.0;
  q_finger_robot_goal[9] = linear_map(q_finger_human[2], human_finger_start[2], human_finger_final[2], robot_finger_start[9], robot_finger_final[9]);
  q_finger_robot_goal[10] = linear_map(q_finger_human[0], human_finger_start[0], human_finger_final[0], robot_finger_start[10], robot_finger_final[10]);
  q_finger_robot_goal[11] = linear_map(q_finger_human[1], human_finger_start[1], human_finger_final[1], robot_finger_start[11], robot_finger_final[11]); 

  return q_finger_robot_goal;
}


/**
 * Helper function. Return the wrist position corresponding to current joint configuration 
 */
Vector3d TrackingConstraint::return_wrist_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }
 
  // Preparations
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);
  
  // return results
  return wrist_pos_cur;
}


/**
 * Helper function. Return the elbow position corresponding to current joint configuration 
 */
Vector3d TrackingConstraint::return_elbow_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
 // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    exit(-1);
  }
 
  // Preparations
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
 
  // return results
  return elbow_pos_cur;
}


/**
 * Return the current wrist position trajectory via FK for debug. (overloaded)
 */
Vector3d TrackingConstraint::return_wrist_pos(bool left_or_right)
{
  // get the current joint value
  const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x = v->estimate(); 

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  q_cur_l = x.block<7, 1>(0, 0);
  q_cur_r = x.block<7, 1>(7, 0);
  
  // Compute unary costs
  Vector3d cur_wrist_pos_vec;
  if(left_or_right) 
    cur_wrist_pos_vec = return_wrist_pos(left_fk_solver, q_cur_l, fdata.l_num_wrist_seg, fdata.l_num_elbow_seg, true, fdata);
  else
    cur_wrist_pos_vec = return_wrist_pos(right_fk_solver, q_cur_r, fdata.r_num_wrist_seg, fdata.r_num_elbow_seg, false, fdata);

  return cur_wrist_pos_vec;
}


/**
 * Return the current elbow position trajectory via FK for debug. (overloaded)
 */
Vector3d TrackingConstraint::return_elbow_pos(bool left_or_right)
{
  // get the current joint value
  const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  q_cur_l = x.block<7, 1>(0, 0);
  q_cur_r = x.block<7, 1>(7, 0);

  // Compute unary costs
  Vector3d cur_elbow_pos_vec;
  if(left_or_right) 
    cur_elbow_pos_vec = return_elbow_pos(left_fk_solver, q_cur_l, fdata.l_num_wrist_seg, fdata.l_num_elbow_seg, true, fdata);
  else
    cur_elbow_pos_vec = return_elbow_pos(right_fk_solver, q_cur_r, fdata.r_num_wrist_seg, fdata.r_num_elbow_seg, false, fdata);

  return cur_elbow_pos_vec;
}


/**
 * Output just the jacobians for DMP starts_and_goals vertex 
 */
MatrixXd TrackingConstraint::output_dmp_jacobian()
{
  return jacobians_for_dmp;
}


/**
 * Output just the jacobians for q vertices
 */
Matrix<double, 20, JOINT_DOF> TrackingConstraint::output_q_jacobian()
{
  return jacobians_for_q;
}


/**
 * @brief Used by g2o internal calculation.
 * 
 * Compute jacobians for each part separately, to reduce the time used for duplicated cost.
 */
void TrackingConstraint::linearizeOplus()
{
  double dmp_eps = 0.005; // better be small enough so that gradients on different dimensions won't diverge too much
  double q_finger_eps = 1.0 * M_PI / 180; // in radius
  Matrix<double, 20, 1> e_plus, e_minus;

  // 1 - For DMP starts and goals
  DMPStartsGoalsVertex *v = dynamic_cast<DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  if (v->fixed()) // if dmp vertex is fixed, use the data passed in, and assign zeros updates to dmp vertex
  {
    // assign zeros
    _jacobianOplusXi = Matrix<double, 20, DMPPOINTS_DOF>::Zero();
    // record
    this->jacobians_for_dmp = _jacobianOplusXi;
  }
  else
  {
    // 1 - Compute jacobians for DMP
    Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
    Matrix<double, DMPPOINTS_DOF, 1> delta_x = Matrix<double, DMPPOINTS_DOF, 1>::Zero();
    for (unsigned int n = 0; n < DMPPOINTS_DOF; n++)
    {
      // set delta
      delta_x[n] = dmp_eps;

      // assign and compute
      v->setEstimate(x+delta_x);
      this->computeError();
      e_plus = _error;

      v->setEstimate(x-delta_x);
      this->computeError();   
      e_minus = _error;

      // reset delta
      delta_x[n] = 0.0;

      // set and store jacobians 
      _jacobianOplusXi.col(n) = dmp_jacobian_scale * (e_plus - e_minus) / (2*dmp_eps); // for DMP starts and goals
     
      // store jacobians
      this->jacobians_for_dmp.col(n) = _jacobianOplusXi.col(n);

      // debug
      // std::cout << "debug: jacobians for dmp = " << this->jacobians_for_dmp.col(n).transpose() << std::endl;

    }

    // reset vertex value
    v->setEstimate(x);

    // 2 - Assign zero jacobians to q vertices
    _jacobianOplusXj = Matrix<double, 20, JOINT_DOF>::Zero(); 

    // skip the calculation of q jacobians when q vertices are fixed
    return; 
  }


  // 2 - For q vertex
  DMP_trajs result;
  if ( !(v->fixed()) ) // if dmp vertex not fixed, use it to generate trajectories
  {
    Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
    MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
    MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
    MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
    MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
    MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
    MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
    MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
    MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
    std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();  
    result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                            lew_new_goal.transpose(), lew_new_start.transpose(),
                                                            rew_new_goal.transpose(), rew_new_start.transpose(),
                                                            rw_new_goal.transpose(), rw_new_start.transpose(),
                                                            NUM_DATAPOINTS); // results are 3 x N
    std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
    std::chrono::duration<double> t_spent1 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
    total_traj += t_spent1.count();
    count_traj++;  
  }

  // Iterate to compute jacobians
  double t_arm_numeric = 0.0;
  double t_arm_jacobian = 0.0;
  Matrix<double, 7, 1> delta_q_arm = Matrix<double, 7, 1>::Zero();
  Matrix<double, 12, 1> delta_q_finger = Matrix<double, 12, 1>::Zero();  
  Matrix<double, 20, 1> e_err_vec_plus, e_err_vec_minus;  // for a single path point 
  
  // initialization
  _jacobianOplusXj = Matrix<double, 20, JOINT_DOF>::Zero() ;

  // get the current joint values
  const DualArmDualHandVertex *v_tmp = dynamic_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> q = v_tmp->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;    
  Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
  q_cur_l = q.block<7, 1>(0, 0);
  q_cur_r = q.block<7, 1>(7, 0);
  q_cur_finger_l = q.block<12, 1>(14, 0);
  q_cur_finger_r = q.block<12, 1>(26, 0); 
  
  // Set new goals(expected trajectory) to fdata
  if (v->fixed()) // if dmp vertex fixed, use data passed in
  {
    fdata.l_wrist_pos_goal = fdata.DMP_lw.block(0, point_id-1, 3, 1);
    fdata.l_elbow_pos_goal = fdata.DMP_le.block(0, point_id-1, 3, 1);
  }
  else
  {
    fdata.l_wrist_pos_goal = result.y_lw.block(0, point_id-1, 3, 1); // Vector3d
    fdata.l_elbow_pos_goal = result.y_le.block(0, point_id-1, 3, 1); // Vector3d
  }
  Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 0), 
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 1),
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 2),
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 3));
  Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
  fdata.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d
  // right arm part:
  if (v->fixed()) // if dmp vertex fixed, use data passed in
  {
    fdata.r_wrist_pos_goal = fdata.DMP_rw.block(0, point_id-1, 3, 1);
    fdata.r_elbow_pos_goal = fdata.DMP_re.block(0, point_id-1, 3, 1);
  }
  else
  {
    fdata.r_wrist_pos_goal = result.y_rw.block(0, point_id-1, 3, 1); // Vector3d
    fdata.r_elbow_pos_goal = result.y_re.block(0, point_id-1, 3, 1); // Vector3d
  }
  Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 0), 
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 1),
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 2),
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 3));
  Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
  fdata.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d
  // hand parts:
  fdata.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(point_id-1, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
  fdata.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(point_id-1, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF


  // Initialize for convenience 
  e_err_vec_plus = Matrix<double, 20, 1>::Zero();
  e_err_vec_minus = Matrix<double, 20, 1>::Zero();

  // (1) - jacobians for arms
  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();      
  // get jacobians from robotstate.... or liweijie's result...
  MatrixXd l_wrist_jacobian = this->J_lw;
  MatrixXd r_wrist_jacobian = this->J_rw;
  MatrixXd l_elbow_jacobian = this->J_le;
  MatrixXd r_elbow_jacobian = this->J_re;
  _jacobianOplusXj.block(0, 0, 6, 7) = l_wrist_jacobian;  // l wrist pos(3) + l wrist ori(3)
  _jacobianOplusXj.block(6, 0, 3, 7) = l_elbow_jacobian.topRows(3); // first three rows // l elbow pos(3)
  _jacobianOplusXj.block(9, 7, 6, 7) = r_wrist_jacobian;  // r wrist pos(3) + r wrist ori(3)
  _jacobianOplusXj.block(15, 7, 3, 7) = r_elbow_jacobian.topRows(3); // r elbow pos
  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();  
  std::chrono::duration<double> t_spent1 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  t_arm_numeric += t_spent1.count();
  
  // Already checked, concatenation is OK.
  // std::cout << "debug: l_wrist_jacobian = \n" << l_wrist_jacobian << std::endl;
  // std::cout << "debug: l_elbow_jacobian = \n" << l_elbow_jacobian << std::endl;
  // std::cout << "debug: r_wrist_jacobian = \n" << r_wrist_jacobian << std::endl;
  // std::cout << "debug: r_elbow_jacobian = \n" << r_elbow_jacobian << std::endl;
  // std::cout << "debug: jacobian for arms = \n" << _jacobianOplusXj.block(0, 0, 18, 14) << std::endl;


  // (2) - jacobians for fingers
  double e_finger_plus, e_finger_minus;
  for (unsigned int d = 0; d < 12; d++)
  {
    // set delta
    delta_q_finger[d] = q_finger_eps;

    // left hand
    e_finger_plus = compute_finger_cost(q_cur_finger_l+delta_q_finger, true, fdata);
    e_finger_minus = compute_finger_cost(q_cur_finger_l-delta_q_finger, true, fdata);
    _jacobianOplusXj(18, d+14) = (e_finger_plus - e_finger_minus) / (2 * q_finger_eps) + 1e-6; // add a small nonzero term in case it's zero?

    // right hand
    e_finger_plus = compute_finger_cost(q_cur_finger_r+delta_q_finger, false, fdata);  
    e_finger_minus = compute_finger_cost(q_cur_finger_r-delta_q_finger, false, fdata);  
    _jacobianOplusXj(19, d+26) = (e_finger_plus - e_finger_minus) / (2 * q_finger_eps) + 1e-6; // add a small nonzero term

    // reset delta
    delta_q_finger[d] = 0.0;
  }

  // 3 - Save jacobians for q vertices
  this->jacobians_for_q = _jacobianOplusXj;
}


/**
 * Helper function for computeError(). Compute error vector for wrist position.
 */
Vector3d TrackingConstraint::compute_wrist_pos_error(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);
  
  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }

  // Get the results
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);

  // Specify human data
  Vector3d wrist_pos_human = (left_or_right ? (fdata.l_wrist_pos_goal) : (fdata.r_wrist_pos_goal) );

  // Get error vector
  Vector3d wrist_pos_err_vec = wrist_pos_human - wrist_pos_cur;
  
  // store for debug 
  // this->cur_wrist_pos_cost = wrist_pos_err_vec.norm();

  // Return cost function value
  return wrist_pos_err_vec;
}


/**
 * Helper function for computeError(). Compute error vector for wrist orientation.
 */
Vector3d TrackingConstraint::compute_wrist_ori_error(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }

  // Get the results
  Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 

  // Specify human data
  Matrix3d wrist_ori_human = ( left_or_right ? (fdata.l_wrist_ori_goal) : (fdata.r_wrist_ori_goal) );

  // Get error vector (differential velocity, computed from differential movement: from R1 to R2, (R2*R1' - I) is the skew symmetric matrix that contains the differential rotation terms)
  // when R1 deviates from R2 too much, the result would not be skew-symmetric any more, and the result of doing so remains to be seen...
  Matrix3d skew_symmetric = (wrist_ori_human * wrist_ori_cur.transpose() - Matrix3d::Identity());
  Vector3d wrist_ori_err_vec;
  wrist_ori_err_vec[0] = (skew_symmetric(2, 1) - skew_symmetric(1, 2)) / 2.0;
  wrist_ori_err_vec[1] = (skew_symmetric(0, 2) - skew_symmetric(2, 0)) / 2.0;
  wrist_ori_err_vec[2] = (skew_symmetric(1, 0) - skew_symmetric(0, 1)) / 2.0;

  // Compute cost function
  double wrist_ori_cost = std::fabs( std::acos( std::min(((wrist_ori_human * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0, 1.0) ) );

  // store for debugging jacobians
  // this->cur_wrist_ori_cost = wrist_ori_cost;

  // Return cost function value
  return wrist_ori_err_vec;
}


/**
 * Helper function for computeErro(). Compute error vector for elbow position.
 */
Vector3d TrackingConstraint::compute_elbow_pos_error(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    exit(-1);
  }
 
  // Get the results
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);

  // Specify human data
  Vector3d elbow_pos_human = (left_or_right ? (fdata.l_elbow_pos_goal) : (fdata.r_elbow_pos_goal) );

  // Get error vector
  Vector3d elbow_pos_err_vec = elbow_pos_human- elbow_pos_cur; 

  // Store for debug
  // this->cur_elbow_pos_cost = elbow_pos_err_vec.norm();

  // Return cost function value
  return elbow_pos_err_vec;
}


/**
 * Internal function.
 */
double TrackingConstraint::compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct &fdata)
{
  // Obtain required data and parameter settings
  Matrix<double, 14, 1> q_finger_human;
  Matrix<double, 14, 1> human_finger_start = fdata.glove_start;
  Matrix<double, 14, 1> human_finger_final = fdata.glove_final;
  Matrix<double, 12, 1> robot_finger_start, robot_finger_final;
  if (left_or_right)
  {
    // Get the sensor data
    q_finger_human = fdata.l_finger_pos_goal;
    // Get bounds
    robot_finger_start = fdata.l_robot_finger_start;
    robot_finger_final = fdata.l_robot_finger_final;
  }
  else
  {
    // Get the sensor data
    q_finger_human = fdata.r_finger_pos_goal;    
    // Get bounds
    robot_finger_start = fdata.r_robot_finger_start;
    robot_finger_final = fdata.r_robot_finger_final;
  }

  // Direct mapping and linear scaling
  Matrix<double, 12, 1> q_finger_robot_goal;
  q_finger_robot_goal[0] = linear_map(q_finger_human[3], human_finger_start[3], human_finger_final[3], robot_finger_start[0], robot_finger_final[0]);
  q_finger_robot_goal[1] = linear_map(q_finger_human[4], human_finger_start[4], human_finger_final[4], robot_finger_start[1], robot_finger_final[1]);
  q_finger_robot_goal[2] = linear_map(q_finger_human[6], human_finger_start[6], human_finger_final[6], robot_finger_start[2], robot_finger_final[2]);
  q_finger_robot_goal[3] = linear_map(q_finger_human[7], human_finger_start[7], human_finger_final[7], robot_finger_start[3], robot_finger_final[3]);
  q_finger_robot_goal[4] = linear_map(q_finger_human[9], human_finger_start[9], human_finger_final[9], robot_finger_start[4], robot_finger_final[4]);
  q_finger_robot_goal[5] = linear_map(q_finger_human[10], human_finger_start[10], human_finger_final[10], robot_finger_start[5], robot_finger_final[5]);
  q_finger_robot_goal[6] = linear_map(q_finger_human[12], human_finger_start[12], human_finger_final[12], robot_finger_start[6], robot_finger_final[6]);
  q_finger_robot_goal[7] = linear_map(q_finger_human[13], human_finger_start[13], human_finger_final[13], robot_finger_start[7], robot_finger_final[7]);
  q_finger_robot_goal[8] = (robot_finger_start[8] + robot_finger_final[8]) / 2.0;
  q_finger_robot_goal[9] = linear_map(q_finger_human[2], human_finger_start[2], human_finger_final[2], robot_finger_start[9], robot_finger_final[9]);
  q_finger_robot_goal[10] = linear_map(q_finger_human[0], human_finger_start[0], human_finger_final[0], robot_finger_start[10], robot_finger_final[10]);
  q_finger_robot_goal[11] = linear_map(q_finger_human[1], human_finger_start[1], human_finger_final[1], robot_finger_start[11], robot_finger_final[11]); 

  // Compute cost
  double finger_scale = 0.005; //0.01; //0.05;//0.1; //0.5; //1.0;
  double finger_cost = finger_scale * (q_finger_robot_goal - q_finger_robot).norm();

  // store for debug
  // this->cur_finger_pos_cost = finger_cost;

  return finger_cost;
}


/**
 * @brief Return finger cost.
 * 
 * Actually, return the finger cost for the current path point.
 * @param[in]   LEFT_RIGHT_BOTH   Specify left or right or both hands to compute cost for.
 */
double TrackingConstraint::return_finger_cost(bool left_or_right, bool both)
{
  // Get the current joint value
  const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x = v->estimate(); 

  // Get joint angles
  Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
  if (left_or_right || both)
    q_cur_finger_l = x.block<12, 1>(14, 0);
  if (!left_or_right || both)
    q_cur_finger_r = x.block<12, 1>(26, 0); 
  
  // Set new goals(expected trajectory) to fdata
  if (left_or_right || both)
    fdata.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(point_id-1, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
  if (!left_or_right || both)
    fdata.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(point_id-1, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF

  // Compute unary costs
  double finger_cost = 0;
  if (left_or_right || both)    
    finger_cost += compute_finger_cost(q_cur_finger_l, true, fdata);  
  if (!left_or_right || both)
    finger_cost += compute_finger_cost(q_cur_finger_r, false, fdata);  

  return finger_cost;
}


/**
 * @brief Return wrist position cost by the specified flag. 
 * 
 * Overloaded to provide easy access.
 * @param[in]   left_or_right   Specify left or right to compute cost for.
 * @param[in]   both            Specify whether to compute cost for both arms.
 */
double TrackingConstraint::return_wrist_pos_cost(bool left_or_right, bool both)
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *dv = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> dx = dv->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = dx.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = dx.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = dx.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = dx.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = dx.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = dx.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = dx.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = dx.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // Get the current joint values
  const DualArmDualHandVertex *qv = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> qx = qv->estimate(); // return the current estimate of the vertex
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  if (left_or_right || both)
    q_cur_l = qx.block<7, 1>(0, 0);
  if (!left_or_right || both)
    q_cur_r = qx.block<7, 1>(7, 0);
  
  // Set new goals(expected trajectory) to fdata
  if (left_or_right || both)
    fdata.l_wrist_pos_goal = result.y_lw.block(0, point_id-1, 3, 1); // Vector3d
  if (!left_or_right || both)
    fdata.r_wrist_pos_goal = result.y_rw.block(0, point_id-1, 3, 1); // Vector3d

  // Compute unary costs
  double wrist_pos_cost = 0;
  if (left_or_right || both)
    wrist_pos_cost += return_wrist_pos_cost(left_fk_solver, q_cur_l, fdata.l_num_wrist_seg, fdata.l_num_elbow_seg, true, fdata); // user data is stored in fdata now
  if (!left_or_right || both)
    wrist_pos_cost += return_wrist_pos_cost(right_fk_solver, q_cur_r, fdata.r_num_wrist_seg, fdata.r_num_elbow_seg, false, fdata);

  return wrist_pos_cost;
}


/**
 * @brief Helper function for estimating the tracking condition. For internal use.
 * 
 * Calculate the current wrist position cost.
 */
double TrackingConstraint::return_wrist_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }

  // Preparations
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);

  Vector3d wrist_pos_human;
  if (left_or_right) // left arm
    wrist_pos_human = fdata.l_wrist_pos_goal;
  else // right arm
    wrist_pos_human = fdata.r_wrist_pos_goal;

  // Compute cost function
  double wrist_pos_cost = (wrist_pos_cur - wrist_pos_human).norm(); // _human is actually the newly generated trajectory

  // Return cost function value
  return wrist_pos_cost;
}


/**
 * @brief Return wrist orientation cost history.
 * 
 * Overloaded to provide easy access.
 * @param[in]   left_or_right   Specify left or right or both arms to compute cost for.
 * @param[in]   both            Specify whether to compute cost for both arms.
 */
double TrackingConstraint::return_wrist_ori_cost(bool left_or_right, bool both)
{
  // Get the current joint value
  const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  q_cur_l = x.block<7, 1>(0, 0);
  q_cur_r = x.block<7, 1>(7, 0);
  
  // Set new goals(expected trajectory) to fdata
  Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 0), 
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 1),
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 2),
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 3));
  Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
  fdata.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d
  Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 0), 
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 1),
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 2),
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 3));
  Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
  fdata.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d

  // Compute unary costs
  double wrist_ori_cost = 0;
  if (left_or_right || both)    
    wrist_ori_cost += return_wrist_ori_cost(left_fk_solver, q_cur_l, fdata.l_num_wrist_seg, fdata.l_num_elbow_seg, true, fdata); // user data is stored in fdata now
  if (!left_or_right || both)
    wrist_ori_cost += return_wrist_ori_cost(right_fk_solver, q_cur_r, fdata.r_num_wrist_seg, fdata.r_num_elbow_seg, false, fdata);

  return wrist_ori_cost;
}


/**
 * @brief Helper function for evaluating the tracking condition. For internal use
 * 
 * Calculate wrist orientation cost of a particular path point. Note that before calculating this, computeError() must be 
 * called at least once, because the {l/r}_wrist_ori_goal are assigned only when computeError() is called. 
 * (this term is not stored in constraint_data before passing into TrackingConstraint.)
 */
double TrackingConstraint::return_wrist_ori_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }

  // Preparations
  Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 
  Matrix3d wrist_ori_human;
  if (left_or_right) // left arm
    wrist_ori_human = fdata.l_wrist_ori_goal;
  else // right arm
    wrist_ori_human = fdata.r_wrist_ori_goal;

  // Compute wrist orientation cost as the rotation angle between two rotation matrices
  double wrist_ori_cost = std::fabs( std::acos( std::min(((wrist_ori_human * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0, 1.0) ) );

  // Return cost function value
  return wrist_ori_cost;
}


/**
 * @brief Return the elbow position cost. 
 * 
 * Overloaded to provide easy access.
 * @param[in]   LEFT_RIGHT_BOTH   Specify left or right or both arms to compute cost for.
 */
double TrackingConstraint::return_elbow_pos_cost(bool left_or_right, bool both)
{
  // Generate new trajectories using DMP
  const DMPStartsGoalsVertex *dv = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> dx = dv->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = dx.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = dx.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = dx.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = dx.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = dx.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = dx.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = dx.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = dx.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N
  
  // get the current joint value
  const DualArmDualHandVertex *qv = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> qx = qv->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  if (left_or_right || both)
    q_cur_l = qx.block<7, 1>(0, 0);
  if (!left_or_right || both)
    q_cur_r = qx.block<7, 1>(7, 0);

  // Set new goals(expected trajectory) to fdata
  if (left_or_right || both)
    fdata.l_elbow_pos_goal = result.y_le.block(0, point_id-1, 3, 1); // Vector3d
  if (!left_or_right || both)
    fdata.r_elbow_pos_goal = result.y_re.block(0, point_id-1, 3, 1); // Vector3d is column vector

  // Compute unary costs
  double elbow_pos_cost = 0;
  if (left_or_right || both)
    elbow_pos_cost += return_elbow_pos_cost(left_fk_solver, q_cur_l, fdata.l_num_wrist_seg, fdata.l_num_elbow_seg, true, fdata); // user data is stored in fdata now
  if (!left_or_right || both)
    elbow_pos_cost += return_elbow_pos_cost(right_fk_solver, q_cur_r, fdata.r_num_wrist_seg, fdata.r_num_elbow_seg, false, fdata);

  return elbow_pos_cost;
}


/**
 * @brief Helper function for evaluating tracking condition. For internal use.
 * 
 * Calculate elbow position cost of a particular path point.
 */
double TrackingConstraint::return_elbow_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for elbow link.");
  }
  

  // Preparations
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);

  Vector3d elbow_pos_human;
  if (left_or_right) // left arm
  {
    elbow_pos_human = fdata.l_elbow_pos_goal;
  }
  else // right arm
  {
    elbow_pos_human = fdata.r_elbow_pos_goal;
  }

  // Compute cost function
  double elbow_pos_cost = (elbow_pos_cur - elbow_pos_human).norm();

  // Return cost function value
  return elbow_pos_cost;
}


/**
 * @brief Helper function. Compute wrist position offset at the current path point. For internal use.
 * 
 * This is only used for manually moving DMP starts and goals
 */
Vector3d TrackingConstraint::return_wrist_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }

  // Preparations
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);

  Vector3d wrist_pos_human;
  if (left_or_right) // left arm
    wrist_pos_human = fdata.l_wrist_pos_goal;
  else // right arm
    wrist_pos_human = fdata.r_wrist_pos_goal;

  // Compute cost function
  Vector3d wrist_pos_offset = wrist_pos_cur - wrist_pos_human; 

  // Return cost function value
  return wrist_pos_offset;
}


/**
 * @brief Helper function. Compute elbow position offset at the current path point. For internal use.
 * 
 * This is only used for manually moving DMP starts and goals.
 */
Vector3d TrackingConstraint::return_elbow_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
    q_in(i) = q_cur(i);

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    exit(-1);
  }

  // Preparations
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);

  Vector3d elbow_pos_human;
  if (left_or_right) // left arm
    elbow_pos_human = fdata.l_elbow_pos_goal;
  else // right arm
    elbow_pos_human = fdata.r_elbow_pos_goal;

  // Compute cost function
  Vector3d elbow_pos_offset = elbow_pos_cur - elbow_pos_human; 

  // Return cost function value
  return elbow_pos_offset;
}


/**
 * @brief Return the offsets of actually executed wrist position trajectory and DMP generated wrist position trajectory. 
 * Overloaded to provide easy access.
 * 
 * This is used for manually moving DMP starts and goals according to the tracking result.
 * @param[in]   left_or_right       Specify left or right arm to compute wrist position offset for.
 * @param[out]  wrist_pos_offsets   Wrist position offset. Since now TrackingConstraint is a binary edge, 
 * the output is of the size of 3 x 1.
 */
Vector3d TrackingConstraint::return_wrist_pos_offset(bool left_or_right)
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *dv = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> dx = dv->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = dx.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = dx.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = dx.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = dx.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = dx.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = dx.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = dx.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = dx.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // get the current joint value
  const DualArmDualHandVertex *qv = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> qx = qv->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  if (left_or_right)
    q_cur_l = qx.block<7, 1>(0, 0);
  else
    q_cur_r = qx.block<7, 1>(7, 0);

  // Set new goals(expected trajectory) to fdata
  if (left_or_right)
    fdata.l_wrist_pos_goal = result.y_lw.block(0, point_id-1, 3, 1); // Vector3d
  else
    fdata.r_wrist_pos_goal = result.y_rw.block(0, point_id-1, 3, 1); // Vector3d

  // Compute unary costs
  Vector3d wrist_pos_offset;
  if (left_or_right)
    wrist_pos_offset = return_wrist_pos_offset(left_fk_solver, q_cur_l, fdata.l_num_wrist_seg, fdata.l_num_elbow_seg, true, fdata);
  else
    wrist_pos_offset = return_wrist_pos_offset(right_fk_solver, q_cur_r, fdata.r_num_wrist_seg, fdata.r_num_elbow_seg, false, fdata);

  return wrist_pos_offset;
}


/**
 * @brief Return the offsets of actually executed elbow position trajectory and DMP generated elbow position trajectory.
 * Overloaded to provide easy access.
 * 
 * This is used for manually moving DMP starts and goals according to the tracking result.
 * @param[in]   left_or_right       Specify left or right arm to compute elbow position offset for.
 * @param[out]  elbow_pos_offsets   Elbow position offset, with the size of 3 x N, N is the number of datapoints.
 */
Vector3d TrackingConstraint::return_elbow_pos_offset(bool left_or_right)
{
  // Get trajectories using DMP
  const DMPStartsGoalsVertex *dv = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> dx = dv->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = dx.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = dx.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = dx.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = dx.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = dx.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = dx.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = dx.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = dx.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // get the current joint value
  const DualArmDualHandVertex *qv = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> qx = qv->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  if (left_or_right)
    q_cur_l = qx.block<7, 1>(0, 0);
  else
    q_cur_r = qx.block<7, 1>(7, 0);
  
  // Set new goals(expected trajectory) to fdata
  if (left_or_right)
    fdata.l_elbow_pos_goal = result.y_le.block(0, point_id-1, 3, 1); // Vector3d
  else
    fdata.r_elbow_pos_goal = result.y_re.block(0, point_id-1, 3, 1); // Vector3d is column vector

  // Compute unary costs
  Vector3d elbow_pos_offset;
  if (left_or_right)    
    elbow_pos_offset = return_elbow_pos_offset(left_fk_solver, q_cur_l, fdata.l_num_wrist_seg, fdata.l_num_elbow_seg, true, fdata);
  else
    elbow_pos_offset = return_elbow_pos_offset(right_fk_solver, q_cur_r, fdata.r_num_wrist_seg, fdata.r_num_elbow_seg, false, fdata);

  return elbow_pos_offset;
}


/**
 * @brief Used by g2o internal calculation.
 * 
 * Here we skip DMP generation when DMP vertex is fixed, so as to reduce number of queries during q optimization.
 * Compute error vector for each path point, in the order: l wrist pos -> l wrist ori -> l elbow pos 
 * -> r wrist pos -> r wrist ori -> r elbow pos -> l finger (1d) -> r finger (1d).
 */
void TrackingConstraint::computeError()
{
  // statistics
  count_tracking++;  
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

  // Use DMP to generate new trajectories
  DMP_trajs result;
  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();  
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  if (!(v->fixed())) // if DMP vertex not fixed, use DMP to generate trajectories
  {
    // Get current DMP starts and goals
    Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
    MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
    MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
    MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
    MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
    MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
    MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
    MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
    MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);

    result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                            lew_new_goal.transpose(), lew_new_start.transpose(),
                                                            rew_new_goal.transpose(), rew_new_start.transpose(),
                                                            rw_new_goal.transpose(), rw_new_start.transpose(),
                                                            NUM_DATAPOINTS); // results are 3 x N
  }
  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent1 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  total_traj += t_spent1.count();
  count_traj++;  

  // Iterate through all path points to compute costs
  double cost = 0;
  double total_cost = 0;

  // get the current joint value
  const DualArmDualHandVertex *qv = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x = qv->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
  q_cur_l = x.block<7, 1>(0, 0);
  q_cur_r = x.block<7, 1>(7, 0);
  q_cur_finger_l = x.block<12, 1>(14, 0);
  q_cur_finger_r = x.block<12, 1>(26, 0); 
  
  // Set new goals(expected trajectory) to user data
  // left arm part:
  if (v->fixed()) // if DMP vertex is fixed, use data passed in
  {
    fdata.l_wrist_pos_goal = fdata.DMP_lw.block(0, point_id-1, 3, 1);
    fdata.l_elbow_pos_goal = fdata.DMP_le.block(0, point_id-1, 3, 1);
  }
  else
  {
    fdata.l_wrist_pos_goal = result.y_lw.block(0, point_id-1, 3, 1); // Vector3d
    fdata.l_elbow_pos_goal = result.y_le.block(0, point_id-1, 3, 1); // Vector3d
  }
  Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 0), 
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 1),
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 2),
                  trajectory_generator_ptr->l_wrist_quat_traj(point_id-1, 3));
  Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
  fdata.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d
  // right arm part:
  if (v->fixed()) // if DMP vertex is fixed, use data passed in
  {
    fdata.r_wrist_pos_goal = fdata.DMP_rw.block(0, point_id-1, 3, 1);
    fdata.r_elbow_pos_goal = fdata.DMP_re.block(0, point_id-1, 3, 1);
  }
  else
  {
    fdata.r_wrist_pos_goal = result.y_rw.block(0, point_id-1, 3, 1); // Vector3d
    fdata.r_elbow_pos_goal = result.y_re.block(0, point_id-1, 3, 1); // Vector3d is column vector
  }
  Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 0), 
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 1),
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 2),
                  trajectory_generator_ptr->r_wrist_quat_traj(point_id-1, 3));
  Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
  fdata.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d


  // hand parts:
  fdata.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(point_id-1, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
  fdata.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(point_id-1, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF

  
  // Compute error vector (coefficients included)
  // initialize error vector
  Matrix<double, 20, 1> err_vec;
  // l arm
  err_vec.block(0, 0, 3, 1) = compute_wrist_pos_error(left_fk_solver, q_cur_l, 
                                                      fdata.l_num_wrist_seg, true, fdata);
  err_vec.block(3, 0, 3, 1) = compute_wrist_ori_error(left_fk_solver, q_cur_l, 
                                                      fdata.l_num_wrist_seg, true, fdata);
  err_vec.block(6, 0, 3, 1) = compute_elbow_pos_error(left_fk_solver, q_cur_l, 
                                                      fdata.l_num_elbow_seg, true, fdata);
  // r arm
  err_vec.block(9, 0, 3, 1) = compute_wrist_pos_error(right_fk_solver, q_cur_r, 
                                                      fdata.r_num_wrist_seg, false, fdata);
  err_vec.block(12, 0, 3, 1) = compute_wrist_ori_error(right_fk_solver, q_cur_r, 
                                                      fdata.r_num_wrist_seg, false, fdata);
  err_vec.block(15, 0, 3, 1) = compute_elbow_pos_error(right_fk_solver, q_cur_r, 
                                                      fdata.r_num_elbow_seg, false, fdata);
  // l finger
  err_vec[18] = compute_finger_cost(q_cur_finger_l, true, fdata);  
  err_vec[19] = compute_finger_cost(q_cur_finger_r, false, fdata);  
  
  // assign to _error
  _error = err_vec;  

  // apply scale before nullspace modification (must be before nullspace modification, otherwise the property would be broken)
  // double uniform_scale = 1.0;//0.1;//0.5;//1.0; 
  double wrist_scale = 1.0; //0.05; //0.1; //0.5; //1.0;
  double elbow_scale = 1.0; //0.05; //0.1; //0.5; //1.0;
  _error.block(0, 0, 6, 1) = wrist_scale * _error.block(0, 0, 6, 1);
  _error.block(6, 0, 3, 1) = elbow_scale * _error.block(6, 0, 3, 1);
  _error.block(9, 0, 6, 1) = wrist_scale * _error.block(9, 0, 6, 1);
  _error.block(15, 0, 3, 1) = elbow_scale * _error.block(15, 0, 3, 1);


  // Adjust wrist and elbow error to implement nullspace control method here
  // get the corresponding Cartesian differential movements
  Matrix<double, 6, 1> dx_lw = _error.block(0, 0, 6, 1);  // pos + ori parts
  Vector3d dx_le = _error.block(6, 0, 3, 1); // only the pos part
  Matrix<double, 6, 1> dx_rw = _error.block(9, 0, 6, 1);  // pos + ori parts
  Vector3d dx_re = _error.block(15, 0, 3, 1); // only the pos part  
  // compute jacobians for wrist and positon, and adjust _error to accomplish nullspace control
  std::vector<double> q_in;
  for (unsigned int j = 0; j < JOINT_DOF; j++)
    q_in.push_back(x[j]);
  this->J_lw = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(q_in, true, LEFT_WRIST_LINK);
  this->J_le = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(q_in, true, LEFT_ELBOW_LINK);
  this->J_rw = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(q_in, false, RIGHT_WRIST_LINK);
  this->J_re = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(q_in, false, RIGHT_ELBOW_LINK);
  // prepare necessary entities
  JacobiSVD<MatrixXd> svd_lw(J_lw, ComputeThinU | ComputeThinV); // svd.singularValues()/.matrixU()/.matrixV()
  JacobiSVD<MatrixXd> svd_rw(J_rw, ComputeThinU | ComputeThinV); // svd.singularValues()/.matrixU()/.matrixV()
  Matrix<double, 7, 1> dq_lw = svd_lw.solve(dx_lw);
  Matrix<double, 7, 1> dq_rw = svd_rw.solve(dx_rw);
  // adjust the error vector according to nullspace control method
  Vector3d dx_le_new = dx_le - J_le.topRows(3) * dq_lw; // take only the first three rows
  Vector3d dx_re_new = dx_re - J_re.topRows(3) * dq_rw; // take only the first three rows
  // assign new error to _error
  _error.block(6, 0, 3, 1) = dx_le_new;
  _error.block(15, 0, 3, 1) = dx_re_new;

  // lower the importance of elbows 
  double elbow_ratio = 0.5;//0.1;//0.01; //0.1; //0.2;//1.0;//0.5;//0.1;//0.01;//1.0; //0.01;
  _error.block(6, 0, 3, 1) = elbow_ratio * _error.block(6, 0, 3, 1);
  _error.block(15, 0, 3, 1) = elbow_ratio * _error.block(15, 0, 3, 1);

  // change direction (control and optimization have different directions)
  _error = -_error;

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_tracking += t_spent.count();
}


#endif
