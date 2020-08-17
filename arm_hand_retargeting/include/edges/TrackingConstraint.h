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
#include <g2o/core/base_multi_edge.h>

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
 */
/* addVertex rule: connect DMP starts and goals vertex first, then datapoints */
class TrackingConstraint : public BaseMultiEdge<1, my_constraint_struct> // <D, E>, dimension and measurement datatype
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor of TrackingConstraint. \n
     * Initialize with left and right KDL FK solver, collision checker and DMP trajectory generator. \n
     * Use list initialization for reference member.
     */
    TrackingConstraint(boost::shared_ptr<DMPTrajectoryGenerator> &_trajectory_generator_ptr, 
                      KDL::ChainFkSolverPos_recursive &_left_fk_solver, 
                      KDL::ChainFkSolverPos_recursive &_right_fk_solver, 
                      boost::shared_ptr<DualArmDualHandCollision> &_dual_arm_dual_hand_collision_ptr,
                      unsigned int num_datapoints) : trajectory_generator_ptr(_trajectory_generator_ptr), 
                                                     left_fk_solver(_left_fk_solver), 
                                                     right_fk_solver(_right_fk_solver), 
                                                     dual_arm_dual_hand_collision_ptr(_dual_arm_dual_hand_collision_ptr)
    {
      // record
      this->num_datapoints = num_datapoints;  
      this->num_vertices = num_datapoints + 1;
      // set the number of vertices this edge connects to
      resize(num_datapoints+1); // q path points plus one DMP vertex
    }


    /// (DMP) Trajectory generator
    boost::shared_ptr<DMPTrajectoryGenerator> &trajectory_generator_ptr;


    // KDL solvers
    KDL::ChainFkSolverPos_recursive &left_fk_solver;      ///< KDL FK solver for left arm.
    KDL::ChainFkSolverPos_recursive &right_fk_solver;     ///< KDL FK solver for right arm.


    /// Collision Checker, which constains RobotState that can be used to compute jacobians, global link transforms, etc.
    boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr;
    

    // functions to compute costs
    double compute_arm_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    double linear_map(double x_, double min_, double max_, double min_hat, double max_hat);
    double compute_finger_cost(Matrix<double, 12, 1> q_finger_robot, bool left_or_right, my_constraint_struct &fdata);
    void computeError();


    // functions for recording cost history
    std::vector<double> return_finger_cost_history(unsigned int LEFT_RIGHT_BOTH);
    std::vector<double> return_wrist_pos_cost_history(unsigned int LEFT_RIGHT_BOTH);
    std::vector<double> return_wrist_ori_cost_history(unsigned int LEFT_RIGHT_BOTH);
    std::vector<double> return_elbow_pos_cost_history(unsigned int LEFT_RIGHT_BOTH);


    // helper function for calculating tracking costs for a particular path point
    double return_wrist_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    double return_wrist_ori_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    double return_elbow_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);


    // for manual manipulation of DMP starts and goals during in-turn optimization
    Vector3d return_wrist_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    Vector3d return_elbow_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    MatrixXd return_wrist_pos_offsets(unsigned int LEFT_OR_RIGHT);
    MatrixXd return_elbow_pos_offsets(unsigned int LEFT_OR_RIGHT);


    // return currently executed Cartesian trajectories for debug
    std::vector<std::vector<double> > return_wrist_pos_traj(bool left_or_right);
    std::vector<std::vector<double> > return_elbow_pos_traj(bool left_or_right);
    Vector3d return_wrist_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
    Vector3d return_elbow_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata);
  

    // return Jacobians results for debug
    MatrixXd output_dmp_jacobian();
    MatrixXd output_q_jacobian();
    Matrix<double, 1, DMPPOINTS_DOF> jacobians_for_dmp;
    Matrix<double, NUM_DATAPOINTS, JOINT_DOF> jacobians_for_q;


    // for debugging every parts' jacobians
    double cur_wrist_pos_cost;
    double cur_wrist_ori_cost;
    double cur_elbow_pos_cost;
    double cur_finger_pos_cost; // for a single path point; used in compute_arm_cost() or compute_finger_cost()
    Matrix<double, 14, NUM_DATAPOINTS> wrist_pos_jacobian_for_q_arm;
    Matrix<double, 14, NUM_DATAPOINTS> wrist_ori_jacobian_for_q_arm;
    Matrix<double, 14, NUM_DATAPOINTS> elbow_pos_jacobian_for_q_arm;
    Matrix<double, 24, NUM_DATAPOINTS> finger_pos_jacobian_for_q_finger;
    Matrix<double, 7, 1> cur_wrist_pos_jacobian_for_q_arm;
    Matrix<double, 7, 1> cur_wrist_ori_jacobian_for_q_arm;
    Matrix<double, 7, 1> cur_elbow_pos_jacobian_for_q_arm;

    Matrix<double, DMPPOINTS_DOF, 1> wrist_pos_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> wrist_ori_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> elbow_pos_jacobian_for_dmp;    
    Matrix<double, DMPPOINTS_DOF, 1> finger_pos_jacobian_for_dmp;

    double cur_wrist_pos_cost_total;
    double cur_wrist_ori_cost_total;
    double cur_elbow_pos_cost_total;
    double cur_finger_pos_cost_total; // for all path points; used in computeError()

    // Map human hands' joint values to robot hands, for setting initial state
    Matrix<double, 12, 1> map_finger_joint_values(Matrix<double, 14, 1> q_finger_human, bool left_or_right, my_constraint_struct &fdata);


    /// Re-implement numeric differentiation
    virtual void linearizeOplus();

    /// Read from disk, leave blank
    virtual bool read( std::istream& in ) {return true;}
    
    /// Write to disk, leave blank
    virtual bool write( std::ostream& out ) const {return true;}

    // flags
    unsigned int LEFT_FLAG = 0;     ///< Flag for querying left arm or left finger related quantities
    unsigned int RIGHT_FLAG = 1;    ///< Flag for querying right arm or right finger related quantities
    unsigned int BOTH_FLAG = 2;     ///< Flag for querying both arms or both fingers related quantities

  private:
    // numbers
    unsigned int num_datapoints; ///< internal variable
    unsigned int num_vertices;   ///< internal variable

};


/**
 * Helper function. Return the wrist position corresponding to current joint configuration 
 */
Vector3d TrackingConstraint::return_wrist_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
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
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
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
 
  // return results
  return elbow_pos_cur;
}


/**
 * Return the current wrist position trajectory via FK for debug
 */
std::vector<std::vector<double> > TrackingConstraint::return_wrist_pos_traj(bool left_or_right)
{
  // prep
  std::vector<std::vector<double>> wrist_pos_traj;
  std::vector<double> cur_wrist_pos(3);

  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    // _vertices is a VertexContainer type, a std::vector<Vertex*>
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex
    //std::cout << "debug: x size is: " << x.rows() << " x " << x.cols() << std::endl;
    //std::cout << "debug: x = \n" << x.transpose() << std::endl;

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    
    // Compute unary costs
    Vector3d cur_wrist_pos_vec;
    if(left_or_right) 
    {
      cur_wrist_pos_vec = return_wrist_pos(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement);
    }
    else
    {
      cur_wrist_pos_vec = return_wrist_pos(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);
    }

    // Copy to std::vector
    cur_wrist_pos[0] = cur_wrist_pos_vec[0]; 
    cur_wrist_pos[1] = cur_wrist_pos_vec[1]; 
    cur_wrist_pos[2] = cur_wrist_pos_vec[2]; 
    wrist_pos_traj.push_back(cur_wrist_pos);

  }

  return wrist_pos_traj;
}


/**
 * Return the current elbow position trajectory via FK for debug
 */
std::vector<std::vector<double> > TrackingConstraint::return_elbow_pos_traj(bool left_or_right)
{
  // prep
  std::vector<std::vector<double>> elbow_pos_traj;
  std::vector<double> cur_elbow_pos(3);

  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    // _vertices is a VertexContainer type, a std::vector<Vertex*>
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex
    //std::cout << "debug: x size is: " << x.rows() << " x " << x.cols() << std::endl;
    //std::cout << "debug: x = \n" << x.transpose() << std::endl;

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    
    // Compute unary costs
    Vector3d cur_elbow_pos_vec;
    if(left_or_right) 
    {
      cur_elbow_pos_vec = return_elbow_pos(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement);
    }
    else
    {
      cur_elbow_pos_vec = return_elbow_pos(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);
    }

    // Copy to std::vector
    cur_elbow_pos[0] = cur_elbow_pos_vec[0]; 
    cur_elbow_pos[1] = cur_elbow_pos_vec[1]; 
    cur_elbow_pos[2] = cur_elbow_pos_vec[2]; 
    elbow_pos_traj.push_back(cur_elbow_pos);

  }

  return elbow_pos_traj;

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
MatrixXd TrackingConstraint::output_q_jacobian()
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
  double dmp_eps = 0.001; // better be small enough so that gradients on different dimensions won't diverge too much
  double q_arm_eps = 0.5 * M_PI / 180; //2.0 * M_PI / 180; //0.5; // may need tests // in radius!!!
  double q_finger_eps = 1.0 * M_PI / 180; //2.0 * M_PI / 180; // in radius
  double e_plus, e_minus;

  // for calculating and storing jacobians for wrist_pos, wrist_ori and elbow_pos
  double e_wrist_pos_plus, e_wrist_pos_minus;  
  double e_wrist_ori_plus, e_wrist_ori_minus;
  double e_elbow_pos_plus, e_elbow_pos_minus;
  double e_finger_pos_plus, e_finger_pos_minus;

  // 1 - For DMP starts and goals
  DMPStartsGoalsVertex *v = dynamic_cast<DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  if (v->fixed()) // if dmp vertex is fixed, use the data passed in, and assign zeros updates to dmp vertex
  {
      // assign zeros
      _jacobianOplus[0] = Matrix<double, 1, DMPPOINTS_DOF>::Zero();
      // record
      this->jacobians_for_dmp = _jacobianOplus[0];
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
      e_plus = _error(0, 0);
      e_wrist_pos_plus = this->cur_wrist_pos_cost_total; // record after calling computeError(); different from that for q
      e_wrist_ori_plus = this->cur_wrist_ori_cost_total;
      e_elbow_pos_plus = this->cur_elbow_pos_cost_total;
      e_finger_pos_plus = this->cur_finger_pos_cost_total;

      v->setEstimate(x-delta_x);
      this->computeError();
      e_minus = _error(0, 0);
      e_wrist_pos_minus = this->cur_wrist_pos_cost_total; // record after calling computeError(); different from that for q 
      e_wrist_ori_minus = this->cur_wrist_ori_cost_total;
      e_elbow_pos_minus = this->cur_elbow_pos_cost_total; 
      e_finger_pos_minus = this->cur_finger_pos_cost_total;   

      // reset delta
      delta_x[n] = 0.0;

      // set and store jacobians 
      _jacobianOplus[0](0, n) = (e_plus - e_minus) / (2*dmp_eps); // for DMP starts and goals
      wrist_pos_jacobian_for_dmp[n] = K_WRIST_POS * (e_wrist_pos_plus - e_wrist_pos_minus) / ( 2 * dmp_eps);
      wrist_ori_jacobian_for_dmp[n] = K_WRIST_ORI * (e_wrist_ori_plus - e_wrist_ori_minus) / ( 2 * dmp_eps);
      elbow_pos_jacobian_for_dmp[n] = K_ELBOW_POS * (e_elbow_pos_plus - e_elbow_pos_minus) / ( 2 * dmp_eps); // remember to change the eps !!
      finger_pos_jacobian_for_dmp[n] = (e_finger_pos_plus - e_finger_pos_minus) / (2 * dmp_eps); // K_FINGER is included

      // store jacobians
      this->jacobians_for_dmp[n] = _jacobianOplus[0](0, n);
    }

    // reset vertex value
    v->setEstimate(x);

    // 2 - Assign zero jacobians to q vertices
    for (unsigned int n = 0; n < num_datapoints; n++)
      _jacobianOplus[n+1] = Matrix<double, 1, JOINT_DOF>::Zero();

    // skip the calculation of q jacobians when q vertices are fixed
    return; 
  }


  // 2 - For q vertices
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
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint values
    const DualArmDualHandVertex *v_tmp = dynamic_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> q = v_tmp->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;    
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = q.block<7, 1>(0, 0);
    q_cur_r = q.block<7, 1>(7, 0);
    q_cur_finger_l = q.block<12, 1>(14, 0);
    q_cur_finger_r = q.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    if (v->fixed()) // if dmp vertex fixed, use data passed in
    {
      _measurement.l_wrist_pos_goal = _measurement.DMP_lw.block(0, n, 3, 1);
      _measurement.l_elbow_pos_goal = _measurement.DMP_le.block(0, n, 3, 1);
    }
    else
    {
      _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
      _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    }
    Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 3));
    Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
    _measurement.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d


    // right arm part:
    if (v->fixed()) // if dmp vertex fixed, use data passed in
    {
      _measurement.r_wrist_pos_goal = _measurement.DMP_rw.block(0, n, 3, 1);
      _measurement.r_elbow_pos_goal = _measurement.DMP_re.block(0, n, 3, 1);
    }
    else
    {
      _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d
      _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d
    }
    Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 3));
    Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
    _measurement.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d

    // hand parts:
    _measurement.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
    _measurement.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF

    
    // (1) - jacobians for arms - way 1: numeric differentiation
    std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();      
    for (unsigned int d = 0; d < 7; d++)
    {
      // set delta
      delta_q_arm[d] = q_arm_eps;

      // left arm
      e_plus = compute_arm_cost(left_fk_solver, q_cur_l+delta_q_arm, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement); // user data is stored in _measurement now
      e_wrist_pos_plus = this->cur_wrist_pos_cost;
      e_wrist_ori_plus = this->cur_wrist_ori_cost;
      e_elbow_pos_plus = this->cur_elbow_pos_cost;

      e_minus = compute_arm_cost(left_fk_solver, q_cur_l-delta_q_arm, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement); // user data is stored in _measurement now
      e_wrist_pos_minus = this->cur_wrist_pos_cost;
      e_wrist_ori_minus = this->cur_wrist_ori_cost;
      e_elbow_pos_minus = this->cur_elbow_pos_cost;

      _jacobianOplus[n+1](0, d) = (e_plus - e_minus) / (2 * q_arm_eps);
      wrist_pos_jacobian_for_q_arm(d, n) = K_WRIST_POS * (e_wrist_pos_plus - e_wrist_pos_minus) / ( 2 * q_arm_eps);
      wrist_ori_jacobian_for_q_arm(d, n) = K_WRIST_ORI * (e_wrist_ori_plus - e_wrist_ori_minus) / ( 2 * q_arm_eps);
      elbow_pos_jacobian_for_q_arm(d, n) = K_ELBOW_POS * (e_elbow_pos_plus - e_elbow_pos_minus) / ( 2 * q_arm_eps);


      // right arm
      e_plus = compute_arm_cost(right_fk_solver, q_cur_r+delta_q_arm, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);
      e_wrist_pos_plus = this->cur_wrist_pos_cost;
      e_wrist_ori_plus = this->cur_wrist_ori_cost;
      e_elbow_pos_plus = this->cur_elbow_pos_cost;
      
      e_minus = compute_arm_cost(right_fk_solver, q_cur_r-delta_q_arm, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);
      e_wrist_pos_minus = this->cur_wrist_pos_cost;
      e_wrist_ori_minus = this->cur_wrist_ori_cost;
      e_elbow_pos_minus = this->cur_elbow_pos_cost;      

      _jacobianOplus[n+1](0, d+7) = (e_plus - e_minus) / (2 * q_arm_eps);      
      wrist_pos_jacobian_for_q_arm(d+7, n) = K_WRIST_POS * (e_wrist_pos_plus - e_wrist_pos_minus) / ( 2 * q_arm_eps);
      wrist_ori_jacobian_for_q_arm(d+7, n) = K_WRIST_ORI * (e_wrist_ori_plus - e_wrist_ori_minus) / ( 2 * q_arm_eps);
      elbow_pos_jacobian_for_q_arm(d+7, n) = K_ELBOW_POS * (e_elbow_pos_plus - e_elbow_pos_minus) / ( 2 * q_arm_eps);

      // reset delta
      delta_q_arm[d] = 0.0;
    }
    std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();  
    std::chrono::duration<double> t_spent1 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
    t_arm_numeric += t_spent1.count();
    

    // (2) - jacobians for fingers
    // left_tmp = compute_finger_cost(q_cur_finger_l, true, _measurement);  
    // right_tmp = compute_finger_cost(q_cur_finger_r, false, _measurement); 
    for (unsigned int d = 0; d < 12; d++)
    {
      // set delta
      delta_q_finger[d] = q_finger_eps;

      // left hand
      e_plus = compute_finger_cost(q_cur_finger_l+delta_q_finger, true, _measurement);
      e_minus = compute_finger_cost(q_cur_finger_l-delta_q_finger, true, _measurement);
      _jacobianOplus[n+1](0, d+14) = K_FINGER * (e_plus - e_minus) / (2*q_finger_eps); // K_FINGER not included
      finger_pos_jacobian_for_q_finger(d, n) = K_FINGER * (e_plus - e_minus) / (2*q_finger_eps);

      // right hand
      e_plus = compute_finger_cost(q_cur_finger_r+delta_q_finger, false, _measurement);  
      e_minus = compute_finger_cost(q_cur_finger_r-delta_q_finger, false, _measurement);  
      _jacobianOplus[n+1](0, d+26) = K_FINGER * (e_plus - e_minus) / (2*q_finger_eps);
      finger_pos_jacobian_for_q_finger(d+12, n) = K_FINGER * (e_plus - e_minus) / (2*q_finger_eps);

      // reset delta
      delta_q_finger[d] = 0.0;
    }
  }

  // 3 - Save jacobians for q vertices
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    for (unsigned int d = 0; d < JOINT_DOF; d++)
      jacobians_for_q(n, d) = _jacobianOplus[n+1](0, d); // starts from 1
  }

}


/**
 * Helper function. Calculate wrist position cost of a particular path point.
 */
double TrackingConstraint::return_wrist_pos_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }

  // Preparations
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);

  Vector3d wrist_pos_human;
  if (left_or_right) // left arm
  {
    wrist_pos_human = fdata.l_wrist_pos_goal;
  }
  else // right arm
  {
    wrist_pos_human = fdata.r_wrist_pos_goal;
  }

  // Compute cost function
  double wrist_pos_cost = (wrist_pos_cur - wrist_pos_human).norm(); // _human is actually the newly generated trajectory

  // Return cost function value
  return wrist_pos_cost;
}


/**
 * Helper function. Calculate wrist orientation cost of a particular path point.
 */
double TrackingConstraint::return_wrist_ori_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }

  // Preparations
  Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 

  Matrix3d wrist_ori_human;
  if (left_or_right) // left arm
  {
    wrist_ori_human = fdata.l_wrist_ori_goal;
  }
  else // right arm
  {
    wrist_ori_human = fdata.r_wrist_ori_goal;
  }

  // Compute cost function
  double wrist_ori_cost = std::fabs( std::acos( std::min(((wrist_ori_human * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0, 1.0) ) );

  // Return cost function value
  return wrist_ori_cost;
}


/**
 * Helper function. Calculate elbow position cost of a particular path point.
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
 * Helper function. Compute arm cost for use in computeError() and linearizeOplus().
 */
double TrackingConstraint::compute_arm_cost(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Way 1: KDL FK solver
  // std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();  
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out, wrist_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    exit(-1);
  }
  else{
      //ROS_INFO_STREAM("FK solver succeeded for elbow link.");
  }
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }
  // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  
  // std::chrono::duration<double> t_spent_10 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  // std::cout << "compute_arm_cost: KDL solver spent " << t_spent_10.count() << " s." << std::endl;


  // Way 2: RobotState
  /*
  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();  
  // select links
  std::string elbow_link_name = (left_or_right ? this->LEFT_ELBOW_LINK : this->RIGHT_ELBOW_LINK);
  std::string wrist_link_name = (left_or_right ? this->LEFT_WRIST_LINK : this->RIGHT_WRIST_LINK);
  // get joint values
  std::vector<double> q_current(JOINT_DOF);
  unsigned int d = (left_or_right ? 0 : q_cur.size()); // offset to cope with left and right arm joints
  for (unsigned int s = 0; s < q_cur.size(); s++)
    q_current[s+d] = q_cur[s];
  std::cout << "q_current = ";
  for (unsigned int s = 0; s < JOINT_DOF; s++)
    std::cout << q_current[s] << " ";
  std::cout << std::endl;  
  // compute pos and ori for elbow and wrist
  dual_arm_dual_hand_collision_ptr->set_joint_values_yumi(q_current);
  Eigen::Vector3d elbow_pos_tmp = this->dual_arm_dual_hand_collision_ptr->get_link_pos(q_current, elbow_link_name);
  Eigen::Vector3d wrist_pos_tmp = this->dual_arm_dual_hand_collision_ptr->get_link_pos(q_current, wrist_link_name);
  Eigen::Matrix3d elbow_ori_tmp = this->dual_arm_dual_hand_collision_ptr->get_link_ori(q_current, elbow_link_name);
  Eigen::Matrix3d wrist_ori_tmp = this->dual_arm_dual_hand_collision_ptr->get_link_ori(q_current, wrist_link_name);

  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();  
  std::chrono::duration<double> t_spent_1100 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  std::cout << "compute_arm_cost: RobotState spent " << t_spent_1100.count() << " s." << std::endl;
  */

  // Get the results
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);
  Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 
  //Vector3d shoulder_pos_cur = Map<Vector3d>(shoulder_cart_out.p.data, 3, 1);

  // Results comparison:
  /*
  std::cout << "Compare Results: " << std::endl;
  std::cout << "1 - KDL: wrist_pos = " << wrist_pos_cur.transpose() << ", elbow_pos = " << elbow_pos_cur.transpose()
            << ", wrist_ori = " << wrist_ori_cur << std::endl;
  std::cout << "2 - RobotState: wrist_pos = " << wrist_pos_tmp.transpose() << ", elbow_pos = " << elbow_pos_tmp.transpose()
            << ", wrist_ori = " << wrist_ori_tmp << std::endl;
  */

  // Specify human data
  Vector3d elbow_pos_human, wrist_pos_human; //,shoulder_pos_human
  Matrix3d wrist_ori_human;
  if (left_or_right) // left arm
  {
    //shoulder_pos_human = fdata.l_shoulder_pos_goal;
    elbow_pos_human = fdata.l_elbow_pos_goal;
    wrist_pos_human = fdata.l_wrist_pos_goal;
    wrist_ori_human = fdata.l_wrist_ori_goal;
  }
  else // right arm
  {
    //shoulder_pos_human = fdata.r_shoulder_pos_goal;
    elbow_pos_human = fdata.r_elbow_pos_goal;
    wrist_pos_human = fdata.r_wrist_pos_goal;
    wrist_ori_human = fdata.r_wrist_ori_goal;
  }
  

  // Compute cost function
  double wrist_pos_cost = (wrist_pos_cur - wrist_pos_human).norm(); // _human is actually the newly generated trajectory
  double elbow_pos_cost = (elbow_pos_cur - elbow_pos_human).norm();
  double wrist_ori_cost = std::fabs( std::acos( std::min(((wrist_ori_human * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0, 1.0) ) );
  double cost = K_WRIST_ORI * wrist_ori_cost + K_WRIST_POS * wrist_pos_cost + K_ELBOW_POS * elbow_pos_cost;

  // store for debugging jacobians
  this->cur_wrist_pos_cost = wrist_pos_cost;
  this->cur_wrist_ori_cost = wrist_ori_cost;
  this->cur_elbow_pos_cost = elbow_pos_cost;

  // Return cost function value
  return cost;
}


/**
 * Helper function. Do linear mapping on finger joints for use in computing finger position cost.
 */
double TrackingConstraint::linear_map(double x_, double min_, double max_, double min_hat, double max_hat)
{
  return (x_ - min_) / (max_ - min_) * (max_hat - min_hat) + min_hat;
}


/**
 * Helper function. Compute finger cost for use in computeError() and linearizeOplus(). \n
 * K_FINGER is not applied here.
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
  double finger_cost = (q_finger_robot_goal - q_finger_robot).norm();

  // store for debug
  this->cur_finger_pos_cost = finger_cost;

  return finger_cost;
}


/**
 * @brief Directly map human finger data to robot hands. 
 * 
 * Result is used as an initial setup for finger part.
 */
Matrix<double, 12, 1> TrackingConstraint::map_finger_joint_values(Matrix<double, 14, 1> q_finger_human, bool left_or_right, my_constraint_struct &fdata)
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
 * @brief Return finger cost history.
 * 
 * Actually, return the finger cost for each path point under the current state. (not history)
 * @param[in]   LEFT_RIGHT_BOTH   Specify left or right or both hands to compute cost for.
 */
std::vector<double> TrackingConstraint::return_finger_cost_history(unsigned int LEFT_RIGHT_BOTH)
{
  // Check if valid
  if (LEFT_RIGHT_BOTH != LEFT_FLAG && LEFT_RIGHT_BOTH != RIGHT_FLAG && LEFT_RIGHT_BOTH != BOTH_FLAG)
  {
    std::cerr << "Flag is not a choice, check again!!!" << std::endl;
    exit(-1);
  }

  // iterate to compute costs
  std::vector<double> finger_cost_history;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    // _vertices is a VertexContainer type, a std::vector<Vertex*>
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      q_cur_finger_l = x.block<12, 1>(14, 0);
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Set new goals(expected trajectory) to _measurement
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      _measurement.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      _measurement.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF

    // Compute unary costs
    double finger_cost = 0;
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)    
      finger_cost += compute_finger_cost(q_cur_finger_l, true, _measurement);  
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      finger_cost += compute_finger_cost(q_cur_finger_r, false, _measurement);  
  
    finger_cost_history.push_back(finger_cost);
  }

  return finger_cost_history;
}


/**
 * @brief Return wrist position cost history.
 * 
 * Actually, return the wrist position cost for each path point under the current state. (not history)
 * @param[in]   LEFT_RIGHT_BOTH   Specify left or right or both arms to compute cost for.
 */
std::vector<double> TrackingConstraint::return_wrist_pos_cost_history(unsigned int LEFT_RIGHT_BOTH)
{
  // Check if valid
  if (LEFT_RIGHT_BOTH != LEFT_FLAG && LEFT_RIGHT_BOTH != RIGHT_FLAG && LEFT_RIGHT_BOTH != BOTH_FLAG)
  {
    std::cerr << "Flag is not a choice, check again!!!" << std::endl;
    exit(-1);
  }

  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> wrist_pos_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // Get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      q_cur_l = x.block<7, 1>(0, 0);
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      q_cur_r = x.block<7, 1>(7, 0);
    
    // Set new goals(expected trajectory) to _measurement
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d

    // Compute unary costs
    double wrist_pos_cost = 0;
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      wrist_pos_cost += return_wrist_pos_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement); // user data is stored in _measurement now
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      wrist_pos_cost += return_wrist_pos_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);

    wrist_pos_costs.push_back(wrist_pos_cost);
  }

  return wrist_pos_costs;
}


/**
 * @brief Return wrist orientation cost history.
 * 
 * Actually, return the wrist orientation cost for each path point under the current state. (not history)
 * @param[in]   LEFT_RIGHT_BOTH   Specify left or right or both arms to compute cost for.
 */
std::vector<double> TrackingConstraint::return_wrist_ori_cost_history(unsigned LEFT_RIGHT_BOTH)
{
  // Check if valid
  if (LEFT_RIGHT_BOTH != LEFT_FLAG && LEFT_RIGHT_BOTH != RIGHT_FLAG && LEFT_RIGHT_BOTH != BOTH_FLAG)
  {
    std::cerr << "Flag is not a choice, check again!!!" << std::endl;
    exit(-1);
  }

  // iterate to compute costs
  std::vector<double> wrist_ori_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    
    // Set new goals(expected trajectory) to _measurement
    Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 3));
    Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
    _measurement.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d
    Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 3));
    Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
    _measurement.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d

    // Compute unary costs
    double wrist_ori_cost = 0;
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)    
      wrist_ori_cost += return_wrist_ori_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement); // user data is stored in _measurement now
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      wrist_ori_cost += return_wrist_ori_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);

    wrist_ori_costs.push_back(wrist_ori_cost);
  }

  return wrist_ori_costs;
}


/**
 * @brief Return elbow position cost history.
 * 
 * Actually, return the elbow position cost for each path point under the current state. (not history)
 * @param[in]   LEFT_RIGHT_BOTH   Specify left or right or both arms to compute cost for.
 */
std::vector<double> TrackingConstraint::return_elbow_pos_cost_history(unsigned int LEFT_RIGHT_BOTH)
{
  // Check if valid
  if (LEFT_RIGHT_BOTH != LEFT_FLAG && LEFT_RIGHT_BOTH != RIGHT_FLAG && LEFT_RIGHT_BOTH != BOTH_FLAG)
  {
    std::cerr << "Flag is not a choice, check again!!!" << std::endl;
    exit(-1);
  }

  // Generate new trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> elbow_pos_costs;
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      q_cur_l = x.block<7, 1>(0, 0);
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      q_cur_r = x.block<7, 1>(7, 0);

    // Set new goals(expected trajectory) to _measurement
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector

    // Compute unary costs
    double elbow_pos_cost = 0;
    if (LEFT_RIGHT_BOTH == LEFT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      elbow_pos_cost += return_elbow_pos_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement); // user data is stored in _measurement now
    if (LEFT_RIGHT_BOTH == RIGHT_FLAG || LEFT_RIGHT_BOTH == BOTH_FLAG)
      elbow_pos_cost += return_elbow_pos_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);

    elbow_pos_costs.push_back(elbow_pos_cost);
  }

  return elbow_pos_costs;
}


/**
 * @brief Helper function. Compute wrist position offset at a particular path point.
 */
Vector3d TrackingConstraint::return_wrist_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }

  // Preparations
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);

  Vector3d wrist_pos_human;
  if (left_or_right) // left arm
  {
    wrist_pos_human = fdata.l_wrist_pos_goal;
  }
  else // right arm
  {
    wrist_pos_human = fdata.r_wrist_pos_goal;
  }

  // Compute cost function
  Vector3d wrist_pos_offset = wrist_pos_cur - wrist_pos_human; 

  // Return cost function value
  return wrist_pos_offset;
}


/**
 * @brief Helper function. Compute elbow position offset at a particular path point.
 */
Vector3d TrackingConstraint::return_elbow_pos_offset(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, bool left_or_right, my_constraint_struct &fdata)
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
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
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
  Vector3d elbow_pos_offset = elbow_pos_cur - elbow_pos_human; 

  // Return cost function value
  return elbow_pos_offset;
}


/**
 * @brief Return the offsets of actually executed wrist position trajectory and DMP generated wrist position trajectory.
 * 
 * This is used for manually moving DMP starts and goals according to the tracking result.
 * @param[in]   LEFT_OR_RIGHT       Specify left or right arm to compute wrist position offset for.
 * @param[out]  wrist_pos_offsets   Wrist position offset, with the size of 3 x N, N is the number of datapoints.
 */
MatrixXd TrackingConstraint::return_wrist_pos_offsets(unsigned int LEFT_OR_RIGHT)
{
  // Check if valid
  if (LEFT_OR_RIGHT != LEFT_FLAG && LEFT_OR_RIGHT != RIGHT_FLAG)
  {
    std::cerr << "Flag is meaningless, check again!!!" << std::endl;
    exit(-1);
  }

  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> wrist_pos_costs;
  MatrixXd wrist_pos_offsets(3, num_datapoints);
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    if (LEFT_OR_RIGHT == LEFT_FLAG)
      q_cur_l = x.block<7, 1>(0, 0);
    if (LEFT_OR_RIGHT == RIGHT_FLAG)
      q_cur_r = x.block<7, 1>(7, 0);

    // Set new goals(expected trajectory) to _measurement
    if (LEFT_OR_RIGHT == LEFT_FLAG)
      _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
    if (LEFT_OR_RIGHT == RIGHT_FLAG)
      _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d

    // Compute unary costs
    Vector3d wrist_pos_offset;
    if (LEFT_OR_RIGHT == LEFT_FLAG)
      wrist_pos_offset = return_wrist_pos_offset(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement);
    if (LEFT_OR_RIGHT == RIGHT_FLAG)
      wrist_pos_offset = return_wrist_pos_offset(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);
    

    wrist_pos_offsets.block(0, n, 3, 1) = wrist_pos_offset;
  }

  return wrist_pos_offsets;
}


/**
 * @brief Return the offsets of actually executed elbow position trajectory and DMP generated elbow position trajectory.
 * 
 * This is used for manually moving DMP starts and goals according to the tracking result.
 * @param[in]   LEFT_OR_RIGHT       Specify left or right arm to compute elbow position offset for.
 * @param[out]  elbow_pos_offsets   Elbow position offset, with the size of 3 x N, N is the number of datapoints.
 */
MatrixXd TrackingConstraint::return_elbow_pos_offsets(unsigned int LEFT_OR_RIGHT)
{
  // Check if valid
  if (LEFT_OR_RIGHT != LEFT_FLAG && LEFT_OR_RIGHT != RIGHT_FLAG)
  {
    std::cerr << "Flag is meaningless, check again!!!" << std::endl;
    exit(-1);
  }

  // Get trajectories using DMP
  const DMPStartsGoalsVertex *v = static_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
  MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
  MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
  MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
  MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
  MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
  MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
  MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
  DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                      lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                      rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                      rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                      NUM_DATAPOINTS); // results are 3 x N

  // iterate to compute costs
  std::vector<double> elbow_pos_costs;
  MatrixXd elbow_pos_offsets(3, num_datapoints);
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    if (LEFT_OR_RIGHT == LEFT_FLAG)
      q_cur_l = x.block<7, 1>(0, 0);
    if (LEFT_OR_RIGHT == RIGHT_FLAG)
      q_cur_r = x.block<7, 1>(7, 0);

    
    // Set new goals(expected trajectory) to _measurement
    if (LEFT_OR_RIGHT == LEFT_FLAG)
      _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    if (LEFT_OR_RIGHT == RIGHT_FLAG)
      _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector


    // Compute unary costs
    // only the right elbow
    Vector3d elbow_pos_offset;
    if (LEFT_OR_RIGHT == LEFT_FLAG)    
      elbow_pos_offset = return_elbow_pos_offset(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement);
    if (LEFT_OR_RIGHT == RIGHT_FLAG)
      elbow_pos_offset = return_elbow_pos_offset(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);

    elbow_pos_offsets.block(0, n, 3, 1) = elbow_pos_offset;
  }

  return elbow_pos_offsets;
}


/**
 * @brief Used by g2o internal calculation.
 * 
 * Here we skip DMP generation when DMP vertex is fixed, so as to reduce number of queries during q optimization.
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

    /*
    std::cout << "debug: \nx = " << x.transpose() << std::endl
              << "lrw_new_goal = " << lrw_new_goal.transpose() << ", lrw_new_start = " << lrw_new_start.transpose() << std::endl
              << "lew_new_goal = " << lew_new_goal.transpose() << ", lew_new_start = " << lew_new_start.transpose() << std::endl
              << "rew_new_goal = " << rew_new_goal.transpose() << ", rew_new_start = " << rew_new_start.transpose() << std::endl
              << "rw_new_goal = " << rw_new_goal.transpose() << ", rw_new_start = " << rw_new_start.transpose() << std::endl;
    */

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

  /*
  std::cout << "lrw_new_goal = " << result.y_lrw.block(0, num_datapoints-1, 3, 1).transpose() << ", lrw_new_start = " << result.y_lrw.block(0, 0, 3, 1).transpose() << std::endl
            << "lew_new_goal = " << result.y_lew.block(0, num_datapoints-1, 3, 1).transpose() << ", lew_new_start = " << result.y_lew.block(0, 0, 3, 1).transpose() << std::endl
            << "rew_new_goal = " << result.y_rew.block(0, num_datapoints-1, 3, 1).transpose() << ", rew_new_start = " << result.y_rew.block(0, 0, 3, 1).transpose() << std::endl
            << "rw_new_goal = " << result.y_rw.block(0, num_datapoints-1, 3, 1).transpose() << ", rw_new_start = " << result.y_rw.block(0, 0, 3, 1).transpose() << std::endl;
  */

  // Iterate to compute costs
  double cost = 0;
  double total_cost = 0;
  // store for debug
  this->cur_wrist_pos_cost_total = 0;
  this->cur_wrist_ori_cost_total = 0;
  this->cur_elbow_pos_cost_total = 0;
  this->cur_finger_pos_cost_total = 0;  
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // get the current joint value
    // _vertices is a VertexContainer type, a std::vector<Vertex*>
    const DualArmDualHandVertex *qv = static_cast<const DualArmDualHandVertex*>(_vertices[n+1]);
    const Matrix<double, JOINT_DOF, 1> x = qv->estimate(); // return the current estimate of the vertex
    //std::cout << "debug: x size is: " << x.rows() << " x " << x.cols() << std::endl;
    //std::cout << "debug: x = \n" << x.transpose() << std::endl;

    // Get joint angles
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
    q_cur_l = x.block<7, 1>(0, 0);
    q_cur_r = x.block<7, 1>(7, 0);
    q_cur_finger_l = x.block<12, 1>(14, 0);
    q_cur_finger_r = x.block<12, 1>(26, 0); 
    
    // Use DMP to generate new position trajectories, orientation & glove trajs are resampled and loaded in DMPTrajectoryGenerator
    // Set new goals(expected trajectory) to _measurement
    // left arm part:
    if (v->fixed()) // if DMP vertex is fixed, use data passed in
    {
      _measurement.l_wrist_pos_goal = _measurement.DMP_lw.block(0, n, 3, 1);
      _measurement.l_elbow_pos_goal = _measurement.DMP_le.block(0, n, 3, 1);
    }
    else
    {
      _measurement.l_wrist_pos_goal = result.y_lw.block(0, n, 3, 1); // Vector3d
      _measurement.l_elbow_pos_goal = result.y_le.block(0, n, 3, 1); // Vector3d
    }
    Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->l_wrist_quat_traj(n, 3));
    Matrix3d l_wrist_ori_goal = q_l.toRotationMatrix();
    _measurement.l_wrist_ori_goal = l_wrist_ori_goal; // Matrix3d

    // right arm part:
    if (v->fixed()) // if DMP vertex is fixed, use data passed in
    {
      _measurement.r_wrist_pos_goal = _measurement.DMP_rw.block(0, n, 3, 1);
      _measurement.r_elbow_pos_goal = _measurement.DMP_re.block(0, n, 3, 1);
    }
    else
    {
      _measurement.r_wrist_pos_goal = result.y_rw.block(0, n, 3, 1); // Vector3d
      _measurement.r_elbow_pos_goal = result.y_re.block(0, n, 3, 1); // Vector3d is column vector
    }
    Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(n, 0), 
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 1),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 2),
                    trajectory_generator_ptr->r_wrist_quat_traj(n, 3));
    Matrix3d r_wrist_ori_goal = q_r.toRotationMatrix();  
    _measurement.r_wrist_ori_goal = r_wrist_ori_goal; // Matrix3d


    // hand parts:
    _measurement.l_finger_pos_goal = trajectory_generator_ptr->l_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
    _measurement.r_finger_pos_goal = trajectory_generator_ptr->r_glove_angle_traj.block(n, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF

    /*
    std::cout << "l_wrist_pos_goal: \n" << _measurement.l_wrist_pos_goal.transpose() << "\n"
              << "l_wrist_ori_goal: \n" << _measurement.l_wrist_ori_goal << "\n"
              << "l_elbow_pos_goal: \n" << _measurement.l_elbow_pos_goal.transpose() << "\n"
              << "quaternion q_l: \n" << q_l.w() << " " << q_l.x() << " " << q_l.y() << " " << q_l.z() << "\n"

              << "r_wrist_pos_goal: \n" << _measurement.r_wrist_pos_goal.transpose() << "\n"
              << "r_wrist_ori_goal: \n" << _measurement.r_wrist_ori_goal << "\n"
              << "r_elbow_pos_goal: \n" << _measurement.r_elbow_pos_goal.transpose() << "\n"
              << "quaternion q_r: \n" << q_r.w() << " " << q_r.x() << " " << q_r.y() << " " << q_r.z() << "\n"

              << "l_finger_pos_goal: \n" << _measurement.l_finger_pos_goal.transpose() << "\n"
              << "r_finger_pos_goal: \n" << _measurement.r_finger_pos_goal.transpose() << std::endl;
    */

    //21,22,23,24, 25,26,27,28, 29,30,31,32, 33,34,
    //35,36,37,38,39,40,41,42,43,44,45,46,47,48

    
    // Compute unary costs
    // 1
    double arm_cost = compute_arm_cost(left_fk_solver, q_cur_l, _measurement.l_num_wrist_seg, _measurement.l_num_elbow_seg, true, _measurement); // user data is stored in _measurement now
    this->cur_wrist_pos_cost_total += this->cur_wrist_pos_cost;
    this->cur_wrist_ori_cost_total += this->cur_wrist_ori_cost;
    this->cur_elbow_pos_cost_total += this->cur_elbow_pos_cost;
    arm_cost += compute_arm_cost(right_fk_solver, q_cur_r, _measurement.r_num_wrist_seg, _measurement.r_num_elbow_seg, false, _measurement);
    this->cur_wrist_pos_cost_total += this->cur_wrist_pos_cost;
    this->cur_wrist_ori_cost_total += this->cur_wrist_ori_cost;
    this->cur_elbow_pos_cost_total += this->cur_elbow_pos_cost;
    //std::cout << "Arm cost=";
    //std::cout << arm_cost << ", ";

    // 2
    double finger_cost = compute_finger_cost(q_cur_finger_l, true, _measurement);  
    this->cur_finger_pos_cost_total += this->cur_finger_pos_cost;
    finger_cost += compute_finger_cost(q_cur_finger_r, false, _measurement);  
    this->cur_finger_pos_cost_total += this->cur_finger_pos_cost; // K_FINGER_POS included
    //std::cout << "Finger cost=";
    //std::cout << finger_cost << ", ";

    // total cost
    cost = arm_cost + K_FINGER * finger_cost; // arm_cost is already weighted in compute_arm_cost()
    //std::cout << "debug: arm_cost = " << arm_cost << ", finger_cost = " << finger_cost << std::endl;
    total_cost += cost;

  }

  // compute the cost (constraint value)
  _error(0, 0) = total_cost;

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_tracking += t_spent.count();

}

#endif
