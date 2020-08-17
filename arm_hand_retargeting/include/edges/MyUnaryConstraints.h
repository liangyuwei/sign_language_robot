#ifndef MY_UNARY_CONSTRAINTS_H
#define MY_UNARY_CONSTRAINTS_H

// Common
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <vector>
#include <string>

// G2O: infrastructure
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/QR>

// G2O Vertices
#include "vertices/DualArmDualHandVertex.h" 

// Self-defined helper variables
#include "config.h"

using namespace g2o;
using namespace Eigen;


/**
 * Constraint edge for collision and position limit costs.
 */
class MyUnaryConstraints : public BaseUnaryEdge<1, my_constraint_struct, DualArmDualHandVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Constructor with initialization of KDL FK solvers and collision checker.
    MyUnaryConstraints(boost::shared_ptr<DualArmDualHandCollision> &_dual_arm_dual_hand_collision_ptr) : dual_arm_dual_hand_collision_ptr(_dual_arm_dual_hand_collision_ptr){};

    /// Compute edge value. Used by g2o internal calculation
    void computeError();

    // functions for recording costs
    double return_pos_limit_cost();   ///< Return current position limit cost value.
    double return_col_cost();         ///< Return number of colliding path points.

    // display for debug
    void output_distance_result();    ///< Print collision condition of the current state.

    /// Compute q updates using robot jacobian
    Eigen::MatrixXd compute_col_q_update(Eigen::MatrixXd jacobian, Eigen::Vector3d dx, double speed);

    // Resolve colliding state by directly operating on joint values
    std::vector<Eigen::Matrix<double, JOINT_DOF, 1>> resolve_path_collisions(std::vector<Eigen::Matrix<double, JOINT_DOF, 1>> q_cur);
    Eigen::Matrix<double, JOINT_DOF, 1> resolve_point_collisions(Eigen::Matrix<double, JOINT_DOF, 1> q_cur, double col_eps); // called in resolve_path_collisions() to resolve colliding state for each path point
  
    /// Read from disk, leave blank
    virtual bool read( std::istream& in ) {return true;}
    
    /// Write to disk, leave blank
    virtual bool write( std::ostream& out ) const {return true;}

    /// Re-implement linearizeOplus for jacobians calculation
    virtual void linearizeOplus();

    // variables to store jacobians of each and both constraints
    Matrix<double, JOINT_DOF, 1> col_jacobians = Matrix<double, JOINT_DOF, 1>::Zero();
    Matrix<double, JOINT_DOF, 1> pos_limit_jacobians;
    Matrix<double, JOINT_DOF, 1> whole_jacobians;


  private:
    /// Collision checker (with distance computation functionality)
    boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr;

    /// Compute collision cost, using the minimum distance 
    double compute_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    
    /// Compute collision cost with only the hands part
    double compute_dual_hands_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, std::string link_name_1, std::string link_name_2, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    
    /// Compute position limit cost
    double compute_pos_limit_cost(Matrix<double, JOINT_DOF, 1> q_whole, my_constraint_struct &fdata);

    // Safety setting related to collision avoidance, note that d_check must be stricly greater than d_safe !!!
    // use different margins of safety for arms and hands
    // here the safety margin should be set according to actual condition: set a collision-free state and check minimum distance for dual_arms and dual_hands using collision_checking_yumi.cpp to have a sense of it
    double d_arm_check = 0.003;  ///< Distance threshold under which to check collision, note that d_check must be strictly greater than d_safe.
    double d_arm_safe = 0.001;   ///< Safety margin for arm part. In the current condition, 0.01 would actually be too large, for some links are very close to each other, e.g. _5_l and _7_l.
    double d_hand_check = 0.002; ///< Distance threshold under which to check collision, note that d_check must be strictly greater than d_safe. 
    double d_hand_safe = 1e-6;   ///< Safety margin for hand part. A small value close to 0 is fine since link51 and link111 are really close to each other under initial collision-free state.
};


/**
 * Compute collision cost and position limit cost, and store them for debug.
 */
void MyUnaryConstraints::computeError()
{
  // statistics
  count_unary++;  
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

  // get the current joint value
  // _vertices is a VertexContainer type, a std::vector<Vertex*>
  const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
  const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
  q_cur_l = x.block<7, 1>(0, 0);
  q_cur_r = x.block<7, 1>(7, 0);
  q_cur_finger_l = x.block<12, 1>(14, 0);
  q_cur_finger_r = x.block<12, 1>(26, 0); 
  
  // Compute unary costs
  // 1
  double collision_cost = compute_collision_cost(x, dual_arm_dual_hand_collision_ptr);
  
  // 2
  double pos_limit_cost = compute_pos_limit_cost(x, _measurement);

  // total cost
  double cost = K_COL * collision_cost + K_POS_LIMIT * pos_limit_cost;
  _error(0, 0) = cost;

  // Display message
  //std::cout << "Computing unary edge's cost: " << cost << std::endl;
  
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_01 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_unary += t_01.count();
}


/**
 * For g2o internal call. \n
 * Calculate jacobians for collision cost and position limit cost. \n
 * For collision part, we adopt gradient normal + robot jacobian method for any arm-arm, arm-hand, hand-hand(different) collision, 
 * and compute numerical derivatives for same-hand finger collision because of the limited DOF for robot fingers which would yeild incorrect dq, 
 * e.g. only 2 DOF for index finger while the given dx is 3-dim data.
 */
void MyUnaryConstraints::linearizeOplus()
{
  // Initialization
  _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();
  
  // epsilons
  double col_eps = 3.0 * M_PI / 180.0; //0.05; // in radius, approximately 3 deg
  double simple_update = 0.2;//0.1;//0.1; // update step for (4,0) or (0,4), i.e. only one end is in colliding state, close to boundary
  double pos_limit_eps = 0.02;

  // Get current joint angles
  DualArmDualHandVertex *v = dynamic_cast<DualArmDualHandVertex*>(_vertices[0]);
  Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex
  Matrix<double, JOINT_DOF, 1> delta_x = Matrix<double, JOINT_DOF, 1>::Zero();

  // Real-time collision checking strategy using robot jacobian and normal
  // double e_cur = compute_collision_cost(x, dual_arm_dual_hand_collision_ptr);
  // convert from matrix to std::vector
  std::vector<double> xx(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    xx[i] = x[i];

  // iterate to calculate pos_limit jacobians
  double e_plus, e_minus;
  for (unsigned int d = 0; d < JOINT_DOF; d++)
  {
    // Position Limit
    delta_x[d] = pos_limit_eps;
    e_plus = compute_pos_limit_cost(x+delta_x, _measurement);
    e_minus = compute_pos_limit_cost(x-delta_x, _measurement);
    _jacobianOplusXi(0, d) += K_POS_LIMIT * (e_plus - e_minus) / (2*pos_limit_eps);
    pos_limit_jacobians[d] = K_POS_LIMIT * (e_plus - e_minus) / (2*pos_limit_eps);

    whole_jacobians[d] = _jacobianOplusXi(0, d);

    // Reset
    delta_x[d] = 0.0;
  }


  // Skip collision 
  if (K_COL == 0.0)
  {
    return;
  }


  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
  double e_cur;
  double speed = 1.0;//1000.0;//10.0;//1.0;//100.0;//1.0;//10.0;//50.0;//100.0;//10.0;//1.0;//0.1;//1.0; // speed up collision updates, since .normal is a normalized vector, we may need this term to modify the speed (or step)  
  double hand_speed = 100.0;//1.0;//100.0; //500.0;//200.0;//10.0;//1.0;//400.0;//200.0;//100.0;// 1.0; // for collision between links belonging to the same hand !!!
  double min_distance;
  // check arms first, if ok, then hands. i.e. ensure arms safety before checking hands
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(xx, "dual_arms", d_arm_check); 
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_col += t_spent.count();
  count_col++;
  // if *dual_arms* group does not satisfy safety requirement, the lastest distance result would be used directly
  // if not, *dual_hands* group would be checked and overwriting the last distance result
  if (min_distance > d_arm_safe) // arm safe
  {
    // check hands 
    t0 = std::chrono::steady_clock::now();
    min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(xx, "dual_hands", d_hand_check); 
    t1 = std::chrono::steady_clock::now();
    t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    total_col += t_spent.count();
    count_col++;
    e_cur = std::max(d_hand_safe - min_distance, 0.0);
  }
  else
  {
    e_cur = std::max(d_arm_safe - min_distance, 0.0); // <0 means colliding, >0 is ok
  }


  // Determine updates when in collision state or safety is not met
  if (e_cur > 0.0) // in collision state or within safety of margin
  {
    // std::cout << "debug: e_cur = " << e_cur << std::endl;

    // get group belonging information: 0 - left_arm, 1 - right_arm, 2 - left_hand, 3 - right_hand, -1 - others
    int group_id_1 = dual_arm_dual_hand_collision_ptr->check_link_belonging(dual_arm_dual_hand_collision_ptr->link_names[0]);
    int group_id_2 = dual_arm_dual_hand_collision_ptr->check_link_belonging(dual_arm_dual_hand_collision_ptr->link_names[1]);
    
    // prep
    std::string link_name_1 = dual_arm_dual_hand_collision_ptr->link_names[0];
    std::string link_name_2 = dual_arm_dual_hand_collision_ptr->link_names[1];

    // std::cout << "debug: Possible collision between " << link_name_1 << " and " << link_name_2 << std::endl;
    // std::cout << "debug: minimum distance is: " << dual_arm_dual_hand_collision_ptr->min_distance << std::endl;

    // calculate global location of nearest/colliding links (reference position is independent of base frame, so don't worry)
    Eigen::Vector3d link_pos_1 = dual_arm_dual_hand_collision_ptr->get_global_link_transform(link_name_1);
    Eigen::Vector3d link_pos_2 = dual_arm_dual_hand_collision_ptr->get_global_link_transform(link_name_2);
    Eigen::Vector3d ref_point_pos_1 = dual_arm_dual_hand_collision_ptr->nearest_points[0] - link_pos_1;
    Eigen::Vector3d ref_point_pos_2 = dual_arm_dual_hand_collision_ptr->nearest_points[1] - link_pos_2;    

    // Inspect the circumstance
    if ((group_id_1 == 0 && group_id_2 == 0) ||
        (group_id_1 == 0 && group_id_2 == 1) || 
        (group_id_1 == 1 && group_id_2 == 0) || 
        (group_id_1 == 1 && group_id_2 == 1) ) // collision between arm and arm (could be the same), update only q_arm
    {
      // debug display
      // std::cout << "debug: possible collision between arm and arm... update only q_arm" << std::endl;
      
      // determine left or right
      bool left_or_right_1 = (group_id_1 == 0); 
      bool left_or_right_2 = (group_id_2 == 0);

      // compute jacobians (for arms) - return is 6 x 7, 6 for instantaneous pos/ori ([dx, dy, dz, dp, dq, dw])
      Eigen::MatrixXd jacobian_1 = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_1, 
                                                                                            ref_point_pos_1, 
                                                                                            left_or_right_1);
      Eigen::MatrixXd jacobian_2 = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_2, 
                                                                                            ref_point_pos_2, 
                                                                                            left_or_right_2);  
      
      // note that .normal is the normalized vector pointing from link_names[0] to link_names[1]; output is column vector with size N x 1
      Eigen::MatrixXd dq_col_update_1 = this->compute_col_q_update(jacobian_1, - dual_arm_dual_hand_collision_ptr->normal, speed);
      Eigen::MatrixXd dq_col_update_2 = this->compute_col_q_update(jacobian_2, dual_arm_dual_hand_collision_ptr->normal, speed);

      // std::cout << "debug: \n" << "dq_col_update_1 = " << dq_col_update_1.transpose() 
      //                          << ", dq_col_update_2 = " << dq_col_update_2.transpose() << std::endl;

      // assign jacobians for collision cost
      // init
      _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();
      // first link
      if (left_or_right_1)
      {
        _jacobianOplusXi.block(0, 0, 1, 7) += dq_col_update_1.transpose();
      }
      else
      {
        _jacobianOplusXi.block(0, 7, 1, 7) += dq_col_update_1.transpose();
      }
      std::cout << "1 - jacobian = " << _jacobianOplusXi << std::endl;
      // for debug
      bool debug = isnan(_jacobianOplusXi.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
        double a;
      }

      // second link
      if (left_or_right_2)
      {
        _jacobianOplusXi.block(0, 0, 1, 7) += dq_col_update_2.transpose();
      }
      else
      {
        _jacobianOplusXi.block(0, 7, 1, 7) += dq_col_update_2.transpose();
      }
      std::cout << "2 - jacobian = " << _jacobianOplusXi << std::endl;

      // for debug
      debug = isnan(_jacobianOplusXi.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
        double a;
      }

    }
    else if ((group_id_1 == 2 && group_id_2 == 2) || (group_id_1 == 3 && group_id_2 == 3) ) // collision between hand and hand (the same one), update only q_finger
    {
      // debug display
      // std::cout << "debug: possible collision between fingers of the same hand... update only q_finger" << std::endl;

      // prep
      bool left_or_right = (group_id_1 == 2);
      int finger_id_1 = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_1, left_or_right);
      int finger_id_2 = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_2, left_or_right);

      // compute robot jacobians (for fingers), return is 6 x N, N could be 4(thumb) or 2(others); and compute updates
      // use gradient normal + robot jacobians method for finger, which would yield incorrect dq
      /*
      Eigen::MatrixXd jacobian_1;
      Eigen::MatrixXd jacobian_2;
      Eigen::MatrixXd dq_col_update_1;
      Eigen::MatrixXd dq_col_update_2;
      std::cout << "Current joint values = " << x.transpose() << std::endl;
      if (finger_id_1 != -1) // if not belonging to palm groups!!!
      {
        jacobian_1 = dual_arm_dual_hand_collision_ptr->get_robot_hand_jacobian(link_name_1, 
                                                                               ref_point_pos_1, 
                                                                               finger_id_1,
                                                                               left_or_right);
        dq_col_update_1 = this->compute_col_q_update(jacobian_1, - dual_arm_dual_hand_collision_ptr->normal, hand_speed*speed);                                                                               
        std::cout << "debug: \n" << "dq_col_update_1 = " << dq_col_update_1.transpose() << std::endl;
      }
      if (finger_id_2 != -1) // if not belonging to palm groups!!!
      {
        jacobian_2 = dual_arm_dual_hand_collision_ptr->get_robot_hand_jacobian(link_name_2, 
                                                                               ref_point_pos_2, 
                                                                               finger_id_2,
                                                                               left_or_right);
        dq_col_update_2 = this->compute_col_q_update(jacobian_2, dual_arm_dual_hand_collision_ptr->normal, hand_speed*speed);                                                                               
        std::cout << "debug: \n" << "dq_col_update_2 = " << dq_col_update_2.transpose() << std::endl;
      }
      */


      // assign jacobians for collision cost
      // init
      _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();
      Matrix<double, 1, JOINT_DOF> jacobian_way_1 = Matrix<double, 1, JOINT_DOF>::Zero();
      Matrix<double, 1, JOINT_DOF> jacobian_way_2 = Matrix<double, 1, JOINT_DOF>::Zero();
      unsigned int d = left_or_right ? 0 : 12;
      double e_up, e_down;
      // first link
      switch (finger_id_1)
      {
        case 0: // thumb
          for (unsigned int s = 0; s < 4; s++)
          {
            // set
            delta_x[22+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 22+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 22+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 22+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 22 + d, 1, 4) += dq_col_update_1.transpose();
            // reset
            delta_x[22+d+s] = 0.0;
          }
          break;
        case 1: //index
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[14+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 14+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 14+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 14+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 14 + d, 1, 2) += dq_col_update_1.transpose();
            // reset
            delta_x[14+d+s] = 0.0;
          }
          break;
        case 2: //middle
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[16+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 16+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 16+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 16+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 16 + d, 1, 2) += dq_col_update_1.transpose();
            // reset
            delta_x[16+d+s] = 0.0;
          }
          break;
        case 3: // ring
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[18+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 18+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 18+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 18+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 18 + d, 1, 2) += dq_col_update_1.transpose();
            // reset
            delta_x[18+d+s] = 0.0;
          }
          break;
        case 4: // little
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[20+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 20+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 20+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 20+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 20 + d, 1, 2) += dq_col_update_1.transpose();
            // reset
            delta_x[20+d+s] = 0.0;
          }
          break;
        case -1: // palm, do nothing
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }
      // std::cout << "1 - jacobian = " << _jacobianOplusXi << std::endl;
      // std::cout << "1 - jacobian_way_1 = " << jacobian_way_1 << std::endl;
      // std::cout << "1 - jacobian_way_2 = " << jacobian_way_2 << std::endl;

      // for debug
      bool debug = isnan(_jacobianOplusXi.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
        double a;
      }

      // second link
      switch (finger_id_2)
      {
        case 0: // thumb
          for (unsigned int s = 0; s < 4; s++)
          {
            // set
            delta_x[22+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 22+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 22+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 22+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 22 + d, 1, 4) += dq_col_update_2.transpose();
            // reset
            delta_x[22+d+s] = 0.0;
          }
          break;
        case 1: //index
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[14+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 14+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 14+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 14+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 14 + d, 1, 2) += dq_col_update_2.transpose();
            // reset
            delta_x[14+d+s] = 0.0;
          }
          break;
        case 2: //middle
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[16+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 16+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 16+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 16+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 16 + d, 1, 2) += dq_col_update_2.transpose();
            // reset
            delta_x[16+d+s] = 0.0;
          }
          break;
        case 3: // ring
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[18+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 18+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); // 
            jacobian_way_1(0, 18+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 18+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 18 + d, 1, 2) += dq_col_update_2.transpose();
            // reset
            delta_x[18+d+s] = 0.0;
          }
          break;
        case 4: // little
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[20+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            _jacobianOplusXi(0, 20+d+s) +=  hand_speed * (e_up - e_down) / (2*col_eps); //
            jacobian_way_1(0, 20+d+s) += K_COL * ((e_up > e_down) ? simple_update : -simple_update);
            jacobian_way_2(0, 20+d+s) += hand_speed * (e_up - e_down) / (2*col_eps); //
          // _jacobianOplusXi.block(0, 20 + d, 1, 2) += dq_col_update_2.transpose();
            // reset
            delta_x[20+d+s] = 0.0;
          }
          break;
        case -1: // palm, do nothing
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }

      // std::cout << "2 - jacobian = " << _jacobianOplusXi << std::endl;
      // std::cout << "2 - jacobian_way_1 = " << jacobian_way_1 << std::endl;
      // std::cout << "2 - jacobian_way_2 = " << jacobian_way_2 << std::endl;

       // for debug
      debug = isnan(_jacobianOplusXi.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
        double a;
      }

    }
    else if (group_id_1 == -1 || group_id_2 == -1) // one of the object doesn't belong to arms or hands, update only one arm+hand
    {

      // std::cout << "debug: possible collision between robot arms/hands and others..." << std::endl;

      // just in case
      if (group_id_1 == -1 && group_id_2 == -1)
      {
        std::cerr << "Error: Something might be wrong, both links in collision do not belong to arms or hands, which should never happen.." << std::endl;
        exit(-1);
      }

      // prep
      int group_id = (group_id_1 == -1) ? group_id_2 : group_id_1;
      std::string link_name = (group_id_1 == -1) ? link_name_2 : link_name_1;
      Eigen::Vector3d ref_point_pos = (group_id_1 == -1) ? ref_point_pos_2 : ref_point_pos_1;
      Eigen::MatrixXd jacobian;
      Eigen::MatrixXd dq_col_update;

      // initialization
      _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();

      // decide if the link is arm or finger
      if (group_id == 2 || group_id == 3) // hand (should update for arm+hand group)
      {
        // compute robot jacobians
        bool left_or_right = (group_id == 2);
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name, left_or_right);
        if (finger_id != -1) // if not palm group !!!
        {
          jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name,
                                                                                  ref_point_pos,
                                                                                  finger_id,
                                                                                  left_or_right);
          // compute updates
          double direction = (group_id_1 == -1) ? 1.0 : -1.0;
          dq_col_update = this->compute_col_q_update(jacobian, direction * dual_arm_dual_hand_collision_ptr->normal, speed);
          // std::cout << "debug: \n" << "dq_col_update = " << dq_col_update.transpose() << std::endl;                                                                                  
        }
        

        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();
        // hand part
        unsigned int d_hand = left_or_right ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            _jacobianOplusXi.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            _jacobianOplusXi.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            _jacobianOplusXi.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            _jacobianOplusXi.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, do nothing
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }
      else // arm (update only for arm group)
      {
        // compute robot jacobians
        bool left_or_right = (group_id == 0);
        jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name,
                                                                            ref_point_pos,
                                                                            left_or_right);
        
        // compute updates
        double direction = (group_id_1 == -1) ? 1.0 : -1.0;
        dq_col_update = this->compute_col_q_update(jacobian, direction * dual_arm_dual_hand_collision_ptr->normal, speed);
        // std::cout << "debug: \n" << "dq_col_update = " << dq_col_update.transpose() << std::endl;

        // assign updates to col jacobian
        unsigned int d_arm = left_or_right ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();

        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }

      // std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
  
    }
    else // all the other conditions, update both q_arm and q_finger ()
    {
      // std::cout << "debug: Possible collision between arm and hand, or left and right hands... update both q_arm and q_finger" << std::endl;

      // prep
      bool left_or_right_1 = ( (group_id_1 == 0) || (group_id_1 == 2) );
      bool left_or_right_2 = ( (group_id_2 == 0) || (group_id_2 == 2) );
      // initialization
      _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero();

      // process the first link
      if (group_id_1 == 2 || group_id_1 == 3) // hand, should calculate robot jacobian for arm+hand group
      {
        // prep
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_1, left_or_right_1);

        // compute robot jacobian for arm+hand group and compute updates
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name_1,
                                                                                                 ref_point_pos_1,
                                                                                                 finger_id,
                                                                                                 left_or_right_1);
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, - dual_arm_dual_hand_collision_ptr->normal, speed);                                                                                  
        // std::cout << "debug: \n" << "dq_col_update_1 = " << dq_col_update.transpose() << std::endl;


        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right_1 ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

        // hand part
        unsigned int d_hand = left_or_right_1 ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            _jacobianOplusXi.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            _jacobianOplusXi.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            _jacobianOplusXi.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            _jacobianOplusXi.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, zero updates would be provided for finger joints, but the size is in consistent with little finger_group since it's used in get_robot_arm_hand_jacobian()
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
        // for debug
        debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }
      else // 0 or 1, arm, should calculate robot jacobian for arm group
      {

        // compute robot jacobian for arm group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_1,
                                                                                            ref_point_pos_1,
                                                                                            left_or_right_1);
        
        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, - dual_arm_dual_hand_collision_ptr->normal, speed);
        // std::cout << "debug: \n" << "dq_col_update_1 = " << dq_col_update.transpose() << std::endl;

        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right_1 ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }
      }
      // std::cout << "debug: 1 - jacobian = " << _jacobianOplusXi << std::endl;
      
     
      // process the second link
      if (group_id_2 == 2 || group_id_2 == 3) // hand, should calculate robot jacobian for arm+hand group
      {
        // prep
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_2, left_or_right_2);


        // compute robot jacobian for arm+hand group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name_2,
                                                                                                 ref_point_pos_2,
                                                                                                 finger_id,
                                                                                                 left_or_right_2);

        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, dual_arm_dual_hand_collision_ptr->normal, speed);
       

        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right_2 ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

        // hand part
        unsigned int d_hand = left_or_right_2 ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            _jacobianOplusXi.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            _jacobianOplusXi.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            _jacobianOplusXi.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            _jacobianOplusXi.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, zero updates would be provided for finger joints, but the size is in consistent with little finger_group since it's used in get_robot_arm_hand_jacobian()
            _jacobianOplusXi.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
        // for debug
        debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }
      else // 0 or 1, arm, should calculate robot jacobian for arm group
      {

        // compute robot jacobian for arm group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_2,
                                                                                            ref_point_pos_2,
                                                                                            left_or_right_2);
        
        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, dual_arm_dual_hand_collision_ptr->normal, speed);
        // std::cout << "debug: \n" << "dq_col_update_2 = " << dq_col_update.transpose() << std::endl;

        // assign updates to col jacobian
        // arm part
        unsigned int d_arm = left_or_right_2 ? 0 : 7;
        _jacobianOplusXi.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();
        // for debug
        bool debug = isnan(_jacobianOplusXi.norm());
        if (debug)
        {
          std::cout << "debug: jacobian = " << _jacobianOplusXi << std::endl;
          double a;
        }

      }
      // std::cout << "debug: 2 - jacobian = " << _jacobianOplusXi << std::endl;

    } // END of calculating collision jacobian for optimization
  }
  else // in collision-free state and outside of safety margin (meaning safe now)
  {
    _jacobianOplusXi = Matrix<double, 1, JOINT_DOF>::Zero(); // assign no updates for avoiding collision
  } // END of calculating collision jacobian for optimization

  // debug:
  // std::cout << "debug: col jacobians = " << _jacobianOplusXi << std::endl;

  // record col jacobians
  col_jacobians = _jacobianOplusXi.transpose();

  // std::cout << "Current Col jacobian = " << _jacobianOplusXi << std::endl;

  // debug:
  /*
  std::cout << "debug: norms of col_jacobian = " << col_jacobians.norm() << std::endl;
  std::cout << "debug: norms of whole_jacobian = " << whole_jacobians.norm() << std::endl;
  std::cout << "debug: col_jacobians = " << col_jacobians.transpose() << std::endl;
  std::cout << "debug: whole_jacobians = " << whole_jacobians.transpose() << std::endl;

  bool result1 = isnan(col_jacobians.norm());
  bool result2 = isnan(whole_jacobians.norm());

  if (result1)
  {
    std::cout << "debug: col_jacobians contains NaN!!!" << std::endl;
    double a;
  }

  if (result2)
  {
    std::cout << "debug: whole_jacobians contains NaN!!!" << std::endl;
    double a;
  }
  // std::cout << "debug: unary jacobians = " <<  _jacobianOplusXi << std::endl;
  //double debug;
  */

}


/** 
 * This function outputs the pair of links that are closest to each other and the corresponding minimum distance,
 *  for checking on collision. 
 */
void MyUnaryConstraints::output_distance_result()
{
  // std::cout << "Distance result: " << std::endl;
  std::cout << "Collision between " << dual_arm_dual_hand_collision_ptr->link_names[0] << " and " 
                                    << dual_arm_dual_hand_collision_ptr->link_names[1] << ", with min_dist = "
                                    << dual_arm_dual_hand_collision_ptr->min_distance << "." << std::endl;

}


/**
 * This function computes q velocity from Cartesian velocity, through the use of robot jacobian and gradient normal. \n
 * From the theory of robotics, we have [dx, dy, dz, dp, dq, dw] = J * dq, i.e. [d_pos, d_ori] = J * dq. \n
 * Here we computes dq from the given d_pos = [dx, dy, dz] and J, the jacobian matrix.
 * @param[in]   jacobian  The robot jacobian at a particular point, with the size of 6 x N where N is the number of joints related.
 * @param[in]   d_pos     The position part of end-effector Cartesian-space update.
 * @param[in]   speed     The scale ratio to control the update step. With too large speed, jacobian would be inaccurate.
 * @param[out]  d_q       Joint velocity, with the size of N x 1.
 */
Eigen::MatrixXd MyUnaryConstraints::compute_col_q_update(Eigen::MatrixXd jacobian, Eigen::Vector3d d_pos, double speed)
{
  // dx = J * dq, given dx and J, solve for dq ==> J'*(J*J')^-1 * dx = dq. This is solvable only when rank(J) = rank(J, dx).
  // So when rank(J) != 6, there is rank(J,dx) > rank(J), and the result would either be incorrect or NaN.
  // Constrain the rows of J to 3, i.e. processing only position data, this way when row rank of J is >= 3, there is rank(J, dx) >= 3, and therefore rank(J, dx) = 3 = rank(J).

  // prep
  Eigen::Matrix<double, 6, 1> d_pos_ori = Eigen::Matrix<double, 6, 1>::Zero();
  d_pos_ori.block(0, 0, 3, 1) = speed * d_pos;// //d_pos; // the jacobian computed should be at the direction that cost increases, here d_pos is the direction for cost to decrease.
  unsigned int num_cols = jacobian.cols();
  Eigen::MatrixXd d_q;


  // pre-processing J and dx to cope with rank issue. If rank is smaller than 6, J rows will be reduced to the same number, and so does d_pos_ori.
  // *** Pros and Cons: Although dpos calculated by J*dq would be consistent with d_pos_ori, J*dq might produce huge deviation in dori part !!!
  unsigned int rank = jacobian.colPivHouseholderQr().rank(); 
  Eigen::MatrixXd d_x = d_pos_ori;//d_pos_ori.block(0, 0, rank, 1);
  Eigen::MatrixXd J = jacobian;//jacobian.block(0, 0, rank, num_cols);
  // std::cout << "debug: rank(J) = " << rank << std::endl;
  // std::cout << "debug: original J = " << jacobian << std::endl;
  // std::cout << "debug: processed J = " << J << std::endl;
  // std::cout << "debug: original dx = " << d_pos_ori.transpose() << std::endl;
  // std::cout << "debug: processed dx = " << d_x.transpose() << std::endl;


  // solve for dq
  std::chrono::steady_clock::time_point t00 = std::chrono::steady_clock::now();
  // 1 - direct calculation
  // d_q = J.transpose() * (J * J.transpose()).inverse() * d_x;
  // 2 - use SVD
  JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV); // svd.singularValues()/.matrixU()/.matrixV()
  d_q = svd.solve(d_x);
  std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_0011 = std::chrono::duration_cast<std::chrono::duration<double>>(t11 - t00);
  // std::cout << "SVD solution took " << t_0011.count() << " s." << std::endl;
  // debug:
  // std::cout << "debug: d_q = " << d_q.transpose() << std::endl;
  // std::cout << "debug: original J * dq = " << (jacobian * d_q).transpose() << std::endl;
  // std::cout << "debug: processed J * dq = " << (J * d_q).transpose() << std::endl;
  // std::cout << "debug: d_x = " << d_x.transpose() << std::endl;
  // std::cout << "debug: d_pos_ori = " << d_pos_ori.transpose() << std::endl;
  // std::cout << "size of d_q is: " << d_q.rows() << " x " << d_q.cols() << std::endl;

  // compute angle between dx and J*dq
  // Vector3d Jdq_pos = (jacobian * d_q).block(0, 0, 3, 1);
  // double ac = (double)(Jdq_pos.transpose() * d_pos) / (Jdq_pos.norm()*d_pos.norm());
  // double theta = std::acos( std::min(ac, 1.0) ) * 180.0 / M_PI;
  // std::cout << "debug: angle between the position vector of J*dq and dx = " << theta << " deg" << std::endl;


  // post-processing on dq, to speed up and apply weighting
  d_q = K_COL * d_q;

  return d_q;
}


/**
 * @brief Resolve any colliding state on a given path.
 * 
 * This function takes in the q results of TRAC-IK, and solves collision state via robot jacobians.(based on minimum distance) 
 */
std::vector<Eigen::Matrix<double, JOINT_DOF, 1>> MyUnaryConstraints::resolve_path_collisions(std::vector<Eigen::Matrix<double, JOINT_DOF, 1>> q_cur)
{
  // Prep
  double col_eps = 3.0 * M_PI / 180.0; // in radius

  // Iterate to resolve collision
  for (unsigned int n = 0; n < q_cur.size(); n++)
  {
    std::cout << ">> Processing point " << (n+1) << "/" << q_cur.size() << " ..." << std::endl;
    q_cur[n] = resolve_point_collisions(q_cur[n], col_eps);    
  }

  return q_cur;
}


/**
 * @brief Resolve collision state of a single path point.
 * 
 * This function is for ease of use, and is called by resolve_path_collisions(). \n
 * Colliding state of a path point is resolved through iteration with robot jacobians and gradient normal. \n
 * col_eps is the step for computing numerical differentiation for finger joints.
 */
Eigen::Matrix<double, JOINT_DOF, 1> MyUnaryConstraints::resolve_point_collisions(Eigen::Matrix<double, JOINT_DOF, 1> x, double col_eps)
{
  // Prep
  double e_cur;
  Eigen::Matrix<double, JOINT_DOF, 1> delta_x = Eigen::Matrix<double, JOINT_DOF, 1>::Zero();

  // Convert data type
  std::vector<double> xx(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    xx[i] = x[i];

  // Check arms first, if ok, then hands. i.e. ensure arms safety before checking hands
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  double min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(xx, "dual_arms", d_arm_check); 
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_col += t_spent.count();
  count_col++;
  // if *dual_arms* group does not satisfy safety requirement, the lastest distance result would be used directly
  // if not, *dual_hands* group would be checked and overwriting the last distance result
  if (min_distance > d_arm_safe) // arm safe
  {
    // check hands 
    t0 = std::chrono::steady_clock::now();
    min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(xx, "dual_hands", d_hand_check); 
    t1 = std::chrono::steady_clock::now();
    t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    total_col += t_spent.count();
    count_col++;
    e_cur = std::max(d_hand_safe - min_distance, 0.0);
  }
  else
  {
    e_cur = std::max(d_arm_safe - min_distance, 0.0); // <0 means colliding, >0 is ok
  }
    
  // Compute updates
  unsigned int cur_iter = 0;
  unsigned int max_iter = 500;
  double scale = 0.001; //1.0; // scale for [dx,dy,dz,dp,dq,dw] Cartesian updates; shouldn't be too large
  double hand_scale = 1.0; // should be set appropriately
  while(e_cur > 0.0 && cur_iter < max_iter) // in collision state or within safety of margin; and within the maximum number of iterations
  {
    // Initialize updates
    Matrix<double, 1, JOINT_DOF> dx = Matrix<double, 1, JOINT_DOF>::Zero();

    // Get collision information
    int group_id_1 = dual_arm_dual_hand_collision_ptr->check_link_belonging(dual_arm_dual_hand_collision_ptr->link_names[0]);
    int group_id_2 = dual_arm_dual_hand_collision_ptr->check_link_belonging(dual_arm_dual_hand_collision_ptr->link_names[1]);
    std::string link_name_1 = dual_arm_dual_hand_collision_ptr->link_names[0];
    std::string link_name_2 = dual_arm_dual_hand_collision_ptr->link_names[1];

    // Info
    std::cout << "debug: Possible collision between " << link_name_1 << " and " << link_name_2 << std::endl;
    std::cout << "debug: minimum distance is: " << dual_arm_dual_hand_collision_ptr->min_distance << std::endl;

    // calculate global location of nearest/colliding links (reference position is independent of base frame, so don't worry)
    Eigen::Vector3d link_pos_1 = dual_arm_dual_hand_collision_ptr->get_global_link_transform(link_name_1);
    Eigen::Vector3d link_pos_2 = dual_arm_dual_hand_collision_ptr->get_global_link_transform(link_name_2);
    Eigen::Vector3d ref_point_pos_1 = dual_arm_dual_hand_collision_ptr->nearest_points[0] - link_pos_1;
    Eigen::Vector3d ref_point_pos_2 = dual_arm_dual_hand_collision_ptr->nearest_points[1] - link_pos_2;    

    // Compute updates under different situation
    if ((group_id_1 == 0 && group_id_2 == 0) || (group_id_1 == 0 && group_id_2 == 1) || (group_id_1 == 1 && group_id_2 == 0) || (group_id_1 == 1 && group_id_2 == 1) ) // collision between arm and arm (could be the same), update only q_arm
    {
      // determine left or right
      bool left_or_right_1 = (group_id_1 == 0); 
      bool left_or_right_2 = (group_id_2 == 0);

      // compute jacobians (for arms) - return is 6 x 7, 6 for instantaneous pos/ori ([dx, dy, dz, dp, dq, dw])
      Eigen::MatrixXd jacobian_1 = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_1, ref_point_pos_1, left_or_right_1);
      Eigen::MatrixXd jacobian_2 = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_2, ref_point_pos_2, left_or_right_2);  
      
      // note that .normal is the normalized vector pointing from link_names[0] to link_names[1]; output is column vector with size N x 1
      Eigen::MatrixXd dq_col_update_1 = this->compute_col_q_update(jacobian_1, - dual_arm_dual_hand_collision_ptr->normal, scale);
      Eigen::MatrixXd dq_col_update_2 = this->compute_col_q_update(jacobian_2, dual_arm_dual_hand_collision_ptr->normal, scale);

      // assign jacobians for collision cost
      // first link
      if (left_or_right_1)
        dx.block(0, 0, 1, 7) += dq_col_update_1.transpose();
      else
        dx.block(0, 7, 1, 7) += dq_col_update_1.transpose();
      std::cout << "1 - jacobian = " << dx << std::endl;

      // second link
      if (left_or_right_2)
        dx.block(0, 0, 1, 7) += dq_col_update_2.transpose();
      else
        dx.block(0, 7, 1, 7) += dq_col_update_2.transpose();
      std::cout << "2 - jacobian = " << dx << std::endl;

    }
    else if ((group_id_1 == 2 && group_id_2 == 2) || (group_id_1 == 3 && group_id_2 == 3) ) // collision between hand and hand (the same one), update only q_finger
    {
      // prep
      bool left_or_right = (group_id_1 == 2);
      int finger_id_1 = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_1, left_or_right);
      int finger_id_2 = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_2, left_or_right);

      // assign jacobians for collision cost
      unsigned int d = left_or_right ? 0 : 12;
      double e_up, e_down;
      // first link
      switch (finger_id_1)
      {
        case 0: // thumb
          for (unsigned int s = 0; s < 4; s++)
          {
            // set
            delta_x[22+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 22+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[22+d+s] = 0.0;
          }
          break;
        case 1: //index
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[14+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 14+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[14+d+s] = 0.0;
          }
          break;
        case 2: //middle
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[16+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 16+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[16+d+s] = 0.0;
          }
          break;
        case 3: // ring
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[18+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 18+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[18+d+s] = 0.0;
          }
          break;
        case 4: // little
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[20+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 20+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[20+d+s] = 0.0;
          }
          break;
        case -1: // palm, do nothing
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }

      // second link
      switch (finger_id_2)
      {
        case 0: // thumb
          for (unsigned int s = 0; s < 4; s++)
          {
            // set
            delta_x[22+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 22+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[22+d+s] = 0.0;
          }
          break;
        case 1: //index
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[14+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 14+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[14+d+s] = 0.0;
          }
          break;
        case 2: //middle
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[16+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 16+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[16+d+s] = 0.0;
          }
          break;
        case 3: // ring
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[18+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 18+d+s) += hand_scale * (e_up - e_down) / (2*col_eps); // 
            // reset
            delta_x[18+d+s] = 0.0;
          }
          break;
        case 4: // little
          for (unsigned int s = 0; s < 2; s++)
          {
            // set
            delta_x[20+d+s] = col_eps;
            // compute
            e_up = compute_dual_hands_collision_cost(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            e_down = compute_dual_hands_collision_cost(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
            dx(0, 20+d+s) +=  hand_scale * (e_up - e_down) / (2*col_eps); //
            // reset
            delta_x[20+d+s] = 0.0;
          }
          break;
        case -1: // palm, do nothing
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }

      // update in the direction opposite to gradient !!!
      dx = -dx;

    }
    else if (group_id_1 == -1 || group_id_2 == -1) // one of the object doesn't belong to arms or hands, update only one arm+hand
    {
      // just in case
      if (group_id_1 == -1 && group_id_2 == -1)
      {
        std::cerr << "Error: Something might be wrong, both links in collision do not belong to arms or hands, which should never happen.." << std::endl;
        exit(-1);
      }

      // prep
      int group_id = (group_id_1 == -1) ? group_id_2 : group_id_1;
      std::string link_name = (group_id_1 == -1) ? link_name_2 : link_name_1;
      Eigen::Vector3d ref_point_pos = (group_id_1 == -1) ? ref_point_pos_2 : ref_point_pos_1;
      Eigen::MatrixXd jacobian;
      Eigen::MatrixXd dq_col_update;

      // decide if the link is arm or finger
      if (group_id == 2 || group_id == 3) // hand (should update for arm+hand group)
      {
        // compute robot jacobians
        bool left_or_right = (group_id == 2);
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name, left_or_right);
        if (finger_id != -1) // if not palm group !!!
        {
          jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name, ref_point_pos, finger_id, left_or_right);
          double direction = (group_id_1 == -1) ? 1.0 : -1.0;
          dq_col_update = this->compute_col_q_update(jacobian, direction * dual_arm_dual_hand_collision_ptr->normal, scale);
        }

        // arm part updates
        unsigned int d_arm = left_or_right ? 0 : 7;
        dx.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();
        
        // hand part updates
        unsigned int d_hand = left_or_right ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            dx.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            dx.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            dx.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            dx.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            dx.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, do nothing
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
      }
      else // arm (update only for arm group)
      {
        // compute robot jacobians
        bool left_or_right = (group_id == 0);
        jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name, ref_point_pos, left_or_right);
        
        // compute updates
        double direction = (group_id_1 == -1) ? 1.0 : -1.0;
        dq_col_update = this->compute_col_q_update(jacobian, direction * dual_arm_dual_hand_collision_ptr->normal, scale);

        // arm part updates
        unsigned int d_arm = left_or_right ? 0 : 7;
        dx.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();
      }  

    }
    else // all the other conditions, update both q_arm and q_finger ()
    {
      // prep
      bool left_or_right_1 = ( (group_id_1 == 0) || (group_id_1 == 2) );
      bool left_or_right_2 = ( (group_id_2 == 0) || (group_id_2 == 2) );
 
      // process the first link
      if (group_id_1 == 2 || group_id_1 == 3) // hand, should calculate robot jacobian for arm+hand group
      {
        // prep
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_1, left_or_right_1);

        // compute robot jacobian for arm+hand group and compute updates
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name_1, ref_point_pos_1, finger_id, left_or_right_1);
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, - dual_arm_dual_hand_collision_ptr->normal, scale);                                                                                  

        // arm part updates
        unsigned int d_arm = left_or_right_1 ? 0 : 7;
        dx.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();
        
        // hand part updates
        unsigned int d_hand = left_or_right_1 ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            dx.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            dx.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            dx.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            dx.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            dx.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, zero updates would be provided for finger joints, but the size is in consistent with little finger_group since it's used in get_robot_arm_hand_jacobian()
            dx.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
      }
      else // 0 or 1, arm, should calculate robot jacobian for arm group
      {
        // compute robot jacobian for arm group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_1, ref_point_pos_1, left_or_right_1);
        
        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, - dual_arm_dual_hand_collision_ptr->normal, scale);

        // arm part updates
        unsigned int d_arm = left_or_right_1 ? 0 : 7;
        dx.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();
      }
    
      // process the second link
      if (group_id_2 == 2 || group_id_2 == 3) // hand, should calculate robot jacobian for arm+hand group
      {
        // prep
        int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_2, left_or_right_2);

        // compute robot jacobian for arm+hand group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name_2, ref_point_pos_2, finger_id, left_or_right_2);

        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, dual_arm_dual_hand_collision_ptr->normal, scale);
      
        // arm part updates
        unsigned int d_arm = left_or_right_2 ? 0 : 7;
        dx.block(0, 0+d_arm, 1, 7) += dq_col_update.block(0, 0, 7, 1).transpose();

        // hand part updates
        unsigned int d_hand = left_or_right_2 ? 0 : 12;
        switch (finger_id)
        {
          case 0: // thummb
            dx.block(0, 22 + d_hand, 1, 4) += dq_col_update.block(7, 0, 4, 1).transpose();
            break;
          case 1: //index
            dx.block(0, 14 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 2: //middle
            dx.block(0, 16 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 3: // ring
            dx.block(0, 18 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case 4: // little
            dx.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          case -1: // palm, zero updates would be provided for finger joints, but the size is in consistent with little finger_group since it's used in get_robot_arm_hand_jacobian()
            dx.block(0, 20 + d_hand, 1, 2) += dq_col_update.block(7, 0, 2, 1).transpose();
            break;
          default:
            std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
            exit(-1);
            break;
        }
      }
      else // 0 or 1, arm, should calculate robot jacobian for arm group
      {
        // compute robot jacobian for arm group
        Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_2, ref_point_pos_2, left_or_right_2);
        
        // compute updates
        Eigen::MatrixXd dq_col_update = this->compute_col_q_update(jacobian, dual_arm_dual_hand_collision_ptr->normal, scale);

        // arm part updates
        unsigned int d_arm = left_or_right_2 ? 0 : 7;
        dx.block(0, 0+d_arm, 1, 7) += dq_col_update.transpose();
      }
      
    } // END of calculating collision jacobian for optimization


    // Update the q joint values
    x = x + dx.transpose();
    std::cout << "dx = " << dx << std::endl;

    // cope with position limit bounds
    double amount_out_of_bound = 0.0;
    for (unsigned int i = 0; i < JOINT_DOF; i++)
    {
      double tmp = std::min(std::max(x[i], _measurement.q_pos_lb[i]), _measurement.q_pos_ub[i]);
      amount_out_of_bound += std::fabs(x[i] - tmp);
      x[i] = tmp;//std::min(std::max(x[i], _measurement.q_pos_lb[i]), _measurement.q_pos_ub[i]);
    }
    std::cout << "amount of out-of-bounds = " << amount_out_of_bound << std::endl;

    // convert to std::vector<double> 
    for (unsigned int i = 0; i < JOINT_DOF; i++)
      xx[i] = x[i];

    // Check again, see if collision-free and within safety of margin
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    double min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(xx, "dual_arms", d_arm_check); 
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    total_col += t_spent.count();
    count_col++;
    if (min_distance > d_arm_safe) // arm safe
    {
      // check hands 
      t0 = std::chrono::steady_clock::now();
      min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(xx, "dual_hands", d_hand_check); 
      t1 = std::chrono::steady_clock::now();
      t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
      total_col += t_spent.count();
      count_col++;
      e_cur = std::max(d_hand_safe - min_distance, 0.0);
    }
    else
    {
      e_cur = std::max(d_arm_safe - min_distance, 0.0); // <0 means colliding, >0 is ok
    }

    std::cout << "debug: current e_cur = " << e_cur << std::endl << std::endl;


    // counter
    cur_iter++;

    // Check terminate condition
    if (e_cur <= 0.0)
      break;

  } // END of while

  std::cout << "Collision fix took " << cur_iter << "/" << max_iter << " rounds." << std::endl;

  // Check the results
  if (e_cur <= 0.0)
    std::cout << "Successfully resolve this collision!!!" << std::endl;
  else
    std::cerr << "Failed to resolve collision state of this path point... " << std::endl;

  
  return x;

}


/**
 * Compute the minimum distance between the specified two links. 
 * When the specified two links are not colliding, the distance is manually set to safety margin. \n
 * This is because the dual_hand part contains multiple fingers ("manipulators"), and we only want to
 * know about the collision state of the specified two links.
 */
double MyUnaryConstraints::compute_dual_hands_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, std::string link_name_1, std::string link_name_2, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr)
{
  // convert from matrix to std::vector
  std::vector<double> x(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    x[i] = q_whole[i];
  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
  double cost;
  double min_distance;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  min_distance = dual_arm_dual_hand_collision_ptr->compute_two_links_distance(x, link_name_1, link_name_2, d_hand_check);
  if (dual_arm_dual_hand_collision_ptr->link_names[0] != link_name_1 ||
      dual_arm_dual_hand_collision_ptr->link_names[1] != link_name_2) 
    min_distance = d_hand_safe; // if collision pair is changed, then the collision between the original two links is resolved !!!

  // std::cout << "debug: Possible collision between " << dual_arm_dual_hand_collision_ptr->link_names[0] 
  //           << " and " << dual_arm_dual_hand_collision_ptr->link_names[1] << std::endl;
  // std::cout << "debug: minimum distance is: " << dual_arm_dual_hand_collision_ptr->min_distance << std::endl;

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_col += t_spent.count();
  count_col++;
  cost = std::max(d_hand_safe - min_distance, 0.0);

  // apply weighting
  cost = K_COL * cost; // weighting...

  return cost;
}


/**
 * This function considers the whole robot model, and check arm part and hand part separately.
 */
double MyUnaryConstraints::compute_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr)
{
  // bypass collision checking
  if (K_COL == 0.0)
    return 0.0;

  // convert from matrix to std::vector
  std::vector<double> x(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    x[i] = q_whole[i];
  
  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
  double cost;
  double min_distance;
  // 1 - check arms first
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(x, "dual_arms", d_arm_check); 
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_col += t_spent.count();
  count_col++;
  // 2 check hands if arms are in safety region  
  if (min_distance > d_arm_safe) // arm safe
  {
    // check hands 
    t0 = std::chrono::steady_clock::now();
    min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(x, "dual_hands", d_hand_check); 
    t1 = std::chrono::steady_clock::now();
    t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    total_col += t_spent.count();
    count_col++;
    cost = std::max(d_hand_safe - min_distance, 0.0);
  }
  else
  {
    cost = std::max(d_arm_safe - min_distance, 0.0); // <0 means colliding, >0 is ok
  }
    
  // apply weighting
  cost = K_COL * cost; // weighting...

  return cost;
}


/**
 * Use l1 penalty for position limit cost. A safety margin can be modified.
 */
double MyUnaryConstraints::compute_pos_limit_cost(Matrix<double, JOINT_DOF, 1> q_whole, my_constraint_struct &fdata)
{
  // add safety margin for ease of limiting
  double safety_margin = 0.0;//0.01;

  // compute out-of-bound cost
  double cost = 0;
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
  {
    // within [lb+margin, ub-margin], sub-set of [lb, ub]
    if (fdata.q_pos_ub[i] - safety_margin < q_whole[i])
      cost += std::max(q_whole[i] - (fdata.q_pos_ub[i] - safety_margin), 0.0); //(q_whole[i] - fdata.q_pos_ub[i] + safety_margin) * (q_whole[i] - fdata.q_pos_ub[i] + safety_margin);
    if (fdata.q_pos_lb[i] + safety_margin > q_whole[i])
      cost += std::max((fdata.q_pos_lb[i] + safety_margin) - q_whole[i], 0.0);//(fdata.q_pos_lb[i] + safety_margin - q_whole[i]) * (fdata.q_pos_lb[i] + safety_margin - q_whole[i]);
  }

  return cost;
}


/**
 * Return collision cost without the weighting coefficient K_COL applied, 
 * and thus it's the collision state.
 */
double MyUnaryConstraints::return_col_cost()
{
  // get the current joint value
  // _vertices is a VertexContainer type, a std::vector<Vertex*>
  const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
  const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
  q_cur_l = x.block<7, 1>(0, 0);
  q_cur_r = x.block<7, 1>(7, 0);
  q_cur_finger_l = x.block<12, 1>(14, 0);
  q_cur_finger_r = x.block<12, 1>(26, 0); 
  
  // 3 (for ease of distinction, won't use distance_computation here)
  std::vector<double> xx(JOINT_DOF);
  for (unsigned int d = 0; d < JOINT_DOF; d++)
    xx[d] = x[d];
 
  double col_result = dual_arm_dual_hand_collision_ptr->check_self_collision(xx);
  double collision_cost = (col_result > 0.0)? 1.0 : 0.0; // col_result 1 for colliding, -1 for collision-free

  return collision_cost;
}


/**
 * Position limit cost is l1 penalty.
 */
double MyUnaryConstraints::return_pos_limit_cost()
{
  // get the current joint value
  // _vertices is a VertexContainer type, a std::vector<Vertex*>
  const DualArmDualHandVertex *v = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
  const Matrix<double, JOINT_DOF, 1> x = v->estimate(); // return the current estimate of the vertex

  // Get joint angles
  Matrix<double, 7, 1> q_cur_l, q_cur_r;
  Matrix<double, 12, 1> q_cur_finger_l, q_cur_finger_r;
  q_cur_l = x.block<7, 1>(0, 0);
  q_cur_r = x.block<7, 1>(7, 0);
  q_cur_finger_l = x.block<12, 1>(14, 0);
  q_cur_finger_r = x.block<12, 1>(26, 0); 
  
  double pos_limit_cost = compute_pos_limit_cost(x, _measurement);

  return pos_limit_cost;
}


#endif
