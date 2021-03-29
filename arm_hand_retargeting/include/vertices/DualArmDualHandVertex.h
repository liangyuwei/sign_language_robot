#ifndef DUAL_ARM_DUAL_HAND_VERTEX_H_
#define DUAL_ARM_DUAL_HAND_VERTEX_H_

// Common
#include <boost/shared_ptr.hpp>

// G2O: infrastructure
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/QR>

// For collision checker
#include "tools/collision_checking_yumi.h"

// Config
#include "config.h"

using namespace g2o;
using namespace Eigen;

/**
 * @brief Define vertex for joint values at a single time instance.
 *
 * Order is: left arm(7 DOF) -> right arm(7 DOF) -> left fingers(12 DOF) -> right fingers(12 DOF).
 */
class DualArmDualHandVertex : public BaseVertex<JOINT_DOF, Matrix<double, JOINT_DOF, 1> >
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Reset vertex state
    virtual void setToOriginImpl();

    /// Update vertex state
    virtual void oplusImpl(const double *update);

    /// Set joint position limits
    void set_bounds(const std::vector<double> _q_l_arm_lb, const std::vector<double> _q_l_arm_ub, 
                    const std::vector<double> _q_r_arm_lb, const std::vector<double> _q_r_arm_ub, 
                    const std::vector<double> _q_l_finger_lb, const std::vector<double> _q_l_finger_ub, 
                    const std::vector<double> _q_r_finger_lb, const std::vector<double> _q_r_finger_ub);
    
    /// Set collision checker 
    void set_collision_checker(boost::shared_ptr<DualArmDualHandCollision> _dual_arm_dual_hand_collision_ptr);
    
    /// Position lower bounds
    std::vector<double> q_lb;
    
    /// Position upper bounds
    std::vector<double> q_ub;
  
    /// Store the update values used during last call to oplusImpl()
    Matrix<double, JOINT_DOF, 1> last_update;

    /// Read from disk, leave blank
    virtual bool read( std::istream& in ) {return true;}

    /// Write to disk, leave blank
    virtual bool write( std::ostream& out ) const {return true;}


  private:
    /// Collision checker for use only within the scope of this class
    boost::shared_ptr<DualArmDualHandCollision> dual_arm_dual_hand_collision_ptr;
    
    /// Record the amount of out-of-bound part of update values
    double amount_out_of_bound;


};


/** 
 * Reset vertex state to zeros.
 */
void DualArmDualHandVertex::setToOriginImpl() 
{
  _estimate << Matrix<double, JOINT_DOF, 1>::Zero();
}


/**
 * The joint limits are used in oplusImpl() to clamp joint values within bounds.
 */
void DualArmDualHandVertex::set_bounds(const std::vector<double> _q_l_arm_lb, const std::vector<double> _q_l_arm_ub, 
                    const std::vector<double> _q_r_arm_lb, const std::vector<double> _q_r_arm_ub, 
                    const std::vector<double> _q_l_finger_lb, const std::vector<double> _q_l_finger_ub, 
                    const std::vector<double> _q_r_finger_lb, const std::vector<double> _q_r_finger_ub)
{      
  // Lower bounds
  q_lb = _q_l_arm_lb;
  q_lb.insert(q_lb.end(), _q_r_arm_lb.cbegin(), _q_r_arm_lb.cend());
  q_lb.insert(q_lb.end(), _q_l_finger_lb.cbegin(), _q_l_finger_lb.cend());
  q_lb.insert(q_lb.end(), _q_r_finger_lb.cbegin(), _q_r_finger_lb.cend());

  // Upper bounds
  q_ub = _q_l_arm_ub;
  q_ub.insert(q_ub.end(), _q_r_arm_ub.cbegin(), _q_r_arm_ub.cend());
  q_ub.insert(q_ub.end(), _q_l_finger_ub.cbegin(), _q_l_finger_ub.cend());
  q_ub.insert(q_ub.end(), _q_r_finger_ub.cbegin(), _q_r_finger_ub.cend());
}


/**
 * Do position limit check and collision check before accepting the updates. Choose among the feasible ones the one closest to the updated values.
 */
void DualArmDualHandVertex::oplusImpl(const double *update) 
{
  // Check if bounds are specified
  if (q_lb.empty() || q_ub.empty())
  {
    std::cerr << "Error: Lower or Upper bounds of joint angles are not set. Did you forget to pass in bounds info?" << std::endl;
    exit(-1);
  }

  // For collision checking
  int num_intervals = 100; // number of intervals from current estimate to new estimate 

  // Set updates and clamp by bounds
  Matrix<double, JOINT_DOF, 1> cur_estimate = _estimate;
  Matrix<double, JOINT_DOF, 1> new_estimate;
  amount_out_of_bound = 0.0; // record amount of out-of-bound parts(which are clamped below)
  double ori;
  for (unsigned int i = 0; i < JOINT_DOF; i++)
  {
    // asssign updates
    new_estimate[i] = cur_estimate[i] + update[i];

    // resolve position limits
    new_estimate[i] = std::max(std::min(new_estimate[i], q_ub[i]), q_lb[i]);
    
    // record for debug
    amount_out_of_bound += std::fabs(new_estimate[i] - cur_estimate[i]);

    // store for debug
    last_update[i] = update[i]; // record updates
  }
  // std::cout << "Current updates on q = " << last_update.transpose() << std::endl << std::endl;
  // std::cout << "Amount of out-of-bounds = " << amount_out_of_bound << std::endl;

  // Collision avoidance
  /*
  Matrix<double, JOINT_DOF, 1> tmp_estimate;
  std::vector<double> tmp_estimate_vec(JOINT_DOF);
  for(int n = 0; n < num_intervals; n++)
  {
    // Do collision checking from the end to the beginning
    tmp_estimate = new_estimate - (double)n * (new_estimate - cur_estimate) / (double)num_intervals; 
    
    // convert to std::vector
    for (unsigned int j = 0; j < JOINT_DOF; j++)
      tmp_estimate_vec[j] = tmp_estimate[j];
    
    double result = dual_arm_dual_hand_collision_ptr->check_self_collision(tmp_estimate_vec);
    // std::cout << "debug: check interpolated point " << (n+1) << "/" << num_intervals << " : " << (result < 0.0 ? "collision-free" : "in-collision") << std::endl;

    // decide if it's ok
    if (result < 0.0) // if collision free, use the current one (the first visited collision free state)
      break;
    else // if all in collision, just reject the updates and use the current state still
      tmp_estimate = cur_estimate; 
  }
  */

  // Assign processed estimates to _estimate for g2o
  _estimate = new_estimate; //tmp_estimate;
  joint_interations += 1;
}


/**
 * Set collision checker for use in update function.\n
 * Note that here we cannot initialize reference member in places other than initialization list (used in constructor)
 * and thus this collision checker is not the same as those used in other edges, since it's passed by value instead of by reference.
 */
void DualArmDualHandVertex::set_collision_checker(boost::shared_ptr<DualArmDualHandCollision> _dual_arm_dual_hand_collision_ptr)
{
  dual_arm_dual_hand_collision_ptr = _dual_arm_dual_hand_collision_ptr;
}


#endif