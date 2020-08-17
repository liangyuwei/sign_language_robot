#ifndef DMP_CONSTRAINTS_H
#define DMP_CONSTRAINTS_H

// Common
#include <iostream>

// G2O: infrastructure
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/QR>

// Related vertex
#include "vertices/DMPStartsGoalsVertex.h"

// Helper variables
#include "config.h"

using namespace g2o;
using namespace Eigen;


/**
 * @brief Constraints edge for constraining the change of DMP starts and goals.
 * 
 * There are three constraints directly related to DMP starts and goals: \n
 * (1) orientation of vector connecting start and goal; \n
 * (2) scale of the magnitude of vector connecting start and goal; \n
 * (3) magnitude of changes in vectors connecting relative DMPs' starts and goals.
 */
class DMPConstraints : public BaseUnaryEdge<1, my_constraint_struct, DMPStartsGoalsVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Constructor with initialization of DMP starts and goals
    DMPConstraints(Matrix<double, 1, 3> lrw_goal, Matrix<double, 1, 3> lrw_start, 
                   Matrix<double, 1, 3> lew_goal, Matrix<double, 1, 3> lew_start, 
                   Matrix<double, 1, 3> rew_goal, Matrix<double, 1, 3> rew_start,
                   Matrix<double, 1, 3> rw_goal, Matrix<double, 1, 3> rw_start);

    // g2o internal
    void computeError();    ///< Used by g2o internal calculation, compute cost value.

    // Compute jacobians
    virtual void linearizeOplus();    ///< Used by g2o internal calculation, compute jacobians.
    
    // Debug output  
    double output_cost(unsigned int FLAG);          
    Matrix<double, 1, DMPPOINTS_DOF> output_jacobian(unsigned int FLAG); 
    
    // Flags for specifying which constraints to choose for debug output
    unsigned int ORIEN_FLAG = 0;        ///< Flag for debug print for orientation constraint.
    unsigned int SCALE_FLAG = 1;        ///< Flag for debug print for scale constraint.
    unsigned int REL_CHANGE_FLAG = 2;   ///< Flag for debug print for relative change constraint.

    /// Read from disk, leave blank
    virtual bool read( std::istream& in ) {return true;}
    
    /// Write to disk, leave blank
    virtual bool write( std::ostream& out ) const {return true;}

  private:
    // Information about the original trajectories
    Matrix<double, 1, 3> lrw_goal;      ///< Original Goal position of lrw DMP.
    Matrix<double, 1, 3> lrw_start;     ///< Original Start position of lrw DMP.
    Matrix<double, 1, 3> lew_goal;      ///< Original Goal position of lew DMP.
    Matrix<double, 1, 3> lew_start;     ///< Original Start position of lew DMP.
    Matrix<double, 1, 3> rew_goal;      ///< Original Goal position of rew DMP.
    Matrix<double, 1, 3> rew_start;     ///< Original Start position of rew DMP.
    Matrix<double, 1, 3> rw_goal;       ///< Original Goal position of rw DMP.
    Matrix<double, 1, 3> rw_start;      ///< Original Start position of lrw DMP.

    // Vectors pointing from starts to goals
    Matrix<double, 1, 3> lrw_vec;       ///< Original vector connecting lrw DMP's start and goal, with the size of 1 x 3.
    Matrix<double, 1, 3> lew_vec;       ///< Original vector connecting lew DMP's start and goal, with the size of 1 x 3.
    Matrix<double, 1, 3> rew_vec;       ///< Original vector connecting rew DMP's start and goal, with the size of 1 x 3.
    Matrix<double, 1, 3> rw_vec;        ///< Original vector connecting rw DMP's start and goal, with the size of 1 x 3.

    // Lengths of vectors connecting starts and goals
    double lrw_len;     ///< Original length of vector connecting lrw DMP's start and goal.
    double lew_len;     ///< Original length of vector connecting lew DMP's start and goal.
    double rew_len;     ///< Original length of vector connecting rew DMP's start and goal.
    double rw_len;      ///< Original length of vector connecting rw DMP's start and goal.

    /// Threshold value for orientation cost
    double max_theta = 5.0 * M_PI / 180.0;  ///< In radius.

    // Store jacobians for debug
    Matrix<double, 1, DMPPOINTS_DOF> orien_jacobians_for_dmp;   ///< The jacobian of vector orientation cost w.r.t DMP starts and goals.
    Matrix<double, 1, DMPPOINTS_DOF> scale_jacobians_for_dmp;   ///< The jacobian of scale cost w.r.t DMP starts and goals.
    Matrix<double, 1, DMPPOINTS_DOF> rel_jacobians_for_dmp;     ///< The jacobian of rel change cost w.r.t DMP starts and goals.

    // Helper functions to compute costs
    double compute_orien_cost(Matrix<double, DMPPOINTS_DOF, 1> x);        ///< Compute orientation constraint value.
    double compute_scale_cost(Matrix<double, DMPPOINTS_DOF, 1> x);        ///< Compute scale constraint value.
    double compute_rel_change_cost(Matrix<double, DMPPOINTS_DOF, 1> x);   ///< Compute relative change constraint value.

};


/**
 * Feed in original starts and goals positions (from human demonstration) at initialization, for use in evaluating constraints values.
 */
DMPConstraints::DMPConstraints(Matrix<double, 1, 3> lrw_goal, Matrix<double, 1, 3> lrw_start, 
                               Matrix<double, 1, 3> lew_goal, Matrix<double, 1, 3> lew_start, 
                               Matrix<double, 1, 3> rew_goal, Matrix<double, 1, 3> rew_start,
                               Matrix<double, 1, 3> rw_goal, Matrix<double, 1, 3> rw_start)
{
  // initialization
  this->lrw_goal = lrw_goal;
  this->lrw_start = lrw_start;
  this->lew_goal = lew_goal;
  this->lew_start = lew_start;
  this->rew_goal = rew_goal;
  this->rew_start = rew_start;
  this->rw_goal = rw_goal;
  this->rw_start = rw_start;          

  // Get vectors pointing from starts to goals
  this->lrw_vec = lrw_goal - lrw_start;
  this->lew_vec = lew_goal - lew_start;
  this->rew_vec = rew_goal - rew_start;
  this->rw_vec = rw_goal - rw_start;

  // Store the lengths
  this->lrw_len = this->lrw_vec.norm();
  this->lew_len = this->lew_vec.norm();
  this->rew_len = this->rew_vec.norm();
  this->rw_len = this->rw_vec.norm();
}


/**
 * computeError() function used by g2o.
 */
void DMPConstraints::computeError()
{
  // Get new DMP starts and goals
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();

  // set error
  _error(0, 0) = K_DMPSTARTSGOALS * this->compute_orien_cost(x) + 
                 K_DMPSCALEMARGIN * this->compute_scale_cost(x) +
                 K_DMPRELCHANGE * this->compute_rel_change_cost(x);
}


/**
 * @brief Output cost for specified constraint under the current state. 
 * 
 * Flag for orientation / scale / relative change cost: ORIEN_FLAG / SCALE_FLAG / REL_CHANGE_FLAG.
 */
double DMPConstraints::output_cost(unsigned int FLAG)
{
  // Get new DMP starts and goals
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the only vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();

  // Decide which to return
  if (FLAG == ORIEN_FLAG)
    return this->compute_orien_cost(x);

  if (FLAG == SCALE_FLAG)
    return this->compute_scale_cost(x);

  if (FLAG == REL_CHANGE_FLAG)
    return this->compute_rel_change_cost(x);
    
  // If no match
  std::cerr << "Flag is not defined!!! Check again!!!" << std::endl;
  exit(-1);
}


/**
 * @brief Output jacobian for the specified constraint from last call to linearizeOplus(). 
 * 
 * Flag for orientation / scale / relative change cost: ORIEN_FLAG / SCALE_FLAG / REL_CHANGE_FLAG.
 */
Matrix<double, 1, DMPPOINTS_DOF> DMPConstraints::output_jacobian(unsigned int FLAG)
{
  // Decide which to output
  if (FLAG == ORIEN_FLAG)
    return this->orien_jacobians_for_dmp;
  
  if (FLAG == SCALE_FLAG)
    return this->scale_jacobians_for_dmp;

  if (FLAG == REL_CHANGE_FLAG)
    return this->rel_jacobians_for_dmp;

  // If no match
  std::cerr << "Flag is not defined, check again!!!" << std::endl;
} 


/**
 * @brief Compute orientation cost.
 * 
 * Function for internal use.
 */
double DMPConstraints::compute_orien_cost(Matrix<double, DMPPOINTS_DOF, 1> x)
{
  // get goals and starts
  Matrix<double, 3, 1> lrw_new_goal; lrw_new_goal = x.block(0, 0, 3, 1);
  Matrix<double, 3, 1> lrw_new_start; lrw_new_start = x.block(3, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_goal; lew_new_goal = x.block(6, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_start; lew_new_start = x.block(9, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_goal; rew_new_goal = x.block(12, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_start; rew_new_start = x.block(15, 0, 3, 1);
  Matrix<double, 3, 1> rw_new_goal; rw_new_goal = x.block(18, 0, 3, 1);
  Matrix<double, 3, 1> rw_new_start; rw_new_start = x.block(21, 0, 3, 1);

  // Get new vectors
  Matrix<double, 3, 1> lrw_new_vec; lrw_new_vec = lrw_new_goal - lrw_new_start;
  Matrix<double, 3, 1> lew_new_vec; lew_new_vec = lew_new_goal - lew_new_start;
  Matrix<double, 3, 1> rew_new_vec; rew_new_vec = rew_new_goal - rew_new_start;
  Matrix<double, 3, 1> rw_new_vec; rw_new_vec = rw_new_goal - rw_new_start;

  // Get angle from acos (in radius, absolute value)
  double cos_lrw_theta = std::min((double)(this->lrw_vec * lrw_new_vec) / (this->lrw_vec.norm() * lrw_new_vec.norm()), 1.0);
  double cos_lew_theta = std::min((double)(this->lew_vec * lew_new_vec) / (this->lew_vec.norm() * lew_new_vec.norm()), 1.0);
  double cos_rew_theta = std::min((double)(this->rew_vec * rew_new_vec) / (this->rew_vec.norm() * rew_new_vec.norm()), 1.0);
  double cos_rw_theta = std::min((double)(this->rw_vec * rw_new_vec) / (this->rw_vec.norm() * rw_new_vec.norm()), 1.0);
  double lrw_theta = std::fabs(std::acos(cos_lrw_theta));
  double lew_theta = std::fabs(std::acos(cos_lew_theta));
  double rew_theta = std::fabs(std::acos(cos_rew_theta));
  double rw_theta = std::fabs(std::acos(cos_rw_theta));

  // get orientation error
  // pow makes the values smaller... 
  double orien_cost = std::max(lrw_theta - max_theta, 0.0) +
                      std::max(lew_theta - max_theta, 0.0) +
                      std::max(rew_theta - max_theta, 0.0) +
                      std::max(rw_theta - max_theta, 0.0); // l1 penalty
  
                      /* // l2 penalty
                      std::pow(std::max(lrw_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(lew_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(rew_theta - max_theta, 0.0), 2) +
                      std::pow(std::max(rw_theta - max_theta, 0.0), 2) ; // use pow() so that jacobian increases as the cost rises up, instead of staying the same...     
                      */

                      /*std::pow( std::max(lrw_theta - max_theta, 0.0) +
                                 std::max(lew_theta - max_theta, 0.0) +
                                 std::max(rew_theta - max_theta, 0.0) +
                                 std::max(rw_theta - max_theta, 0.0), 2);*/

  return orien_cost;
}


/**
 * @brief Compute scale cost.
 * 
 * Function for internal use. Constrain the change in scale of vectors connecting DMP starts and goals, 
 * in case the optimization algorithm scales the DMP trajectories down so as to track more easily.(lazy)
 */
double DMPConstraints::compute_scale_cost(Matrix<double, DMPPOINTS_DOF, 1> x)
{
  // get goals and starts
  Matrix<double, 3, 1> lrw_new_goal; lrw_new_goal = x.block(0, 0, 3, 1);
  Matrix<double, 3, 1> lrw_new_start; lrw_new_start = x.block(3, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_goal; lew_new_goal = x.block(6, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_start; lew_new_start = x.block(9, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_goal; rew_new_goal = x.block(12, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_start; rew_new_start = x.block(15, 0, 3, 1);
  Matrix<double, 3, 1> rw_new_goal; rw_new_goal = x.block(18, 0, 3, 1);
  Matrix<double, 3, 1> rw_new_start; rw_new_start = x.block(21, 0, 3, 1);

  // Get new vectors
  Matrix<double, 3, 1> lrw_new_vec; lrw_new_vec = lrw_new_goal - lrw_new_start;
  Matrix<double, 3, 1> lew_new_vec; lew_new_vec = lew_new_goal - lew_new_start;
  Matrix<double, 3, 1> rew_new_vec; rew_new_vec = rew_new_goal - rew_new_start;
  Matrix<double, 3, 1> rw_new_vec; rw_new_vec = rw_new_goal - rw_new_start;

  // Get the lengths of new vectors and ratios
  double lrw_new_len = lrw_new_vec.norm();
  double lew_new_len = lew_new_vec.norm();
  double rew_new_len = rew_new_vec.norm();
  double rw_new_len = rw_new_vec.norm();
  double lrw_ratio = lrw_new_len / this->lrw_len;
  double lew_ratio = lew_new_len / this->lew_len;
  double rew_ratio = rew_new_len / this->rew_len;
  double rw_ratio = rw_new_len / this->rw_len;
  Vector4d ratios;
  ratios << lrw_ratio, lew_ratio, rew_ratio, rw_ratio;
  double margin = ratios.maxCoeff() - ratios.minCoeff();

  // set error
  // setting margin for max and min ratios could yield cheating behavior, i.e. the trajectories become very tiny for the manipulators to track under small cost
  double max_margin = 0.05; // within 5%
  //double scale_cost = std::pow(std::max(margin-max_margin, 0.0), 2); // add pow() to allow jacobian increase as the cost rises, instead of staying the same gradient
  // compromise: set 1.0+-ratio_bound as the region of expected scale
  double ratio_bound = 0.05; // within +-5%
  double scale_cost = std::max(std::fabs(lrw_ratio - 1.0) - ratio_bound, 0.0) +
                      std::max(std::fabs(lew_ratio - 1.0) - ratio_bound, 0.0) + 
                      std::max(std::fabs(rew_ratio - 1.0) - ratio_bound, 0.0) +
                      std::max(std::fabs(rw_ratio - 1.0) - ratio_bound, 0.0) +
                      std::max(margin-max_margin, 0.0); // l1 penalty

                      /*
                      std::pow(std::max(std::fabs(lrw_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(std::fabs(lew_ratio - 1.0) - ratio_bound, 0.0), 2) + 
                      std::pow(std::max(std::fabs(rew_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(std::fabs(rw_ratio - 1.0) - ratio_bound, 0.0), 2) +
                      std::pow(std::max(margin-max_margin, 0.0), 2); // l2 penalty
                      */

  return scale_cost;

}


/**
 *  @brief Compute relative change cost.
 * 
 * Function for internal use. Constrain the shift of starts and goals of relative-traj DMPs, 
 * do not let it go too far from the human demonstrated relative motion characteristics.
 */
double DMPConstraints::compute_rel_change_cost(Matrix<double, DMPPOINTS_DOF, 1> x)
{
  // get goals and starts
  Matrix<double, 3, 1> lrw_new_goal; lrw_new_goal = x.block(0, 0, 3, 1);
  Matrix<double, 3, 1> lrw_new_start; lrw_new_start = x.block(3, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_goal; lew_new_goal = x.block(6, 0, 3, 1);
  Matrix<double, 3, 1> lew_new_start; lew_new_start = x.block(9, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_goal; rew_new_goal = x.block(12, 0, 3, 1);
  Matrix<double, 3, 1> rew_new_start; rew_new_start = x.block(15, 0, 3, 1);

  // Get the lengths of new vectors and ratios
  double lrw_start_change = (lrw_new_start.transpose() - this->lrw_start).norm();
  double lrw_goal_change = (lrw_new_goal.transpose() - this->lrw_goal).norm();
  double lew_start_change = (lew_new_start.transpose() - this->lew_start).norm();
  double lew_goal_change = (lew_new_goal.transpose() - this->lew_goal).norm();
  double rew_start_change = (rew_new_start.transpose() - this->rew_start).norm();
  double rew_goal_change = (rew_new_goal.transpose() - this->rew_goal).norm();

  // set error
  double lrw_margin = 0.0;//0.02;//0.01; // within 1 cm (a bad result displays 0.05 offset, calculated in MATLAB) // relax a few
  double ew_margin = 0.05; //0.02; // relative change between elbow and wrist, allow for robot with different configuration to track
  double rel_change_cost = std::max(lrw_start_change - lrw_margin, 0.0) +
                           std::max(lrw_goal_change - lrw_margin, 0.0) +
                           std::max(lew_start_change - ew_margin, 0.0) +
                           std::max(lew_goal_change - ew_margin, 0.0) +
                           std::max(rew_start_change - ew_margin, 0.0) +
                           std::max(rew_goal_change - ew_margin, 0.0); // l1 penalty

  return rel_change_cost;
}


/**
 * Compute jacobians for g2o internal call. And also store them for debug.
 */
void DMPConstraints::linearizeOplus()
{
  // Prep
  double dmp_eps = 0.01;//0.005;//0.02;

  // Get new DMP starts and goals
  const DMPStartsGoalsVertex *v = dynamic_cast<const DMPStartsGoalsVertex*>(_vertices[0]); // the last vertex connected
  Matrix<double, DMPPOINTS_DOF, 1> x = v->estimate();
  Matrix<double, DMPPOINTS_DOF, 1> delta_x = Matrix<double, DMPPOINTS_DOF, 1>::Zero();
  double e_plus, e_minus;
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
  {
    // set delta_x
    delta_x[d] = dmp_eps;

    // evaluate jacobians for orientation cost
    e_plus = this->compute_orien_cost(x+delta_x);
    e_minus = this->compute_orien_cost(x-delta_x);
    _jacobianOplusXi(0, d) = K_DMPSTARTSGOALS * (e_plus - e_minus) / (2*dmp_eps);
    this->orien_jacobians_for_dmp(0, d) = K_DMPSTARTSGOALS * (e_plus - e_minus) / (2*dmp_eps);
    // if (this->orien_jacobians_for_dmp(0, d) > 1 || this->orien_jacobians_for_dmp(0, d) < -1)
    // {
    //   std::cout << "gradient value is " << this->orien_jacobians_for_dmp(0, d) << "." << std::endl;
    //   std::cout << "breakpoint for debug" << std::endl;
    // }

    // evaluate jacobians for scale cost
    e_plus = this->compute_scale_cost(x+delta_x);
    e_minus = this->compute_scale_cost(x-delta_x);
    _jacobianOplusXi(0, d) += K_DMPSCALEMARGIN * (e_plus - e_minus) / (2*dmp_eps); // add up the influence
    this->scale_jacobians_for_dmp(0, d) = K_DMPSCALEMARGIN * (e_plus - e_minus) / (2*dmp_eps);
    // if (this->scale_jacobians_for_dmp(0, d) > 1 || this->scale_jacobians_for_dmp(0, d) < -1)
    // {
    //   std::cout << "gradient value is " << this->scale_jacobians_for_dmp(0, d) << "." << std::endl;
    //   std::cout << "breakpoint for debug" << std::endl;
    // }

    // evaluate jacobians for rel change cost
    e_plus = this->compute_rel_change_cost(x+delta_x);
    e_minus = this->compute_rel_change_cost(x-delta_x);
    _jacobianOplusXi(0, d) += K_DMPRELCHANGE * (e_plus - e_minus) / (2*dmp_eps); // add up the influence
    this->rel_jacobians_for_dmp(0, d) = K_DMPRELCHANGE * (e_plus - e_minus) / (2*dmp_eps);

    // reset delta_x
    delta_x[d] = 0.0;
  }
}


#endif