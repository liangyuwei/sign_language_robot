#ifndef SMOOTHNESS_CONSTRAINT_H
#define SMOOTHNESS_CONSTRAINT_H

// Common
#include <iostream>

// For max
#include <algorithm>

// G2O: infrastructure
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>

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
 * Constraint edge for smoothness cost between adjacent path points 
 */
class SmoothnessConstraint : public BaseBinaryEdge<1, double, DualArmDualHandVertex, DualArmDualHandVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /// Used by g2o internal call
    void computeError();

    /// Function for recording cost hsitory
    double return_smoothness_cost();

    /// Used by g2o internal call
    virtual void linearizeOplus();

    // output jacobians for debug
    Matrix<double, 2, JOINT_DOF> q_jacobian;
    Matrix<double, 2, JOINT_DOF> output_q_jacobian()
    {
      return q_jacobian;
    }

    // Read and write, leave blank
    virtual bool read( std::istream& in ) {return true;}
    virtual bool write( std::ostream& out ) const {return true;}

  private:
    double smoothness_scale = 0.01; //1.0; //0.5; //0.1; //0.5; //1.0;//0.1;// 0.01;//0.05; //0.1; //0.5; //1.0;
    
    // set up bounds for each DOF
    double margin_of_smoothness_arm = 5.0 * M_PI / 180.0;
    double margin_of_smoothness_finger = 5.0 * M_PI / 180.0;

    // bounds for the whole joint angle block
    double margin_of_smoothness = std::sqrt(std::pow(3.0 * M_PI / 180.0, 2) * JOINT_DOF); //0.0;//std::sqrt(std::pow(1.0 * M_PI / 180.0, 2) * JOINT_DOF); // 1 degree offset for each dim

};


/**
 * Compute smoothness cost.
 */
void SmoothnessConstraint::computeError()
{
  // statistics
  count_smoothness++;  
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

  // Get the values of the two vertices
  const DualArmDualHandVertex *v0 = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
  const DualArmDualHandVertex *v1 = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
  const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

  // Skip if q vertices are fixed
  if(v0->fixed() && v1->fixed())
  {
    _error(0, 0) = 0.0;
    return;
  }

  // Compute smoothness cost
  // _error(0, 0) = smoothness_scale * max( (x0 - x1).norm() - margin_of_smoothness, 0.0) / margin_of_smoothness;
  const Matrix<double, JOINT_DOF, 1> dx = (x1-x0).cwiseAbs();
  _error(0, 0) = 0.0;
  for (unsigned d = 0; d < 14; d++)
  {
    _error(0, 0) += pow(max(dx[d] - margin_of_smoothness_arm, 0.0), 2);
  }
  for (unsigned d = 14; d < JOINT_DOF; d++)
  {
    _error(0, 0) += pow(max(dx[d] - margin_of_smoothness_finger, 0.0), 2);
  }  
  _error(0, 0) = smoothness_scale * _error(0, 0);
  // _error(0, 0) = sqrt(_error(0, 0));

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_smoothness += t_spent.count();
}


/**
 * Use l1 penalty for smoothness cost. No coefficient or scale applied to get the actual state.
 */
double SmoothnessConstraint::return_smoothness_cost()
{
  // Get the values of the two vertices
  const DualArmDualHandVertex *v0 = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
  const DualArmDualHandVertex *v1 = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
  const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

  return (x0 - x1).norm(); //std::max( (x0 - x1).norm() - margin_of_smoothness, 0.0);
}


/**
 * Compute jacobians for each arm and hand part, so as to apply different eps for arm and hand parts.
 */
void SmoothnessConstraint::linearizeOplus()
{
  // setup
  double q_arm_eps = 2.0 * M_PI / 180.0;
  double q_finger_eps = 2.0 * M_PI / 180.0;

  // Get vertex values
  DualArmDualHandVertex *v0 = static_cast<DualArmDualHandVertex*>(_vertices[0]);
  DualArmDualHandVertex *v1 = static_cast<DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
  const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

  // prep
  Matrix<double, JOINT_DOF, 1> delta_x = Matrix<double, JOINT_DOF, 1>::Zero();
  double e_plus, e_minus;


  // For arms 
  for (unsigned int d = 0; d < 7; d++)
  {
    // --- Left Arm --- //

    // set delta
    delta_x[d] = q_arm_eps;

    // (1) - jacobian for the former vertex
    v0->setEstimate(x0+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v0->setEstimate(x0-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    _jacobianOplusXi(0, d) = (e_plus-e_minus) / (2*q_arm_eps);
    v0->setEstimate(x0); // reset

    // (2) - jacobian for the latter vertex
    v1->setEstimate(x1+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v1->setEstimate(x1-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    _jacobianOplusXj(0, d) = (e_plus-e_minus) / (2*q_arm_eps);
    v1->setEstimate(x1); // reset

    // reset delta
    delta_x[d] = 0.0;


    // --- Right Arm --- //

    // set delta
    delta_x[d+7] = q_arm_eps;

    // (1) - jacobian for the former vertex
    v0->setEstimate(x0+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v0->setEstimate(x0-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    _jacobianOplusXi(0, d+7) = (e_plus-e_minus) / (2*q_arm_eps);
    v0->setEstimate(x0); // reset

    // (2) - jacobian for the latter vertex
    v1->setEstimate(x1+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v1->setEstimate(x1-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    _jacobianOplusXj(0, d+7) = (e_plus-e_minus) / (2*q_arm_eps);
    v1->setEstimate(x1); // reset

    // reset delta
    delta_x[d+7] = 0.0;
  }


  // For fingers
  for (unsigned int d = 0; d < 12; d++)
  {
    // --- Left Hand --- //
    
    // set delta
    delta_x[d+14] = q_finger_eps;

    // (1) - jacobian for the former vertex
    v0->setEstimate(x0+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v0->setEstimate(x0-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    _jacobianOplusXi(0, d+14) = (e_plus-e_minus) / (2*q_finger_eps);
    v0->setEstimate(x0); // reset

    // (2) - jacobian for the latter vertex
    v1->setEstimate(x1+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v1->setEstimate(x1-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    _jacobianOplusXj(0, d+14) = (e_plus-e_minus) / (2*q_finger_eps);
    v1->setEstimate(x1); // reset

    // reset delta
    delta_x[d+14] = 0.0;


    // --- Right Arm --- //

    // set delta
    delta_x[d+26] = q_finger_eps;

    // (1) - jacobian for the former vertex
    v0->setEstimate(x0+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v0->setEstimate(x0-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    _jacobianOplusXi(0, d+26) = (e_plus-e_minus) / (2*q_finger_eps);
    v0->setEstimate(x0); // reset

    // (2) - jacobian for the latter vertex
    v1->setEstimate(x1+delta_x);
    this->computeError();
    e_plus = _error(0, 0);
    v1->setEstimate(x1-delta_x);
    this->computeError();
    e_minus = _error(0, 0);
    _jacobianOplusXj(0, d+26) = (e_plus-e_minus) / (2*q_finger_eps);
    v1->setEstimate(x1); // reset

    // reset delta
    delta_x[d+26] = 0.0;
  }

  // store jacobians for debug
  for (unsigned int j = 0; j < JOINT_DOF; j++)
  {
    q_jacobian(0, j) = _jacobianOplusXi(0, j);
    q_jacobian(1, j) = _jacobianOplusXj(0, j);
  }
}

#endif
