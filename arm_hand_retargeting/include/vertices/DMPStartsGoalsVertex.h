#ifndef DMP_STARTS_GOALS_VERTEX_H_
#define DMP_STARTS_GOALS_VERTEX_H_

// Common
#include <iostream>

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

// Helper variables
#include "config.h"

using namespace g2o;
using namespace Eigen;

/**
 * @brief Define vertex for starts and goals of DMPs that encode position trajectories. 
 *
 * Order is: lrw(left and right wrist), lew(left elbow and wrist), rew(right elbow and wrist), rw(right wrist), with each goal pos before each start pos.
 */
class DMPStartsGoalsVertex : public BaseVertex<DMPPOINTS_DOF, Matrix<double, DMPPOINTS_DOF, 1> >
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Reset vertex state
    virtual void setToOriginImpl();
  
    /// Update vertex state
    virtual void oplusImpl(const double *update);

    /// The update values used during last call of oplusImpl()
    Matrix<double, DMPPOINTS_DOF, 1> last_update;

    /// Read from disk, leave blank
    virtual bool read( std::istream& in ) {return true;}

    /// Write to disk, leave blank
    virtual bool write( std::ostream& out ) const {return true;}
};


/**
 * Reset vertex state to zeros.
 */
void DMPStartsGoalsVertex::setToOriginImpl() 
{
  _estimate << Matrix<double, DMPPOINTS_DOF, 1>::Zero();
}


/**
 * Apply updates to vertex state, and record the update values for debug.
 */
void DMPStartsGoalsVertex::oplusImpl(const double *update) 
{
  //std::cout << "debug: update DMP starts and goals " << std::endl;
  num_update++;
  for (unsigned int i = 0; i < DMPPOINTS_DOF; ++i)
  {
    _estimate[i] += update[i];
    last_update(i, 0) = update[i]; // record updates
  }
  dmp_iteractions += 1;
  //std::cout << "debug: current update = " << last_update.transpose() << std::endl;
}



#endif