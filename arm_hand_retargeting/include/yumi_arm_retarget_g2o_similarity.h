// ROS
#include <ros/ros.h>

// Common
#include <vector>
#include <math.h>
#include <iostream>
#include <chrono>
#include <string>
#include <cfloat>

// For acos, fabs, pow
#include <cmath>

// For max
#include <algorithm>

// G2O: infrastructure
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/block_solver.h>

// G2O: optimization algorithms
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

// G2O: linear equation solver
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

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

// For TRAC-IK
#include <trac_ik/trac_ik.hpp>

// For file write and read
#include "tools/h5_io.h"

// Process the terminal arguments
#include <getopt.h>

// For collision checking
#include "tools/collision_checking_yumi.h"

// For DMP trajectory generator
#include "tools/generate_trajectory_using_DMP.h" 

// For vertices
#include "vertices/DMPStartsGoalsVertex.h"
#include "vertices/DualArmDualHandVertex.h"

// For edges
#include "edges/DMPConstraints.h"
#include "edges/MyUnaryConstraints.h"
#include "edges/SmoothnessConstraint.h"
#include "edges/TrackingConstraint.h"

// Some global and static variables defined for convenience.
#include "config.h"

using namespace g2o;
using namespace Eigen;
using namespace H5;


// Program arguments, free of re-compilation
std::string in_file_name = "test_imi_data_YuMi.h5";           
std::string in_group_name = "fengren_1";
std::string out_file_name = "mocap_ik_results_YuMi_g2o.h5";


/*!
 * This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver.
 * @param[in,out] constraint_data Store the wrist ID and elbow ID in the self-defined struct data type.
 * @param[in]     left_or_right   Specify left or right arm to derive KDL FK solver for.
 */
KDL::ChainFkSolverPos_recursive setup_kdl(my_constraint_struct &constraint_data, bool left_or_right)
{
  // Params
  const std::string ELBOW_LINK = (left_or_right ? LEFT_ELBOW_LINK : RIGHT_ELBOW_LINK);
  const std::string WRIST_LINK = (left_or_right ? LEFT_WRIST_LINK : RIGHT_WRIST_LINK);

  // Get tree
  KDL::Tree kdl_tree; 
  if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      exit(-1);
  }

  // Get chain  
  KDL::Chain kdl_chain; 
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    exit(-1);
  }

  // Find segment number for wrist and elbow links, store in constraint_dat
  unsigned int num_wrist_seg = (left_or_right ? constraint_data.l_num_wrist_seg : constraint_data.r_num_wrist_seg);
  unsigned int num_elbow_seg = (left_or_right ? constraint_data.l_num_elbow_seg : constraint_data.r_num_elbow_seg);
  if (num_wrist_seg == 0 || num_elbow_seg == 0) // if the IDs not set
  {
    unsigned int num_segments = kdl_chain.getNrOfSegments();
    num_wrist_seg = num_segments - 1;
    //ROS_INFO_STREAM("There are " << num_segments << " segments in the kdl_chain");
    for (unsigned int i = 0; i < num_segments; ++i){
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        num_elbow_seg = i;
        break;
      }
    }
  }

  if (left_or_right)
  {
    constraint_data.l_num_wrist_seg = num_wrist_seg;
    constraint_data.l_num_elbow_seg = num_elbow_seg;
  }
  else
  {
    constraint_data.r_num_wrist_seg = num_wrist_seg;
    constraint_data.r_num_elbow_seg = num_elbow_seg;    
  }

  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  //ROS_INFO_STREAM("Joint dimension is: " << kdl_chain.getNrOfJoints()); // 6 joints, 8 segments, checked!

  return fk_solver;

}








