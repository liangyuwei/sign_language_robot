#ifndef COLLISION_CONSTRAINT_H
#define COLLISION_CONSTRAINT_H

// Common
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <cmath> // for isnan

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

using namespace std;
using namespace g2o;
using namespace Eigen;


/**
 * @brief Collision edge for dense collision checking and avoidance. (approximate CCD)
 * 
 * We model collision cost as 6-dim error vector, each 3-dim describes the contact normal pointing to the direction of collision. \n
 * Note that the concept of jacobian (gradient) is about the direct in which f rises, and thus we let dx be the direction where collision 
 * gets more severe. \n
 * For reactive collision avoidance, dx = J * dq; for optimization, we solve J * dq = -error (from g2o LS formulation, J^T * J * dq = -b = - J^T * e), 
 * and thus error = -dx.
 */
class CollisionConstraint : public BaseBinaryEdge<6, my_constraint_struct, DualArmDualHandVertex, DualArmDualHandVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor with initialization of a collision checker and the density of collision checking.
     * 
     * This constraint edge performs dense discrete collision checking to ensure a proper interpolation of 
     * the resultant joint trajectories would still be collision free, i.e. to cope with possible collision state
     * during transition from x0 to x1, approximating Continuous-time Collision Checking.
     * @param[in]    num_checks    Number of discrete collision checks between two vertices.
     */
    CollisionConstraint(boost::shared_ptr<DualArmDualHandCollision> &_dual_arm_dual_hand_collision_ptr,
                        unsigned int num_checks) : dual_arm_dual_hand_collision_ptr(_dual_arm_dual_hand_collision_ptr)
    {
      this->num_checks = num_checks;
    }


    /// Compute edge value. Used by g2o internal calculation
    void computeError();

    // functions for recording costs
    double return_col_cost();         ///< Return number of colliding path points.

    // display for debug
    void output_distance_result();    ///< Print collision condition of the current state.

    /// Compute q updates using robot jacobian
    Eigen::MatrixXd compute_col_q_update(Eigen::MatrixXd jacobian, Eigen::Vector3d dx, double speed);

    // Resolve colliding state by directly operating on joint values
    std::vector<Eigen::Matrix<double, JOINT_DOF, 1>> resolve_path_collisions(std::vector<Eigen::Matrix<double, JOINT_DOF, 1>> q_cur, unsigned int num_intervals=100);
    Eigen::Matrix<double, JOINT_DOF, 1> resolve_point_collisions(Eigen::Matrix<double, JOINT_DOF, 1> x0, Eigen::Matrix<double, JOINT_DOF, 1> x1, double col_eps, unsigned int num_intervals); // called in resolve_path_collisions() to resolve colliding state for each path point
    double dense_collision_checking(Eigen::Matrix<double, JOINT_DOF, 1> x0, Eigen::Matrix<double, JOINT_DOF, 1> x1, Eigen::Matrix<double, JOINT_DOF, 1> &x_col, unsigned int num_intervals);

    /// Read from disk, leave blank
    virtual bool read( std::istream& in ) {return true;}
    
    /// Write to disk, leave blank
    virtual bool write( std::ostream& out ) const {return true;}

    /// Re-implement linearizeOplus for jacobians calculation
    virtual void linearizeOplus();

    // variables to store jacobians of each and both constraints
    Matrix<double, 6, JOINT_DOF> col_jacobians = Matrix<double, 6, JOINT_DOF>::Zero();

  private:
    /// Collision checker (with distance computation functionality)
    boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr;

    /// Number of collision checks between two vertices. 
    /// Note that it must be greater than 0, otherwise the dense collision checking would be bypassed.
    unsigned int num_checks;

    /// Compute collision cost with only the hands part
    double compute_dual_hands_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, std::string link_name_1, std::string link_name_2, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    
    /// Compute distance vector, for use in determining error vector of finger collision
    Vector3d compute_dual_hands_collision_error_vector(Matrix<double, JOINT_DOF, 1> q_whole, std::string link_name_1, std::string link_name_2, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr);
    
    // Safety setting related to collision avoidance, note that d_check must be stricly greater than d_safe !!!
    // use different margins of safety for arms and hands
    // here the safety margin should be set according to actual condition: set a collision-free state and check minimum distance for dual_arms and dual_hands using collision_checking_yumi.cpp to have a sense of it
    double d_arm_check = 0.003;  ///< Distance threshold under which to check collision, note that d_check must be strictly greater than d_safe.
    double d_arm_safe = 0.001;   ///< Safety margin for arm part. In the current condition, 0.01 would actually be too large, for some links are very close to each other, e.g. _5_l and _7_l.
    double d_hand_check = 0.002; ///< Distance threshold under which to check collision, note that d_check must be strictly greater than d_safe. 
    double d_hand_safe = 1e-6;   ///< Safety margin for hand part. A small value close to 0 is fine since link51 and link111 are really close to each other under initial collision-free state.

    // Scale factors for different scenerios
    double non_finger_col_scale = 1.0;
    double finger_col_scale = 100.0;

    // Time of contact (stored to reduce number of queries)
    double time_of_contact;

    // Intermediate contact state
    Matrix<double, JOINT_DOF, 1> x_colliding;
};


/**
 * @brief Compute collision cost.
 * 
 * Perform dense collision checking first to see if possible collision state exists. \n
 * If in collision, query the distance and contact information, and use it to set appropriate
 * error vector for the two colliding links. \n
 * Note that for non-same-hand collision situation, we use dx which points to collision(gets severe) 
 * directly as the error vector, for ease of computation of jacobians, while for same-hand collision 
 * situation, we set distance vector (minimum distance projected onto each axis according to the 
 * normalized contact normal vector). \n
 * The _error must be in accordance with _jacobianOplus.
 */
void CollisionConstraint::computeError()
{
  // statistics
  count_unary++;  
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

  // initialization
  _error = Matrix<double, 6, 1>::Zero();

  // get the current joint value
  // _vertices is a VertexContainer type, a std::vector<Vertex*>
  const DualArmDualHandVertex *v0 = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
  const DualArmDualHandVertex *v1 = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
  const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

  // Dense collision checking
  this->time_of_contact = dense_collision_checking(x0, x1, this->x_colliding, this->num_checks);
  bool no_collision = (this->time_of_contact < 0.0); // -1 for no collision

  // Set the error vector (contact normal)
  if (no_collision)
  {
    _error = Matrix<double, 6, 1>::Zero();
  }
  else
  {
    // convert from matrix to std::vector
    std::vector<double> x(JOINT_DOF);
    for (unsigned int i = 0; i < JOINT_DOF; ++i)
      x[i] = this->x_colliding[i];
    
    // Check collision, get contact information
    double min_distance;
    double potential_scale; // the amount beyond the margin of safety
    // 1 - check arms first
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(x, "dual_arms", d_arm_check); 
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    potential_scale = std::pow(std::max(d_arm_safe - min_distance, 0.0), 2) / std::pow(d_arm_safe, 2);
    total_col += t_spent.count();
    count_col++;
    // 2 - check hands if arms are in safety region  
    if (min_distance > d_arm_safe) // arm safe
    {
      t0 = std::chrono::steady_clock::now();
      min_distance = dual_arm_dual_hand_collision_ptr->compute_self_distance_test(x, "dual_hands", d_hand_check); 
      t1 = std::chrono::steady_clock::now();
      t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
      total_col += t_spent.count();
      count_col++;
      potential_scale = std::pow(std::max(d_hand_safe - min_distance, 0.0), 2) / std::pow(d_hand_safe, 2);
    }

    // Set error vector (note that in the direction where cost rises) for colliding links
    int group_id_1 = dual_arm_dual_hand_collision_ptr->check_link_belonging(dual_arm_dual_hand_collision_ptr->link_names[0]);
    int group_id_2 = dual_arm_dual_hand_collision_ptr->check_link_belonging(dual_arm_dual_hand_collision_ptr->link_names[1]);
    std::string link_name_1 = dual_arm_dual_hand_collision_ptr->link_names[0];
    std::string link_name_2 = dual_arm_dual_hand_collision_ptr->link_names[1];

    // if object 1 does not belong to arms or hands, then it's in the environment, thus no need for assigning dx to it
    Vector3d contact_normal = dual_arm_dual_hand_collision_ptr->normal;
    _error.block(0, 0, 3, 1) = (group_id_1 == -1 ? 0.0 : 1.0) * contact_normal; // distance_result.normal points from link_names[0] to link_names[1]
    _error.block(3, 0, 3, 1) = (group_id_2 == -1 ? 0.0 : -1.0) * contact_normal; // distance_result.normal points from link_names[0] to link_names[1]

    // 1 - apply the first weighting factor accounting for margin of safety, i.e. potential field, and the second factor manually set up
    _error = potential_scale * non_finger_col_scale * _error;


    // if same-hand collision, then the error vector should be distance vector
    if ((group_id_1 == 2 && group_id_2 == 2) || (group_id_1 == 3 && group_id_2 == 3) )
    {  
      Vector3d distance_vector = compute_dual_hands_collision_error_vector(this->x_colliding, 
                                                                          dual_arm_dual_hand_collision_ptr->link_names[0],
                                                                          dual_arm_dual_hand_collision_ptr->link_names[1], 
                                                                          dual_arm_dual_hand_collision_ptr);
      _error.block(0, 0, 3, 1) = distance_vector; 
      _error.block(3, 0, 3, 1) = distance_vector;

      // 2 - apply the first weighting factor accounting for safety margin, and the second weighting factor manually set up; for same-hand collision situation
      _error = potential_scale * finger_col_scale * _error;
    }


  }


  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_01 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  total_unary += t_01.count();
}


/**
 * @brief For g2o internal call. \n
 * Calculate jacobians for collision cost. \n
 * 
 * Do dense collision checking on the transition from x0 to x1, and use contact normal + robot jacobian 
 * at the contact point to modify **x1**. \n
 * We adopt gradient normal + robot jacobian method for any arm-arm, arm-hand, hand-hand(different) collision, 
 * and compute numerical derivatives for same-hand finger collision because of the limited DOF for robot fingers which would yeild incorrect dq, 
 * e.g. only 2 DOF for index finger while the given dx is 3-dim data.
 */
void CollisionConstraint::linearizeOplus()
{
  // Initialization
  _jacobianOplusXi = Matrix<double, 6, JOINT_DOF>::Zero();  // since we only modify the latter point, _jacobianOplusXi stays zeros
  _jacobianOplusXj = Matrix<double, 6, JOINT_DOF>::Zero();

  // Get current state
  DualArmDualHandVertex *v0 = dynamic_cast<DualArmDualHandVertex*>(_vertices[0]);
  Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); 
  DualArmDualHandVertex *v1 = dynamic_cast<DualArmDualHandVertex*>(_vertices[1]);
  Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); 

  // Use the result from computeError() directly, reduce number of queries
  double t_contact = this->time_of_contact; 
  if (t_contact < -1e-6) // no collision, return zero jacobians. cope with possible numeric error
    return;
  Matrix<double, JOINT_DOF, 1> x_col = this->x_colliding;

  // epsilons
  double col_eps = 3.0 * M_PI / 180.0; //0.05; // in radius, approximately 3 deg
  double hand_speed = this->finger_col_scale; //100; 

  // Get colliding joint angles
  Matrix<double, JOINT_DOF, 1> x = x_col;
  Matrix<double, JOINT_DOF, 1> delta_x = Matrix<double, JOINT_DOF, 1>::Zero();

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

  // Compute jacobians for different situations
  if ((group_id_1 == 0 && group_id_2 == 0) ||
      (group_id_1 == 0 && group_id_2 == 1) || 
      (group_id_1 == 1 && group_id_2 == 0) || 
      (group_id_1 == 1 && group_id_2 == 1) ) // collision between arm and arm (could be the same), update only q_arm
  {    
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

    // first link (take only the first three rows)
    if (left_or_right_1)
      _jacobianOplusXj.block(0, 0, 3, 7) = jacobian_1.topRows(3); //block(0, 0, 3, 7); // take the first three rows 
    else
      _jacobianOplusXj.block(0, 7, 3, 7) = jacobian_1.topRows(3); //block(0, 0, 3, 7);
    // std::cout << "1 - jacobian = " << _jacobianOplusXj << std::endl;
    // for debug
    bool debug = isnan(_jacobianOplusXj.norm());
    if (debug)
    {
      std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
      double a;
    }

    // second link (take only the first three rows)
    if (left_or_right_2)
      _jacobianOplusXj.block(3, 0, 3, 7) = jacobian_2.topRows(3); //.block(0, 0, 3, 7);
    else
      _jacobianOplusXj.block(3, 7, 3, 7) = jacobian_2.topRows(3); //block(0, 0, 3, 7);
    // std::cout << "2 - jacobian = " << _jacobianOplusXj << std::endl;

    // for debug
    debug = isnan(_jacobianOplusXj.norm());
    if (debug)
    {
      std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
      double a;
    }
  }
  else if ((group_id_1 == 2 && group_id_2 == 2) || (group_id_1 == 3 && group_id_2 == 3) ) // collision between hand and hand (the same one), update only q_finger
  {
    // prep
    bool left_or_right = (group_id_1 == 2);
    int finger_id_1 = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_1, left_or_right);
    int finger_id_2 = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name_2, left_or_right);

    // assign jacobians for collision cost
    // init
    unsigned int d = left_or_right ? 0 : 12;
    Vector3d e_up, e_down;

    // first link 
    switch (finger_id_1)
    {
      case 0: // thumb
        for (unsigned int s = 0; s < 4; s++)
        {
          // set
          delta_x[22+d+s] = col_eps;
          // compute
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(0, 22+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); 
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(0, 14+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); //
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(0, 16+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); //
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(0, 18+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); //
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(0, 20+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); //
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
    // std::cout << "1 - jacobian = " << _jacobianOplusXj << std::endl;
    // std::cout << "1 - jacobian_way_1 = " << jacobian_way_1 << std::endl;
    // std::cout << "1 - jacobian_way_2 = " << jacobian_way_2 << std::endl;

    // for debug
    bool debug = isnan(_jacobianOplusXj.norm());
    if (debug)
    {
      std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(3, 22+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); //
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(3, 14+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); //
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(3, 16+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); //;
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(3, 18+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); // 
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
          e_up = compute_dual_hands_collision_error_vector(x+delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          e_down = compute_dual_hands_collision_error_vector(x-delta_x, link_name_1, link_name_2, dual_arm_dual_hand_collision_ptr);
          _jacobianOplusXj.block(3, 20+d+s, 3, 1) = hand_speed * (e_up - e_down) / (2*col_eps); //
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
    // std::cout << "2 - jacobian = " << _jacobianOplusXj << std::endl;
    // std::cout << "2 - jacobian_way_1 = " << jacobian_way_1 << std::endl;
    // std::cout << "2 - jacobian_way_2 = " << jacobian_way_2 << std::endl;

      // for debug
    debug = isnan(_jacobianOplusXj.norm());
    if (debug)
    {
      std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
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
    int first_or_second = (group_id_1 == -1) ? 3 : 0; // check if the first or second link belongs to the environment
    std::string link_name = (group_id_1 == -1) ? link_name_2 : link_name_1;
    Eigen::Vector3d ref_point_pos = (group_id_1 == -1) ? ref_point_pos_2 : ref_point_pos_1;
    Eigen::MatrixXd jacobian;

    // decide if the link is arm or finger
    if (group_id == 2 || group_id == 3) // hand (should update for arm+hand group)
    {
      // compute robot jacobians
      bool left_or_right = (group_id == 2);
      int finger_id = dual_arm_dual_hand_collision_ptr->check_finger_belonging(link_name, left_or_right);
      if (finger_id != -1) // if not palm group !!!
        jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_hand_jacobian(link_name, ref_point_pos, finger_id, left_or_right);  

      // assign jacobians
      // arm part
      unsigned int d_arm = left_or_right ? 0 : 7;
      _jacobianOplusXj.block(first_or_second, 0+d_arm, 3, 7) = jacobian.block(0, 0, 3, 7); //+= dq_col_update.block(0, 0, 7, 1).transpose();
      // hand part
      unsigned int d_hand = left_or_right ? 0 : 12;
      switch (finger_id)
      {
        case 0: // thummb
          _jacobianOplusXj.block(first_or_second, 22 + d_hand, 3, 4) = jacobian.block(0, 7, 3, 4); 
          break;
        case 1: //index
          _jacobianOplusXj.block(first_or_second, 14 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case 2: //middle
          _jacobianOplusXj.block(first_or_second, 16 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case 3: // ring
          _jacobianOplusXj.block(first_or_second, 18 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case 4: // little
          _jacobianOplusXj.block(first_or_second, 20 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case -1: // palm, do nothing
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }
      // for debug
      bool debug = isnan(_jacobianOplusXj.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
        double a;
      }

    }
    else // arm (should update only for arm group)
    {
      // compute robot jacobians
      bool left_or_right = (group_id == 0);
      jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name, ref_point_pos, left_or_right);
      
      // assign jacobians
      unsigned int d_arm = left_or_right ? 0 : 7;
      _jacobianOplusXj.block(first_or_second, 0+d_arm, 3, 7) = jacobian.topRows(3); 

      // for debug
      bool debug = isnan(_jacobianOplusXj.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
        double a;
      }
    }

  }
  else // all the other conditions, update both q_arm and q_finger ()
  {
    // std::cout << "debug: Possible collision between arm and hand, or left and right hands... update both q_arm and q_finger" << std::endl;

    // prep
    bool left_or_right_1 = ( (group_id_1 == 0) || (group_id_1 == 2) );
    bool left_or_right_2 = ( (group_id_2 == 0) || (group_id_2 == 2) );

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
 
      // assign jacobians
      // arm part
      unsigned int d_arm = left_or_right_1 ? 0 : 7;
      _jacobianOplusXj.block(0, 0+d_arm, 3, 7) = jacobian.block(0, 0, 3, 7); //+= dq_col_update.block(0, 0, 7, 1).transpose();
      // for debug
      bool debug = isnan(_jacobianOplusXj.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
        double a;
      }

      // hand part
      unsigned int d_hand = left_or_right_1 ? 0 : 12;
      switch (finger_id)
      {
        case 0: // thummb
          _jacobianOplusXj.block(0, 22 + d_hand, 3, 4) = jacobian.block(0, 7, 3, 4); 
          break;
        case 1: //index
          _jacobianOplusXj.block(0, 14 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case 2: //middle
          _jacobianOplusXj.block(0, 16 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case 3: // ring
          _jacobianOplusXj.block(0, 18 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case 4: // little
          _jacobianOplusXj.block(0, 20 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case -1: // palm, zero updates would be provided for finger joints, but the size is in consistent with little finger_group since it's used in get_robot_arm_hand_jacobian()
          _jacobianOplusXj.block(0, 20 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }
      // for debug
      debug = isnan(_jacobianOplusXj.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
        double a;
      }

    }
    else // 0 or 1, arm, should calculate robot jacobian for arm group
    {

      // compute robot jacobian for arm group
      Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_1,
                                                                                          ref_point_pos_1,
                                                                                          left_or_right_1);
    
      // assign jacobians
      // arm part
      unsigned int d_arm = left_or_right_1 ? 0 : 7;
      _jacobianOplusXj.block(0, 0+d_arm, 3, 7) = jacobian.topRows(3); 
      // for debug
      bool debug = isnan(_jacobianOplusXj.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
        double a;
      }
    }
    // std::cout << "debug: 1 - jacobian = " << _jacobianOplusXj << std::endl;
    
    
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

      // assign jacobians
      // arm part
      unsigned int d_arm = left_or_right_2 ? 0 : 7;
      _jacobianOplusXj.block(3, 0+d_arm, 3, 7) = jacobian.block(0, 0, 3, 7); 
      // for debug
      bool debug = isnan(_jacobianOplusXj.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
        double a;
      }

      // hand part
      unsigned int d_hand = left_or_right_2 ? 0 : 12;
      switch (finger_id)
      {
        case 0: // thummb
          _jacobianOplusXj.block(3, 22 + d_hand, 3, 4) = jacobian.block(0, 7, 3, 4); 
          break;
        case 1: //index
          _jacobianOplusXj.block(3, 14 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2); 
          break;
        case 2: //middle
          _jacobianOplusXj.block(3, 16 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2);
          break;
        case 3: // ring
          _jacobianOplusXj.block(3, 18 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2);
          break;
        case 4: // little
          _jacobianOplusXj.block(3, 20 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2);
          break;
        case -1: // palm, zero updates would be provided for finger joints, but the size is in consistent with little finger_group since it's used in get_robot_arm_hand_jacobian()
          _jacobianOplusXj.block(3, 20 + d_hand, 3, 2) = jacobian.block(0, 7, 3, 2);
          break;
        default:
          std::cerr << "error: Finger ID doesn't lie in {-1,0,1,2,3,4}!!!" << std::endl;
          exit(-1);
          break;
      }
      // for debug
      debug = isnan(_jacobianOplusXj.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
        double a;
      }

    }
    else // 0 or 1, arm, should calculate robot jacobian for arm group
    {

      // compute robot jacobian for arm group
      Eigen::MatrixXd jacobian = dual_arm_dual_hand_collision_ptr->get_robot_arm_jacobian(link_name_2,
                                                                                          ref_point_pos_2,
                                                                                          left_or_right_2);

      // assign jacobians
      // arm part
      unsigned int d_arm = left_or_right_2 ? 0 : 7;
      _jacobianOplusXj.block(3, 0+d_arm, 3, 7) = jacobian.topRows(3); 
      // for debug
      bool debug = isnan(_jacobianOplusXj.norm());
      if (debug)
      {
        std::cout << "debug: jacobian = " << _jacobianOplusXj << std::endl;
        double a;
      }

    }
    // std::cout << "debug: 2 - jacobian = " << _jacobianOplusXj << std::endl;

  } // END of calculating collision jacobian for optimization


  // record col jacobians
  col_jacobians = _jacobianOplusXj; //.transpose();

}


/** 
 * This function outputs the pair of links that are closest to each other and the corresponding minimum distance,
 *  for checking on collision. 
 */
void CollisionConstraint::output_distance_result()
{
  // std::cout << "Distance result: " << std::endl;
  std::cout << "Collision between " << dual_arm_dual_hand_collision_ptr->link_names[0] << " and " 
                                    << dual_arm_dual_hand_collision_ptr->link_names[1] << ", with min_dist = "
                                    << dual_arm_dual_hand_collision_ptr->min_distance << "." << std::endl;

}


/**
 * @brief This function computes q velocity from Cartesian velocity, through the use of robot jacobian and gradient normal. 
 * 
 * From the theory of robotics, we have [dx, dy, dz, dp, dq, dw] = J * dq, i.e. [d_pos, d_ori] = J * dq. \n
 * Here we computes dq from the given d_pos = [dx, dy, dz] and J, the jacobian matrix.
 * @param[in]   jacobian  The robot jacobian at a particular point, with the size of 6 x N where N is the number of joints related.
 * @param[in]   d_pos     The position part of end-effector Cartesian-space update.
 * @param[in]   speed     The scale ratio to control the update step. With too large speed, jacobian would be inaccurate.
 * @param[out]  d_q       Joint velocity, with the size of N x 1. \n
 * 
 * Note that the update d_q calculated by this function points in the direction where collision cost decreases, 
 * and thus it is opposite to the cost's gradient direction in a sense.(Gradient is the direction in which f rises the fastest.)
 */
Eigen::MatrixXd CollisionConstraint::compute_col_q_update(Eigen::MatrixXd jacobian, Eigen::Vector3d d_pos, double speed)
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
  // d_q = K_COL * d_q;

  return d_q;
}


/**
 * @brief Resolve any colliding state on a given path.
 * 
 * This function takes in the q results of TRAC-IK, and solves collision state via robot jacobians.(based on minimum distance) 
 * @param[in]     q_cur             The joint trajectory that requires reactive collision avoidance processing.
 * Note that the first point mustn't be in colliding state, because this function won't resolve the collision of the first path point!!!
 * @param[in]     num_intervals     Number of intervals of interpolation for dense discrete collision checking between adjacent path points. 
 * Indicates the density of discrete collision checking. Note that a large value would cost more computation.
 */
std::vector<Eigen::Matrix<double, JOINT_DOF, 1>> CollisionConstraint::resolve_path_collisions(std::vector<Eigen::Matrix<double, JOINT_DOF, 1>> q_cur, unsigned int num_intervals)
{
  // Prep
  double col_eps = 3.0 * M_PI / 180.0; // in radius

  // Iterate to resolve collision
  for (unsigned int n = 0; n < q_cur.size() - 1; n++)
  {
    std::cout << ">> Processing point " << (n+1) << "/" << q_cur.size() << " ..." << std::endl;
    q_cur[n+1] = resolve_point_collisions(q_cur[n], q_cur[n+1], col_eps, num_intervals);    
  }

  return q_cur;
}


/**
 * @brief Resolve possible collision state that exists on the linearly interpolated line between two discrete path points. 
 * 
 * This function is for ease of use, and is called by resolve_path_collisions(). \n
 * Colliding state of a path point is resolved through iteration with robot jacobians and gradient normal. \n
 * @param[in]   col_eps         The step for computing numerical differentiation for finger joints.
 * @param[in]   x0              First path point. Note that x0 mustn't be in collision state!!!
 * @param[in]   x1              The next path point, which would be modified using robot jacobian + contact normal when any collision state exists between x0 and x1.
 * @param[in]   num_intervals   Number of intervals of interpolation for dense discrete collision checking. Indicates the density of binary collision checking. 
 * @param[out]  x1_mod(actually x1)          The modified next path point, which would be equal to input x1 if no collision detected on the linearly interpolated line between x0 and x1.
 */
Eigen::Matrix<double, JOINT_DOF, 1> CollisionConstraint::resolve_point_collisions(Eigen::Matrix<double, JOINT_DOF, 1> x0, Eigen::Matrix<double, JOINT_DOF, 1> x1, double col_eps, unsigned int num_intervals)
{
  // Prep
  double e_cur;
  Eigen::Matrix<double, JOINT_DOF, 1> delta_x = Eigen::Matrix<double, JOINT_DOF, 1>::Zero();

  // Perform dense discrete collision checking on the linearly interpolated line between x0 and x1
  // unsigned int num_intervals = 100; // density

  // Iterate to check, and resolve collision state, until everything is ok
  bool no_collision = false;
  Eigen::Matrix<double, JOINT_DOF, 1> x_col;
  while (!no_collision)
  {
    // Check if any possible collision state
    double t_contact = dense_collision_checking(x0, x1, x_col, num_intervals);
    no_collision = (t_contact < 0.0); // if no time of contact, then no intermediate collision!

    // Resolve collision state by modifying x1 with the contact information at x_col
    if (!no_collision)
    {
      // Copy the data, for convenience
      Eigen::Matrix<double, JOINT_DOF, 1> x = x_col;

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

      // Compute updates and apply it until collision is resolved
      unsigned int cur_iter = 0;
      unsigned int max_iter = 500; // maximum iteration used to resolve the collision state
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
        x1 = x1 + dx.transpose(); // update the latter point to resolve the collision of interpolated point
        std::cout << "dx = " << dx << std::endl;
        x = (x1 - x0) * t_contact + x0; // obtain the interpolated state using time of contact

        // cope with position limit bounds
        double amount_out_of_bound = 0.0;
        for (unsigned int i = 0; i < JOINT_DOF; i++)
        {
          double tmp = std::min(std::max(x[i], _measurement.q_pos_lb[i]), _measurement.q_pos_ub[i]);
          amount_out_of_bound += std::fabs(x[i] - tmp);
          x[i] = tmp;//std::min(std::max(x[i], _measurement.q_pos_lb[i]), _measurement.q_pos_ub[i]);
        }
        std::cout << "amount of out-of-bounds = " << amount_out_of_bound << std::endl;

        // convert interpolated state back to std::vector<double> 
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

    }
  }

  return x1; // return the modified latter path point
}


/**
 * @brief Perform dense discrete, binary collision checking.
 * 
 * This function is for approximating Continuous-time Collision Checking, i.e. return the time of first contact on a (linearly) interpolated path.
 * @param[in,out]   x_col           If any colliding state exists in between x0 and x1, return that state.
 * @param[out]      t_contact       Time of contact. -1 if no intermediate collision state is found.
 */
double CollisionConstraint::dense_collision_checking(Eigen::Matrix<double, JOINT_DOF, 1> x0, Eigen::Matrix<double, JOINT_DOF, 1> x1, Eigen::Matrix<double, JOINT_DOF, 1> &x_col, unsigned int num_intervals)
{
  // Prep
  bool no_collision = true;
  double t_contact = -1;

  // Iterate to check possible collision state on a linearly interpolated path
  unsigned int n = 0;
  while (no_collision && n < num_intervals)
  {
    // Get interpolated point 
    Eigen::Matrix<double, JOINT_DOF, 1> x = (double)(n+1) * (x1 - x0) / num_intervals + x0;

    // Convert data type
    std::vector<double> xx(JOINT_DOF);
    for (unsigned int i = 0; i < JOINT_DOF; ++i)
      xx[i] = x[i];

    // Check
    double col_result = dual_arm_dual_hand_collision_ptr->check_self_collision(xx);
    no_collision = (col_result < 0.0);  // col_result 1 for colliding, -1 for collision-free
    
    // assign colliding state
    if (!no_collision)
    {
      t_contact = (double)(n+1) / num_intervals;
      x_col = x;
    }

    // counter
    n++;
  }

  return t_contact;//no_collision;
}


/**
 * Compute the minimum distance between the specified two links. 
 * When the specified two links are not colliding, the distance is manually set to safety margin. \n
 * This is because the dual_hand part contains multiple fingers ("manipulators"), and we only want to
 * know about the collision state of the specified two links.
 */
double CollisionConstraint::compute_dual_hands_collision_cost(Matrix<double, JOINT_DOF, 1> q_whole, std::string link_name_1, std::string link_name_2, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr)
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
  // cost = K_COL * cost; // weighting...

  return cost;
}


/**
 * @brief Compute the error vector for two specified links.
 * 
 * When the specified two links are not colliding, the error vector is manually set to zeros. \n
 * This function is for the same-hand collision part specifically, so in order to indicate the goodness 
 * of a step, we need to reflect the performance on three dimensions. \n
 * Since the calculated contact normal is a normalized vector, we need to convert it into a distance vector, 
 * whose elements indicate the minimum distance projected onto each axis.
 */
Vector3d CollisionConstraint::compute_dual_hands_collision_error_vector(Matrix<double, JOINT_DOF, 1> q_whole, std::string link_name_1, std::string link_name_2, boost::shared_ptr<DualArmDualHandCollision> &dual_arm_dual_hand_collision_ptr)
{
  // convert from matrix to std::vector
  std::vector<double> x(JOINT_DOF);
  for (unsigned int i = 0; i < JOINT_DOF; ++i)
    x[i] = q_whole[i];

  // Collision checking (or distance computation), check out the DualArmDualHandCollision class
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

  // Get cost (distance to margin of safety)
  double cost = std::max(d_hand_safe - min_distance, 0.0);

  // apply weighting
  // cost = K_COL * cost; // weighting...

  // Get projected distance vector
  Vector3d contact_normal = (dual_arm_dual_hand_collision_ptr->normal).cwiseAbs();

  return cost * contact_normal;
}


/**
 * Return the dense collision checking result, i.e. whether there exists possible collision state 
 * between transition from x0 to x1. \n
 * 
 * 0 for no collision, and 1 for collision.
 */
double CollisionConstraint::return_col_cost()
{
  // get the current joint value
  // _vertices is a VertexContainer type, a std::vector<Vertex*>
  const DualArmDualHandVertex *v0 = static_cast<const DualArmDualHandVertex*>(_vertices[0]);
  const Matrix<double, JOINT_DOF, 1> x0 = v0->estimate(); // return the current estimate of the vertex
  const DualArmDualHandVertex *v1 = static_cast<const DualArmDualHandVertex*>(_vertices[1]);
  const Matrix<double, JOINT_DOF, 1> x1 = v1->estimate(); // return the current estimate of the vertex

  // do dense discrete collision checking
  Matrix<double, JOINT_DOF, 1> x_col;
  double t_contact = dense_collision_checking(x0, x1, x_col, num_checks);

  double collision_cost = (t_contact < 0.0 ? 0.0 : 1.0); // t_contact < 0 indicates no collision

  return collision_cost;
}




#endif
