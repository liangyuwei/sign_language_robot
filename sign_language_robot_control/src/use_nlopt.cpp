#include <nlopt.h>
#include <vector>
#include <iostream>

// For acos
#include <cmath>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// For KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// For file write and read
#include <fstream>
#include <string>


int count = 0;

typedef struct {

  // Cost func related
  Eigen::Matrix<double, 6, 1> q_prev(6); // used across optimizations over the whole trajectory
  Eigen::Vector3d elbow_pos_goal;
  Eigen::Vector3d wrist_pos_goal;
  Eigen::Matrix3d wrist_ori_goal;
  
  // Constraint func related
  std::vector<double> qlb(6), qub(6);

  // KDL FK related
  KDL::ChainFkSolverPos_recursive fk_solver;
  unsigned int num_wrist_seg;
  unsigned int num_elbow_seg;

} my_constraint_struct;



// Loss function
double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{

  // Counter information
  ++count;
  std::cout << "Evaluation " << count << std::endl;


  // Get additional information by typecasting void* f_data(user-defined data)
  my_constraint_struct *d = (my_constraint_struct *) f_data;
  

  // Calculate loss function(tracking performance + continuity)
  Eigen::Matrix<double, 6, 1> q_cur = Eigen::Map<Eigen::Matrix<double, 6, 1> >(x, 6, 1);
  double cost = compute_cost(fk_solver, q_cur, num_wrist_seg, num_elbow_seg, f_data);


  // Compute gradient using Numeric Differentiation
  // only compute gradient if not NULL
  if(!grad.empty())
  {
    double eps = 0.01;
    Eigen::Matrix<double, 6, 1> q_tmp;
    double cost1, cost2;
    for (unsigned int i = 0; i < q_tmp.size(); ++i)
    {
      // 1
      q_tmp = q_cur;
      q_tmp[i] += eps;
      cost1 = compute_cost(fk_solver, q_tmp, num_wrist_seg, num_elbow_seg, f_data);
      // 2
      q_tmp = q_cur;
      q_tmp[i] -= eps;
      cost2 = compute_cost(fk_solver, q_tmp, num_wrist_seg, num_elbow_seg, f_data);
      // combine 1 and 2
      grad[i] = (cost1 - cost2) / (2.0 * eps);
    }
  }


  // Return cost function value
  return cost;

}


double compute_cost(KDL::ChainFkSolverPos_recursive fk_solver, Eigen::Matrix<double, 6, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, my_constraint_struct &f_data)
{

  // Get joint angles
  unsigned int n = x.size();
  KDL::JntArray q_in(n); 
  for (unsigned int i = 0; i < n; ++i)
  {
    q_in(i) = q_cur(i);
  }


  // Do FK using KDL
  KDL::Frame elbow_cart_out, wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    return -1;
  }
  else{
    ROS_INFO_STREAM("FK solver succeeded for elbow link.");
  }
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    return -1;
  }
  else{
    ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }


  // Compute cost function
  Eigen::Vector3d elbow_pos_cur = Eigen::Map<Eigen::Vector3d>(elbow_cart_out.p.data, 3, 1);
  Eigen::Vector3d wrist_pos_cur = Eigen::Map<Eigen::Vector3d>(wrist_cart_out.p.data, 3, 1);
  Eigen::Matrix3d wrist_ori_cur = Eigen::Map<Eigen::Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 
  double cost = (f_data.elbow_pos_goal - elbow_pos_cur).squaredNorm() \
              + (f_data.wrist_pos_goal - wrist_pos_cur).squaredNorm() \
              + std::pow( std::acos (( (f_data.wrist_ori_goal * wrist_ori_cur.transpose()).trace() - 1) / 2.0), 2);

  if (!first_iter)
    cost += (q_cur - f_data.q_prev).squaredNorm();


  // Return cost function value
  return cost;

}


// Constraint function; expected to be myconstraint(x)<=0
void myconstraint(unsigned m, double *result, unsigned n, const double *x,
                             double *grad, /* NULL if not needed */
                             void *f_data)
{

  // No constraints!!! Upper and lower bounds are bounds!!!

  /*

  // m-dim vector-valued constraints, n-dim joints
  my_constraint_struct *d = (my_constraint_struct *) f_data;
  f_data.qlb
  f_data.qub

  // Compute gradients of constraint functions(if non-NULL, it points to an array with the size of m*n; access through)
  if (grad){

    for (unsigned i = 0; i < m; ++i)
    {
      for(unsigned j = 0; j < n; ++j)
      {

        grad[i * n + j] = (i < 6) ? 1 : -1; // 6 upperbounds

      }

    }

  }


  // Compute constraints and store in `result`
  for (unsigned int i = 0; i < m; ++i)
  {
    result[i] = 
  }
  
  */


}


// Set up KDL, feed in constraint_data with solver, wrist ID and elbow ID
bool setup_kdl(my_constraint_struct &constraint_data)
{
  

  // Params
  const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/ur_description/urdf/ur5_robot_with_hands.urdf";
  const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
  const std::string ELBOW_LINK = "left_forearm_link";
  const std::string WRIST_LINK = "left_ee_link";

  // Get tree
  KDL::Tree kdl_tree; 
   if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
  ROS_INFO("Successfully built a KDL tree from URDF file.");

  // Get chain  
  KDL::Chain kdl_chain; 
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    return false;
  }
  ROS_INFO("Successfully obtained chain from root to wrist.");

  // Find segment number for wrist and elbow links, for calculating FK later
  unsigned int num_segments = kdl_chain.getNrOfSegments();
  constraint_data.num_wrist_seg = num_segments - 1;
  ROS_INFO_STREAM("There are " << num_segments << " segments in the kdl_chain");
  for (unsigned int i = 0; i < num_segments; ++i){
    if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
      constraint_data.num_elbow_seg = i;
      ROS_INFO_STREAM("Elbow link found.");
      break;
    }
  }

  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  constraint_data.fk_solver = fk_solver;
  ROS_INFO_STREAM("Joint dimension is: " << kdl_chain.getNrOfJoints()); // 6 joints, 8 segments, checked!


  return true;

}



bool first_iter = true;

int main(int argc, char **argv[])
{

  // Input Cartesian trajectories
  const unsigned int joint_value_dim = 6; 
  unsigned int num_datapoints = 1000;
  std::vector<double> raw_wrist_elbow_traj(num_datapoints, 15); // elbow pos + wrist pos + wrist rot
  std::fstream fin("cart_traj.csv", ios::in);
  std::string line;
  int row_id = 0;
  while (getline(fin, line)) // read a whole line
  {
    // Display for debug
    std::cout << "Read a line from .csv file: " << line << std::endl;
  
    // Read each column separated by ','
    std::istringstream sin(line);
    std::string ss;
    int col_id = 0;
    while(getline(sin, ss, ','))
    {
      raw_wrist_elbow_traj[row_id][col_id] = std::atof(ss); // convert to float and store
      col_id++;    
    }
  
    row_id++;
  }

  // Convert Cartesian trajectories
  std::vector<std::vector<double> > wrist_elbow_traj(num_datapoints, std::vector<double>(joint_value_dim));


  // Parameters setting
  std::vector<double> qlb = {-6.28, -5.498, -7.854, -6.28, -7.854, -6.28};
  std::vector<double> qub = {6.28, 7.069, 4.712, 6.28, 4.712, 6.28};
  std::vector<double> x(joint_value_dim);
  double minf;
  double tol = 1e-4;
  double stopval = 1e-8;


  // Set up KDL(get solver, wrist ID and elbow ID)
  my_constraint_struct constraint_data;
  setup_kdl(constraint_data);


  // Set up optimizer
  try
    nlopt::opt opt(nlopt::LD_SLSQP, joint_value_dim);
  catch(std::bad_alloc e)
  {
    std::cout << "Something went wrong in the constructor: " << e << std::endl;
    return -1;
  }
  opt.set_min_objective(myfunc, NULL); // set objective function to minimize; with no additional information passed(f_data)
  opt.set_lower_bounds(qlb); // set lower bounds
  opt.set_upper_bounds(qub); // set upper bounds
  opt.set_stopval(stopval); // stop value
  opt.set_ftol_rel(tol); // objective function value changes by less than `tol` multiplied by the absolute value of the function value
  opt.set_ftol_abs(1e-8); // objective function value changes by less than `tol`
  opt.set_xtol_rel(tol); // optimization parameters' magnitude changes by less than `tol` multiplied by the current magnitude(can set weights for each dimension)
  opt.set_xtol_abs(1e-12); // optimization parameters' magnitude changes by less than `tol`
  opt.set_maxeval(100); // maximum evaluation
  opt.set_maxtime(1.0); // maximum time



  // Start iterations
  std::vector<std::vector<double> > q_results(num_datapoints, std::vector<double>(joint_value_dim));
  for (unsigned int it = 0; it < num_datapoints; ++it)
  {

    // Set goal point
    Eigen::Vector3d elbow_pos_goal;
    Eigen::Vector3d wrist_pos_goal;
    Eigen::Matrix3d wrist_ori_goal;


    // Start optimization
    try
    {
      nlopt::result opt_result = opt.optimize(x, minf);
    }
    catch (std::runtime_error e1){
      std::cout << "Runtime error: " << e1 << std::endl;
    }
    catch (std::invalid_argument e2){
      std::cout << "Invalid argument: " << e2 << std::endl;    
    }
    catch (std::bad_alloc e3){
      std::cout << "Ran out of memory: " << e3 << std::endl;    
    }
    catch (nlopt::roundoff_limited e4){
      std::cout << "Roundoff errors limited progress: " << e4 << std::endl;    
    }
    catch (nlopt::forced_stop e5){
      std::cout << "Forced termination: " << e5 << std::endl;    
    }
    switch(opt_result)
    {
      case nlopt::SUCCESS:
        std::cout << "Optimization finished successfully." << std::endl;
        break;
      case nlopt::STOPVAL_REACHED:
        std::cout << "Optimization terminated due to STOPVAL reached." << std::endl;
        break;
      case nlopt::FTOL_REACHED:
        std::cout << "Optimization terminated due to FTOL reached." << std::endl;
        break;
      case nlopt::XTOL_REACHED:
        std::cout << "Optimization terminated due to XTOL reached." << std::endl;
        break;
      case nlopt::MAXEVAL_REACHED:
        std::cout << "Optimization terminated due to MAXEVAL reached." << std::endl;
        break;
      case nlopt::MAXTIME_REACHED`:
        std::cout << "Optimization terminated due to MAXTIME reached." << std::endl;
        break;
    }
    std::cout << "Path point " << it + 1  <<"/" << num_datapoints << " -- NLopt found minimum f: " << minf << " after " << opt.get_numevals() << " evaluations." << std::endl;


    // Store the result(joint values)
    q_results[it] = x;


    // Record the current joint as q_prev
    constraint_data.q_prev = Eigen::Map<Eigen::Matrix<double, 6, 1>>(x.data()); // used across optimizations over the whole trajectory  
    first_iter = false;

  }


  // Store the results
  std::ofstream f;
  f.open("ik_data.csv", ios::out);
  for (unsigned int i = 0; i < num_datapoints; ++i)
  {
    for (unsigned int j = 0; j < joint_value_dim; ++j)
    {
      if (i != 0) f << ',';
      f << q_results[i][j];
    }
    f << std::endl;
  }
  f.close();

 
  return 0;

}



