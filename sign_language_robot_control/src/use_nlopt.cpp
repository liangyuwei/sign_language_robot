#include <math.h>
#include <nlopt.h>
#include <vector>
#include <iostream>

int count = 0;

typedef struct {

  // Cost func related
  std::vector<double> q_previous;
  std::vector<double> exp_elbow_pos(3);
  KDL::FRAME exp_wrist_conf;
  
  // Constraint func related
  std::vector<double> qlb, qub;

  // KDL FK related
  KDL::ChainFkSolverPos_recursive fk_solver;
  KDL::Chain kdl_chain;
  unsigned int num_wrist_seg;
  unsigned int num_elbow_seg;


  

} my_constraint_struct;


typedef struct {

  std::vector<double> elbow_pos(3);
  KDL::Frame wrist_conf;

} wrist_elbow_struct;


// Loss function
double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{

  // Counter information
  ++count;
  std::cout << "Evaluation " << count << std::endl;


  // Get additional information by typecasting void* f_data(user-defined data)
  my_constraint_struct *d = (my_constraint_struct *) f_data;
  

  // Compute the configurations of elbow and wrist links
  unsigned int n = x.size();
  KDL::JntArray q_in(n); // initialize a JntArray for KDL FK computation
  for (unsigned int i = 0; i < n; ++i)
  {
    q_in(i) = x[i];
  }
  wrist_elbow_struct wrist_elbow_data; // initialize a struct for storing the elbow and wrist configurations
  wrist_elbow_data = KDL_FK(f_data.fk_solver, q_in, f_data.num_wrist_seg, f_data.num_elbow_seg);


  // Calculate loss function
  double cost;
  exp_elbow_pos
  exp_wrist_conf


  // Compute gradient using numeric differentiation
  // only compute gradient if not NULL(???)
  if(!grad.empty())
  {
    
  }

  return 

}


wrist_elbow_struct KDL_FK(KDL::ChainFkSolverPos_recursive fk_solver, KDL::JntArray q_in, unsigned int num_wrist_seg, unsigned int num_elbow_seg)
{

  // Compute the configurations of elbow and wrist
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


  // Construct struct and return
  wrist_elbow_struct wrist_elbow_data;
  wrist_elbow_data.elbow_pos[0] = elbow_cart_out.p.data[0];
  wrist_elbow_data.elbow_pos[1] = elbow_cart_out.p.data[1];
  wrist_elbow_data.elbow_pos[2] = elbow_cart_out.p.data[2];
  wrist_elbow_data.wrist_conf = wrist_cart_out;

  return wrist_elbow_data;

}


// Constraint function; expected to be myconstraint(x)<=0
void myconstraint(unsigned m, double *result, unsigned n, const double *x,
                             double *grad, /* NULL if not needed */
                             void *f_data)
{
  // m-dim vector-valued constraints, n-dim joints
  my_constraint_struct *d = (my_constraint_struct *) f_data;


  // Compute gradients of constraint functions(if non-NULL, it points to an array with the size of m*n; access through)
  if (grad){

    for (unsigned i = 0; i < m; ++i)
    {
      for(unsigned j = 0; j < n; ++j)
      {

        grad[i * n + j] = 

      }

    }

  }


  // Compute constraints and store in `result`
  result[i] = 


}

double q_pos_constraint()



int main(int argc, char **argv[])
{

  // Input Cartesian trajectories
  

  // Settings
  const unsigned int joint_value_dim = 6; 
  unsigned int num_datapoints = 1000;

  my_constraint_struct constraint_data;
  constraint_data.qlb = {-6.28, -5.498, -7.854, -6.28, -7.854, -6.28};
  constraint_data.qub = {6.28, 7.069, 4.712, 6.28, 4.712, 6.28};


  std::vector<double> x(joint_value_dim); // optimization variables, default initialized to zeros
  double minf; // for storing the minimum objective value

  unsigned int num_constraints = joint_value_dim * 2; // upper+lower bounds for each JOINT
  std::vector<double> vtol(num_constraints, 1e-8); // vector-valued tolerance
  
  double tol = 1e-4;
  double stopval = 1e-8;

  // Set up optimizer
  try
    nlopt::opt opt(nlopt::LD_SLSQP, num_datapoints * joint_value_dim + num_datapoints - 1);
  catch(std::bad_alloc e)
  {
    std::cout << "Something went wrong in the constructor: " << e << std::endl;
  }
  opt.set_min_objective(myfunc, NULL); // set objective function to minimize; with no additional information passed(f_data)
  opt.set_lower_bounds(constraint_data.qlb); // set lower bounds
  opt.set_upper_bounds(constraint_data.qub); // set upper bounds
  opt.add_inequality_mconstraint(myconstraint, NULL, vtol); // vector-valued constraints(more convenient)
  opt.set_stopval(stopval); // stop value
  opt.set_ftol_rel(tol); // objective function value changes by less than `tol` multiplied by the absolute value of the function value
  opt.set_ftol_abs(1e-8); // objective function value changes by less than `tol`
  opt.set_xtol_rel(tol); // optimization parameters' magnitude changes by less than `tol` multiplied by the current magnitude(can set weights for each dimension)
  opt.set_xtol_abs(1e-12); // optimization parameters' magnitude changes by less than `tol`
  opt.set_maxeval(100); // maximum evaluation
  opt.set_maxtime(1.0); // maximum time
  

  // Start optimization
  try
  {
    nlopt::result opt_result = opt.optimize(x, minf);
  }
  catch (std::runtime_error e1)
  {
    std::cout << "Runtime error: " << e1 << std::endl;
  }
  catch (std::invalid_argument e2)
  {
    std::cout << "Invalid argument: " << e2 << std::endl;    
  }
  catch (std::bad_alloc e3)
  {
    std::cout << "Ran out of memory: " << e3 << std::endl;    
  }
  catch (nlopt::roundoff_limited e4)
  {
    std::cout << "Roundoff errors limited progress: " << e4 << std::endl;    
  }
  catch (nlopt::forced_stop e5)
  {
    std::cout << "Forced termination: " << e5 << std::endl;    
  }


  // Display, store results and exit
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
  std::cout << "NLopt found minimum f: " << minf << " after " << opt.get_numevals() << " evaluations." << std::endl;
  
  return 0;

}



