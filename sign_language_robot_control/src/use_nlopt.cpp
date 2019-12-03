#include <math.h>
#include <nlopt.h>
#include <vector>
#include <iostream>

int count = 0;

typedef struct {

  std::vector<double> qlb, qub;
  std::vector<double> q_previous;

} my_constraint_data;



// Loss function
double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{

  // Counter information
  ++count;
  std::cout << "Evaluation " << count << std::endl;


  // Get additional information by typecasting void* f_data(user-defined data)
  //


  // Compute the configurations of elbow and wrist links
  


  // Calculate loss function



  // Compute gradient using CppAD
  // only compute gradient if not NULL(???)
  if(!grad.empty())
  {
    
  }

  return 


}


// Constraint function; expected to be myconstraint(x)<=0
void myconstraint(unsigned m, double *result, unsigned n, const double *x,
                             double *grad, /* NULL if not needed */
                             void *f_data)
{
  // m-dim vector-valued constraints, n-dim joints
  my_constraint_data *d = (my_constraint_data *) data;


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



int main(int argc, char **argv[])
{

  // Input Cartesian trajectories
  

  // Settings
  const unsigned int joint_value_dim = 6; 
  unsigned int num_datapoints = 1000;

  my_constraint_data constraint_data;
  constraint_data.qlb = {};
  constraint_data.qub = {};


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



