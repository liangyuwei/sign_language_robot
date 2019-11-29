#include <math.h>
#include <nlopt.h>
#include <vector>

int count = 0;

typedef struct {
  double vel_limits[], acc_limits[];
  double jlb[], jub[];
} my_constraint_data;


// Loss function
double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{

  printf("Evaluation %d\n", count);
  // Counter
  ++count;

  // only compute gradient if not NULL(???)
  if(grad)
  {

  }

  return 


}

// Constraint function; expected to be myconstraint(x)<=0
double myconstraint(const double *x, double *grad, void *data)
{

  my_constraint_data *d = (my_constraint_data *) data;

  double a = d->a, b = d->b;
    if (grad) {
      grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
      grad[1] = -1.0;
    }
  return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);


}



int main(int argc, char **argv[])
{

  // Input Cartesian trajectories
  

  // Settings
  const int joint_value_dim = 6; // the sequence of time differences is also the optimization variable
  int num_datapoints = 1000;
  my_constraint_data data; // set up constraint parameters
  data.vel_limits = 
  data.acc_limits = 


  // Set up optimiaztion
  double lb[num_datapoints * joint_value_dim + num_datapoints - 1]; // default initialization to zeros
  double ub[num_datapoints * joint_value_dim + num_datapoints - 1];
  for (int i = 0; i < num_datapoints; ++i) // set joint limits
  {
    for (int j = 0; j < joint_value_dim; ++j)
    {
      lb[i * joint_value_dim + j] = data.jlb[j];
      ub[i * joint_value_dim + j] = data.jub[j];
    }
  }
  for (int t = 0; t < num_datapoints - 1; ++t) // set time difference's limit
  {
    lb[num_datapoints * joint_value_dim + t] = 0;
    ub[num_datapoints * joint_value_dim + t] = HUGE_VAL;
  }

  
  // Set up optimizer
  nlopt::opt opt(nlopt::LD_SLSQP, num_datapoints * joint_value_dim + num_datapoints - 1);
  nlopt_set_lower_bounds(opt, lb); // set lower bounds
  nlopt_set_upper_bounds(opt, ub); // set upper bounds
  nlopt::opt:set_min_objective(myfunc, NULL); // set objective function to minimize
  nlopt_add_inequality_constraint(opt, myconstraint, &data[0], 1e-8); // inequality constraint for each optimization variable
  nlopt_add_inequality_constraint(opt, myconstraint, &data[0], 1e-8); 


  // Start optimization
  double x[num_datapoints * joint_value_dim + num_datapoints - 1]; // default initialization to zeros
  double minf; // for storing the minimum objective value
  if (nlopt::opt::optimize(opt, x, &minf) < 0)
  {
    printf("NLopt failed!\n");
  }
  else
  {
    printf("NLopt found minimum f: %0.10g, after %d evaluations\n", minf, count);
  }
  

  // Finish optimization
  nlopt_destroy(opt);
  
  return 0;

}



