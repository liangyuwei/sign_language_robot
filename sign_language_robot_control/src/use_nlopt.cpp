// ROS
#include <ros/ros.h>

// NLopt
#include <nlopt.hpp> // C++ version!!!

// 
#include <vector>
#include <iostream>

// For acos, fabs
#include <cmath>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> // for Map()

// For KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// For file write and read
#include <string>
#include "H5Cpp.h"


// global flags
int count = 0;
bool first_iter = true;

using namespace Eigen;
using namespace H5;

typedef struct {
  // Cost func related
  Matrix<double, 6, 1> q_prev; // used across optimizations over the whole trajectory
  Vector3d elbow_pos_goal;
  Vector3d wrist_pos_goal;
  Matrix3d wrist_ori_goal;
  
  // Constraint func related
  //std::vector<double> qlb, qub;

  // KDL FK related
  //KDL::ChainFkSolverPos_recursive fk_solver;
  unsigned int num_wrist_seg = 0;
  unsigned int num_elbow_seg = 0; // initialized as flag

} my_constraint_struct;


// Used for computing cost and grad(numeric differentiation) in myfunc()
double compute_cost(KDL::ChainFkSolverPos_recursive fk_solver, Matrix<double, 6, 1> q_cur, unsigned int num_wrist_seg, unsigned int num_elbow_seg, my_constraint_struct *fdata)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL
  KDL::Frame elbow_cart_out, wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that the number here is not the segment ID, but the number of segments till target segment
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    return -1;
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for elbow link.");
  }
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1);
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    return -1;
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }

  // Compute cost function
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);
  Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 
  double cost = 1.0 * (fdata->elbow_pos_goal - elbow_pos_cur).norm() 
              + 20.0 * (fdata->wrist_pos_goal - wrist_pos_cur).norm() \
              + 10.0 * std::fabs( std::acos (( (fdata->wrist_ori_goal * wrist_ori_cur.transpose()).trace() - 1.0) / 2.0));
  if (!first_iter)
    cost += 0.5 * (q_cur - fdata->q_prev).norm();

  // Display for debug
  /*std::cout << "Cost func structure: " << std::endl
            << "elbow pos err = " << (fdata->elbow_pos_goal - elbow_pos_cur).norm() << std::endl
            << "wrist pos err = " << (fdata->wrist_pos_goal - wrist_pos_cur).norm() << std::endl
            << "wrist ori err = " << std::fabs( std::acos (( (fdata->wrist_ori_goal * wrist_ori_cur.transpose()).trace() - 1) / 2.0)) << std::endl
            << "smoothness err = " << (first_iter? 0 : (q_cur - fdata->q_prev).squaredNorm()) << std::endl;*/
  //std::cout << "Total cost: " << cost << std::endl;
  //std::cout << "During evaluation, q_prev = " << (fdata->q_prev) << std::endl; // checked


  // Return cost function value
  return cost;


}


// This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver 
KDL::ChainFkSolverPos_recursive setup_kdl(my_constraint_struct &constraint_data)
{
  // Params
  const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/ur_description/urdf/ur5_robot_with_hands.urdf";
  const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
  const std::string ELBOW_LINK = "right_forearm_link";
  const std::string WRIST_LINK = "right_ee_link";

  // Get tree
  KDL::Tree kdl_tree; 
   if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      exit(-1);
   }
  //ROS_INFO("Successfully built a KDL tree from URDF file.");

  // Get chain  
  KDL::Chain kdl_chain; 
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    exit(-1);
  }
  //ROS_INFO("Successfully obtained chain from root to wrist.");


  // Find segment number for wrist and elbow links, store in constraint_dat
  if (constraint_data.num_wrist_seg == 0 || constraint_data.num_elbow_seg == 0) // if the IDs not set
  {
    unsigned int num_segments = kdl_chain.getNrOfSegments();
    constraint_data.num_wrist_seg = num_segments - 1;
    //ROS_INFO_STREAM("There are " << num_segments << " segments in the kdl_chain");
    for (unsigned int i = 0; i < num_segments; ++i){
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        constraint_data.num_elbow_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        break;
      }
    }
  }


  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  //ROS_INFO_STREAM("Joint dimension is: " << kdl_chain.getNrOfJoints()); // 6 joints, 8 segments, checked!

  return fk_solver;

}


// Loss function
double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{

  // Counter information
  ++count;
  //std::cout << "Evaluation " << count << std::endl;


  // Get additional information by typecasting void* f_data(user-defined data)
  my_constraint_struct *fdata = (my_constraint_struct *) f_data;


  // Get fk solver( and set IDs if first time)
  KDL::ChainFkSolverPos_recursive fk_solver = setup_kdl(*fdata);
  //std::cout << "At evaluation of cost func, after setting up kdl solver." << std::endl;


  // Calculate loss function(tracking performance + continuity)
  //std::vector<double> x_tmp = x;
  Matrix<double, 6, 1> q_cur;
  q_cur << x[0], x[1], x[2], x[3], x[4], x[5]; //Map<Matrix<double, 6, 1>>(x_tmp.data(), 6, 1);
  double cost = compute_cost(fk_solver, q_cur, fdata->num_wrist_seg, fdata->num_elbow_seg, fdata);


  // Compute gradient using Numeric Differentiation
  // only compute gradient if not NULL
  if(!grad.empty())
  {
    double eps = 0.001;
    Matrix<double, 6, 1> q_tmp;
    double cost1, cost2;
    for (unsigned int i = 0; i < q_tmp.size(); ++i)
    {
      // 1
      q_tmp = q_cur;
      q_tmp[i] += eps;
      cost1 = compute_cost(fk_solver, q_tmp, fdata->num_wrist_seg, fdata->num_elbow_seg, fdata);
      // 2
      q_tmp = q_cur;
      q_tmp[i] -= eps;
      cost2 = compute_cost(fk_solver, q_tmp, fdata->num_wrist_seg, fdata->num_elbow_seg, fdata);
      // combine 1 and 2
      grad[i] = (cost1 - cost2) / (2.0 * eps);
    }
  }


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


bool write_h5(const std::string file_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector)
{
  // Set up file name and dataset name
  const H5std_string  FILE_NAME(file_name);
  const H5std_string  DATASET_NAME(dataset_name);

  // Convert 2-dim std::vector to 2-dim raw buffer(array)
  double data[ROW][COL];
  for (int j = 0; j < ROW; j++)
  {
    for (int i = 0; i < COL; i++)
    data[j][i] = data_vector[j][i];
  }

  try
  {
    // Create a file, or earse all data of an existing file
    H5File file( FILE_NAME, H5F_ACC_RDWR ); // H5F_ACC_TRUNC
      
    // Create data space for fixed size dataset
    hsize_t dimsf[2];              // dataset dimensions
    dimsf[0] = ROW;
    dimsf[1] = COL;
    DataSpace dataspace(2, dimsf);

    // Define datatype for the data in the file.
    IntType datatype( PredType::NATIVE_DOUBLE );
    datatype.setOrder( H5T_ORDER_LE );

    // Create a new dataset within the file using defined dataspace and datatype
    DataSet dataset = file.createDataSet( DATASET_NAME, datatype, dataspace );

    //Write the data to the dataset using default memory space, file space, and transfer properties.
    dataset.write( data, PredType::NATIVE_DOUBLE );

  } 
  // catch failure caused by the H5File operations
  catch( FileIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSet operations
  catch( DataSetIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSpace operations
  catch( DataSpaceIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSpace operations
  catch( DataTypeIException error )
  {
    error.printErrorStack();
    return -1;
  }

  // Finish
  return true;

}


// Read h5 file for joint path
std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string dataset_name)
{
  // Set up file name and dataset name
  const H5std_string  FILE_NAME(file_name);
  const H5std_string  DATASET_NAME(dataset_name);

  try
  {
    // Open the specified file and the specified dataset in the file.
    H5File file( FILE_NAME, H5F_ACC_RDONLY );
    DataSet dataset = file.openDataSet(DATASET_NAME);

    // Get the class of the datatype that is used by the dataset.
    H5T_class_t type_class = dataset.getTypeClass();

    // Get dataspace of the dataset.
    DataSpace dataspace = dataset.getSpace();

    // Get the dimension size of each dimension in the dataspace and display them.
    hsize_t dims_out[2];
    int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
    int ROW = dims_out[0], COL = dims_out[1];

    // Read data into raw buffer(array) and convert to std::vector
    double data_array[ROW][COL];
    dataset.read(data_array, PredType::NATIVE_DOUBLE);
    std::vector<std::vector<double>> data_vector(ROW, std::vector<double>(COL));
    for (int j = 0; j < dims_out[0]; j++)
    {
      for (int i = 0; i < dims_out[1]; i++)
        data_vector[j][i] = data_array[j][i];
    }

    return data_vector;

  } 
   // catch failure caused by the H5File operations
   catch( FileIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSet operations
   catch( DataSetIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSpace operations
   catch( DataSpaceIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSpace operations
   catch( DataTypeIException error )
   {
      error.printErrorStack();
      exit(-1);
   }

}



int main(int argc, char **argv)
{

  // Input Cartesian trajectories
  const unsigned int joint_value_dim = 6;   
  std::vector<double> x(joint_value_dim);
  const std::string in_file_name = "fake_elbow_wrist_paths.h5";
  const std::string in_dataset_name = "fake_path_right_1";
  std::vector<std::vector<double>> read_wrist_elbow_traj = read_h5(in_file_name, in_dataset_name); 
  // using read_h5() does not need to specify the size!!!
  // elbow pos(3) + wrist pos(3) + wrist rot(9) = 15-dim
  unsigned int num_datapoints = read_wrist_elbow_traj.size(); 

  // display a few examples(debug)
  /*
  std::cout << "Display the read fake path data: " << std::endl;
  for (int i = 0; i < num_datapoints; ++i)
  {
    for (int j = 0; j < read_wrist_elbow_traj[i].size(); ++j)
      std::cout << read_wrist_elbow_traj[i][j] << " ";
    std::cout << std::endl;
  }
  exit(0);
  */

  // Parameters setting
  std::vector<double> qlb = {-1.0, -1.0, -1.0, -1.57, -1.57, -1.57};//{-6.28, -5.498, -7.854, -6.28, -7.854, -6.28};
  std::vector<double> qub = {1.0, 1.0, 1.0, 1.57, 1.57, 1.57};//{6.28, 7.069, 4.712, 6.28, 4.712, 6.28};
  //std::vector<double> x(joint_value_dim);
  double minf;
  double tol = 1e-4;
  double stopval = 1e-8;


  // Set up KDL(get solver, wrist ID and elbow ID)
  my_constraint_struct constraint_data; 
  setup_kdl(constraint_data); // set IDs, discard the solver handle

  //std::cout << "At Main func, before set up optimizer." << std::endl;

  // Set up optimizer
  /*try
  {*/
  nlopt::opt opt(nlopt::LD_SLSQP, joint_value_dim); // nlopt::LD_SLSQP
  /*}
  catch(std::bad_alloc e)
  {
    std::cout << "Something went wrong in the constructor: " << e.what() << std::endl;
    return -1;
  }*/

  opt.set_lower_bounds(qlb); // set lower bounds
  opt.set_upper_bounds(qub); // set upper bounds
  opt.set_stopval(1e-6); // stop value
  opt.set_ftol_rel(1e-8); // objective function value changes by less than `tol` multiplied by the absolute value of the function value
  //opt.set_ftol_abs(1e-12); // objective function value changes by less than `tol`
  opt.set_xtol_rel(1e-6); // optimization parameters' magnitude changes by less than `tol` multiplied by the current magnitude(can set weights for each dimension)
  //opt.set_xtol_abs(1e-8); // optimization parameters' magnitude changes by less than `tol`
  opt.set_maxeval(200); // maximum evaluation
  //opt.set_maxtime(3.0); // maximum time


  // Start iterations
  std::vector<std::vector<double> > q_results(num_datapoints, std::vector<double>(joint_value_dim));
  for (unsigned int it = 0; it < num_datapoints; ++it)
  {

    // Reset counter.
    count = 0; 

    // Set goal point
    std::vector<double> path_point = read_wrist_elbow_traj[it];
    std::vector<double> wrist_pos(path_point.begin(), path_point.begin()+3); // 3-dim
    std::vector<double> wrist_ori(path_point.begin()+3, path_point.begin()+12); // 9-dim
    std::vector<double> elbow_pos(path_point.begin()+12, path_point.begin()+15); //end()); // 3-dim
    /** check the extracted data sizes **
    std::cout << "wrist_pos.size() = " << wrist_pos.size() << ", ";
    std::cout << "wrist_ori.size() = " << wrist_ori.size() << ", ";
    std::cout << "elbow_pos.size() = " << elbow_pos.size() << "." << std::endl; */
    // convert
    Vector3d wrist_pos_goal = Map<Vector3d>(wrist_pos.data(), 3, 1);
    Matrix3d wrist_ori_goal = Map<Matrix<double, 3, 3, RowMajor>>(wrist_ori.data(), 3, 3);
    Vector3d elbow_pos_goal = Map<Vector3d>(elbow_pos.data(), 3, 1);
    constraint_data.wrist_pos_goal = wrist_pos_goal;
    constraint_data.wrist_ori_goal = wrist_ori_goal;
    constraint_data.elbow_pos_goal = elbow_pos_goal;
    /** Be careful with the data assignment above !!!! **
    //if (it == 10){

    std::cout << "Display the goal point: " << std::endl;
    std::cout << "Path point is: ";
    for (int i = 0; i < wrist_pos.size() + wrist_ori.size() + elbow_pos.size(); ++i) std::cout << path_point[i] << " "; 
    std::cout << std::endl << "Wrist pos is: " << constraint_data.wrist_pos_goal << std::endl
                           << "Wrist rot is: " << constraint_data.wrist_ori_goal << std::endl
                           << "Elbow pos is: " << constraint_data.elbow_pos_goal << std::endl;
    exit(0);
    //} */
    /*std::cout << "q_prev is: " << constraint_data.q_prev.transpose() << std::endl;
    if (it == 6)
      exit(0);*/


    // Set up objective function and additional data to pass in
    my_constraint_struct *f_data = &constraint_data;
    opt.set_min_objective(myfunc, (void *) f_data); // set objective function to minimize; with no additional information passed(f_data)


    // Start optimization
    /*try
    {*/

      // Display messages
      std::cout << "========== Path point " << it + 1  <<"/" << num_datapoints << " ==========" << std::endl;
      nlopt::result opt_result;
      //std::cout << "first_iter is " << first_iter << " before optimizing the first point." << std::endl; // ok, checked
      if (first_iter)
        opt_result = opt.optimize(x, minf);
      else
      {
        //std::cout << "previous result is: " << constraint_data.q_prev << "." << std::endl; // not changing!!
        /*std::cout << "previous result is: ";
        for (int t = 0; t < q_results[it-1].size(); ++t)  std::cout << q_results[it-1][t] << " ";
        std::cout << std::endl;*/
        x = q_results[it-1]; // pass the previous result into x, otherwise the previous result would be modified again!!!(passed as reference into opt.optimize())
        opt_result = opt.optimize(x, minf); // use previous result as initial guess
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
        case nlopt::MAXTIME_REACHED:
          std::cout << "Optimization terminated due to MAXTIME reached." << std::endl;
          break;
      }
      std::cout << "NLopt found minimum f: " << minf << " after " << opt.get_numevals() << " evaluations." << std::endl;


      // Store the result(joint values)
      q_results[it] = x;
      //std::vector<double> q_tmp(x); // copy construct
      //q_results.push_back(q_tmp);

      // display the current result
      /*std::cout << "q result is: ";
      for (int t = 0; t < x.size(); ++t)  std::cout << x[t] << " ";
      std::cout << std::endl;*/

      // Record the current joint as q_prev
      //Matrix<double, 6, 1> q_prev = Map<Eigen::Matrix<double, 6, 1>>(x.data(), 6, 1);
      constraint_data.q_prev = Map<Eigen::Matrix<double, 6, 1>>(x.data(), 6, 1); // used across optimizations over the whole trajectory  
      //std::cout << "q_prev is: " << constraint_data.q_prev.transpose() << std::endl;
      first_iter = false;

    /*}
    catch (std::runtime_error e1){
      std::cout << "Runtime error: " << e1.what() << std::endl;
    }
    catch (std::invalid_argument e2){
      std::cout << "Invalid argument: " << e2.what() << std::endl;    
    }
    catch (std::bad_alloc e3){
      std::cout << "Ran out of memory: " << e3.what() << std::endl;    
    }*/

    /*catch (nlopt::roundoff_limited e4){ // will be caught earlier handler for 'std::runtime_error e1'
      std::cout << "Roundoff errors limited progress: " << e4.what() << std::endl;    
    }
    catch (nlopt::forced_stop e5){ // will be caught earlier handler for 'std::runtime_error
      std::cout << "Forced termination: " << e5.what() << std::endl;    
    }*/

    //std::cout << "first_iter is " << first_iter << " after optimizing the first point." << std::endl;
    //exit(0);
  }


  // display the results
  /*
  std::cout << "q_results is: " << std::endl;
  for (int i = 0; i < num_datapoints; ++i)
  {
    for (int j = 0; j < joint_value_dim; ++j)
      std::cout << q_results[i][j] << " ";
    std::cout << std::endl;
  }*/


  // Store the results
  const std::string file_name = "ik_results.h5";
  const std::string dataset_name = "ik_result_right_1";
  bool result = write_h5(file_name, dataset_name, num_datapoints, joint_value_dim, q_results);
  if(result)
    std::cout << "Joint path results successfully stored!" << std::endl;

 
  return 0;

}



