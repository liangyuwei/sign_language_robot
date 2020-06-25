#include <iostream>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> // for Map()

// For file write and read
#include <string>
#include "H5Cpp.h"
//#include "H5Location.h"


#include "generate_trajectory_using_DMP.h"


using namespace std;
using namespace Eigen;
using namespace H5;


bool DMPTrajectoryGenerator::save_eigen_matrix_h5(MatrixXd data, std::string name)
{
  unsigned int nrow = data.rows();
  unsigned int ncol = data.cols();
  std::vector<std::vector<double> > data_vec(nrow, std::vector<double>(ncol));
  for (unsigned int i = 0; i < nrow; i++)
  {
    for (unsigned int j = 0; j < ncol; j++)
    {
      data_vec[i][j] = data(i, j);
    }
  }
  bool flag = write_h5(this->file_name, this->group_name, name, nrow, ncol, data_vec);

  return flag;

}


bool write_h5(const std::string file_name, const std::string group_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector)
{
  // Set up file name and dataset name
  const H5std_string FILE_NAME(file_name);
  const H5std_string GROUP_NAME(group_name);
  const H5std_string DATASET_NAME(dataset_name);

  // Convert 2-dim std::vector to 2-dim raw buffer(array)
  double data[ROW][COL];
  for (int j = 0; j < ROW; j++)
  {
    for (int i = 0; i < COL; i++)
    data[j][i] = data_vector[j][i];
  }

  try
  {

    // Shutdown auto-print of error information
    herr_t status = status = H5Eset_auto(H5E_DEFAULT, NULL, NULL);

    // Create a file(create, fail if it exists)
    H5Fcreate(FILE_NAME.c_str(), H5F_ACC_EXCL, H5P_DEFAULT, H5P_DEFAULT);
    
    // Create a file (must be an existing file)
    H5File file( FILE_NAME, H5F_ACC_RDWR );

    // Create a group (if exists, destroy it, and re-create another)
    Group group;
    status = H5Lget_info(file.getId(), GROUP_NAME.c_str(), NULL, H5P_DEFAULT);
    if (status==0)
    {
      std::cout << "The group already exists, open it." << std::endl;
      group = file.openGroup(GROUP_NAME);
    }
    else
    {
      std::cout << "The group doesn't exist, create one." << std::endl;
      group = file.createGroup(GROUP_NAME);
    }

  
    // Set up datatype and dataspace for the dataset to be store
    hsize_t dimsf[2];              // dataset dimensions
    dimsf[0] = ROW;
    dimsf[1] = COL;
    DataSpace dataspace(2, dimsf);
    IntType datatype( PredType::NATIVE_DOUBLE );
    datatype.setOrder( H5T_ORDER_LE );


    // Way 1 - Create a dataset within a 'group'
    status = H5Lget_info(group.getId(), DATASET_NAME.c_str(), NULL, H5P_DEFAULT);
    if (status == 0)
    {
      std::cout << "The dataset already exists, remove it and re-create another one." << std::endl;
      group.unlink(DATASET_NAME.c_str());
    }
    else
    {
      std::cout << "The dataset doesn't exist, create one." << std::endl;
    }
    DataSet dataset1 = group.createDataSet(DATASET_NAME, datatype, dataspace);


    // Way 2 - Create a new dataset within the 'file'
    //DataSet dataset2 = file.createDataSet( DATASET_NAME, datatype, dataspace );


    //Write the data to the dataset using default memory space, file space, and transfer properties.
    dataset1.write( data, PredType::NATIVE_DOUBLE );
    //dataset2.write( data, PredType::NATIVE_DOUBLE );

  } // File and group will be closed as their instances go out of scope

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
std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name)
{
  // Set up file name and dataset name
  const H5std_string FILE_NAME(file_name);
  const H5std_string DATASET_NAME(dataset_name);
  const H5std_string GROUP_NAME(group_name);

  try
  {
    // Open the specified file and the specified dataset in the file.
    H5File file( FILE_NAME, H5F_ACC_RDONLY );
    //DataSet dataset = file.openDataSet(DATASET_NAME)

    // Open a group 
    Group group = file.openGroup(GROUP_NAME);
    DataSet dataset = group.openDataSet(DATASET_NAME);

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


/* Class Instantiation */
DMPTrajectoryGenerator::DMPTrajectoryGenerator(std::string file_name, std::string group_name)
{

  /* Initialize the class using the original imitation trajectory, and use new pass points to deform it. */

  this->file_name = file_name;
  this->group_name = group_name;

  // Read from h5 file
  this->Yr_lrw = convert_to_matrixxd(read_h5(file_name, group_name, "Yr_lrw"));
  this->Yr_lew = convert_to_matrixxd(read_h5(file_name, group_name, "Yr_lew"));
  this->Yr_rew = convert_to_matrixxd(read_h5(file_name, group_name, "Yr_rew"));
  this->Yr_rw = convert_to_matrixxd(read_h5(file_name, group_name, "Yr_rw"));
  //std::cout << "debug: Yr size = " << this->Yr_lrw.rows() << " x " << this->Yr_lrw.cols() << std::endl;

  this->sIn = convert_to_matrixxd(read_h5(file_name, group_name, "sIn_lrw")); // common time profile
  //std::cout << "debug: sIn size = " << this->sIn.rows() << " x " << this->sIn.cols() << std::endl;

  this->lrw_goal = convert_to_matrixxd(read_h5(file_name, group_name, "lrw_goal"));
  this->lrw_start = convert_to_matrixxd(read_h5(file_name, group_name, "lrw_start"));
  this->lew_goal = convert_to_matrixxd(read_h5(file_name, group_name, "lew_goal"));
  this->lew_start = convert_to_matrixxd(read_h5(file_name, group_name, "lew_start"));
  this->rew_goal = convert_to_matrixxd(read_h5(file_name, group_name, "rew_goal"));
  this->rew_start = convert_to_matrixxd(read_h5(file_name, group_name, "rew_start"));
  this->rw_goal = convert_to_matrixxd(read_h5(file_name, group_name, "rw_goal"));
  this->rw_start = convert_to_matrixxd(read_h5(file_name, group_name, "rw_start"));
  //std::cout << "debug: goal size = " << this->lrw_goal.rows() << " x " << this->lrw_goal.cols() << std::endl;

  // orientation and glove angle trajectories
  this->l_wrist_quat_traj = convert_to_matrixxd(read_h5(file_name, group_name, "l_wrist_quat_resampled"));
  this->r_wrist_quat_traj = convert_to_matrixxd(read_h5(file_name, group_name, "r_wrist_quat_resampled"));
  this->l_glove_angle_traj = convert_to_matrixxd(read_h5(file_name, group_name, "l_glove_angle_resampled"));
  this->r_glove_angle_traj = convert_to_matrixxd(read_h5(file_name, group_name, "r_glove_angle_resampled"));

}

/* Reproduction Test: using original starts and goals */
void DMPTrajectoryGenerator::debug_repro()
{
  // Call to reproduce trajectories (with the original starts and goals)
  DMP_trajs result = generate_trajectories(this->lrw_goal, this->lrw_start,
                                           this->lew_goal, this->lew_start,
                                           this->rew_goal, this->rew_start,
                                           this->rw_goal, this->rw_start,
                                           50); //400); // complete reproduction, or interpolated

  // Save to h5 for analysis in MATLAB
  save_eigen_matrix_h5(result.y_lrw, "TEST_y_lrw");
  save_eigen_matrix_h5(result.y_lew, "TEST_y_lew");
  save_eigen_matrix_h5(result.y_rew, "TEST_y_rew");
  save_eigen_matrix_h5(result.y_rw, "TEST_y_rw");

  save_eigen_matrix_h5(result.y_re, "TEST_y_re");
  save_eigen_matrix_h5(result.y_lw, "TEST_y_lw");
  save_eigen_matrix_h5(result.y_le, "TEST_y_le");
  
  std::cout << "Generated trajectories written to .h5!" << std::endl;

}


/* Reproduction Test: using original starts and goals */
void DMPTrajectoryGenerator::debug_generalize()
{
  // Call to reproduce trajectories (with the original starts and goals)
  MatrixXd lrw_goal_offst(1, 3); lrw_goal_offst << 0.01, -0.01, 0.05;
  MatrixXd lrw_start_offst(1, 3); lrw_start_offst << -0.01, 0.01, 0.05;
  MatrixXd lew_goal_offst(1, 3); lew_goal_offst << 0.01, 0.0, -0.05;
  MatrixXd lew_start_offst(1, 3); lew_start_offst << 0.01, -0.01, -0.05;
  MatrixXd rew_goal_offst(1, 3); rew_goal_offst << 0.01, 0.02, 0.03;
  MatrixXd rew_start_offst(1, 3); rew_start_offst << 0.02, 0.01, 0.05;
  MatrixXd rw_goal_offst(1, 3); rw_goal_offst << 0.01, 0.0, 0.0;
  MatrixXd rw_start_offst(1, 3); rw_start_offst << -0.01, 0.0, 0.0;


  DMP_trajs result = generate_trajectories(this->lrw_goal+lrw_goal_offst, this->lrw_start+lrw_start_offst,
                                           this->lew_goal+lew_goal_offst, this->lew_start+lew_start_offst,
                                           this->rew_goal+rew_goal_offst, this->rew_start+rew_start_offst,
                                           this->rw_goal+rw_goal_offst, this->rw_start+rw_start_offst,
                                           400);

  // Save to h5 for analysis in MATLAB
  save_eigen_matrix_h5(result.y_lrw, "TEST_y_lrw");
  save_eigen_matrix_h5(result.y_lew, "TEST_y_lew");
  save_eigen_matrix_h5(result.y_rew, "TEST_y_rew");
  save_eigen_matrix_h5(result.y_rw, "TEST_y_rw");

  save_eigen_matrix_h5(result.y_re, "TEST_y_re");
  save_eigen_matrix_h5(result.y_lw, "TEST_y_lw");
  save_eigen_matrix_h5(result.y_le, "TEST_y_le");
  
  std::cout << "Generated trajectories written to .h5!" << std::endl;

}


MatrixXd DMPTrajectoryGenerator::convert_to_matrixxd(std::vector<std::vector<double>> input)
{

  unsigned int nrow = input.size();
  unsigned int ncol = input[0].size();

  MatrixXd output(nrow, ncol);

  // copy data row by row
  for (unsigned int i = 0; i < nrow; i++)
    output.row(i) = Map<MatrixXd>(input[i].data(), 1, ncol);

  return output;

}


/* Compute linear piece-wise function h(x) from given pass_points, including start, goal and via-points */
/*
void DMPTrajectoryGenerator::interpolate_hseq(MatrixXd pass_points)
{

  unsigned int num_datapoints = this->f_seq.rows();//f_seq.size();
  unsigned int JOINT_DOF = this->f_seq.cols(); //f_seq[0].size();


  // Interpolate h(x) sequence
  MatrixXd h_seq(this->f_seq.rows(), this->f_seq.cols());
  double t = 0;
  for(unsigned int n = 0; n < num_datapoints; n++)
  {
    // get current time
    t = this->time_range(n);

    // find pass-point interval
    unsigned int ID = this->pass_time.rows()-1; // upper bound; starts from 0
    for(unsigned int i = 0; i < this->pass_time.rows(); i++)
    {
      if(t < pass_time(i))
      {      
        ID = i; // upper bound
        break;
      }
    }
    
    double x0 = pass_time(ID-1);
    double x1 = pass_time(ID);
    double ratio = (t-x0) / (x1-x0);
    
    // for pos/angle data
    for(unsigned int pg_id = 0; pg_id < this->pos_and_glove_id.rows(); pg_id++)
    {

      unsigned int dof_id = (unsigned int) pos_and_glove_id(pg_id) - 1; // starts from z0!!!
      // interpolate
      double y0 = pass_points(ID-1, dof_id);
      double y1 = pass_points(ID, dof_id);
      h_seq(n, dof_id) = (y1-y0) * ratio + y0;
    }

    // for quaternion data
    for(unsigned int q_id = 0; q_id < 2; q_id++)
    {     
    
      unsigned int dof_id = (unsigned int) quat_id(q_id*4) - 1; // start from 0
      // slerp interpolate
      Quaterniond q0(pass_points(ID-1, dof_id), pass_points(ID-1, dof_id+1), pass_points(ID-1, dof_id+2), pass_points(ID-1, dof_id+3));
      Quaterniond q1(pass_points(ID, dof_id), pass_points(ID, dof_id+1), pass_points(ID, dof_id+2), pass_points(ID, dof_id+3));  
      Quaterniond qres = q0.slerp(ratio, q1);
      // store result
      //Vector<double, 1, 4> m;
      h_seq.block(n, dof_id, 1, 4) << qres.w(), qres.x(), qres.y(), qres.z(); //= Matrix // [w,x,y,z], in consistent with MATLAB quaternion data type
    }

  }


  this->h_seq = h_seq;

}
*/



MatrixXd DMPTrajectoryGenerator::interpolate_trajectory(MatrixXd original_trajectory, unsigned int num_datapoints)
{
  // Prep
  VectorXd t_ori(train_nbData);
  double dt_ori = 1.0 / (train_nbData-1);
  for (unsigned int i = 0; i < train_nbData; i++)
    t_ori[i] = 1 - dt_ori*i;
  //std::cout << "debug: t_ori = " << t_ori.transpose() << std::endl;
  
  VectorXd t_interp(num_datapoints);
  double dt_interp = 1.0 / (num_datapoints-1);
  for (unsigned int i = 0; i < num_datapoints; i++)
    t_interp[i] = 1 - dt_interp*i;
  //std::cout << "debug: t_interp = " << t_interp.transpose() << std::endl;
    
  // Linear interpolation
  MatrixXd interp_trajectory(original_trajectory.rows(), num_datapoints); // initialization
  for (unsigned int t = 0; t < num_datapoints-1; t++) // skip the last
  {
    for (unsigned int n = 0; n < train_nbData-1; n++) // skip the last
    {
      if (t_interp[t] <= t_ori[n] && t_interp[t] > t_ori[n+1]) // find the interval
      {
        double ratio = (t_interp[t] - t_ori[n]) / (t_ori[n+1] - t_ori[n]);
        interp_trajectory.block(0, t, 3, 1) = original_trajectory.block(0, n, 3, 1) 
                                    + ratio * (original_trajectory.block(0, n+1, 3, 1) - original_trajectory.block(0, n, 3, 1));
        break; // skip to interpolate next point
      }
    }
  }
  // set the last point
  interp_trajectory.block(0, num_datapoints-1, 3, 1) = original_trajectory.block(0, train_nbData-1, 3, 1);

  return interp_trajectory;

}



MatrixXd DMPTrajectoryGenerator::generate_trajectory(MatrixXd new_goal, MatrixXd new_start, MatrixXd Yr, unsigned int num_datapoints)
{
  // Input: new_goal, new_start - 1 x 3
  //        Yr - 3 x 400
  // Output: repro - 3 x 400

  // Prep
  Matrix<double, 3, 1> xStart;
  for (unsigned int i = 0; i < new_start.cols(); i++)
    xStart(i) = new_start(0, i);
  Matrix<double, 3, 1> xTar;
  for (unsigned int i = 0; i < new_goal.cols(); i++)
    xTar(i) = new_goal(0, i);    

  Matrix<double, 3, 1> x = xStart;
  Matrix<double, 3, 1> dx = Matrix<double, 3, 1>::Zero();
  Matrix<double, 3, 1> ddx = Matrix<double, 3, 1>::Zero();
  MatrixXd repro(3, this->train_nbData); // 3 x 400

  // Iterate to compute the trajectory
  for (unsigned int t = 0; t < this->train_nbData; t++)
  {
    // Compute acceleration, velocity and position
    ddx = kP * (xTar - x) - kV * dx + sIn(t) * Yr.col(t).cwiseProduct((xTar-xStart).cwiseAbs());
    dx = dx + ddx * this->dt;
    x = x + dx * this->dt;
    repro.col(t) = x;
  }

  // Interpolate to get expected number of path points
  MatrixXd repro_interp(3, num_datapoints);
  repro_interp = this->interpolate_trajectory(repro, num_datapoints);


  // return the result
  return repro_interp;
 
}



/* Input new starts and new goals, return generalized trajectories */
DMP_trajs DMPTrajectoryGenerator::generate_trajectories(MatrixXd lrw_new_goal, MatrixXd lrw_new_start,
                                                       MatrixXd lew_new_goal, MatrixXd lew_new_start,
                                                       MatrixXd rew_new_goal, MatrixXd rew_new_start,
                                                       MatrixXd rw_new_goal, MatrixXd rw_new_start,
                                                       unsigned int num_datapoints)
{
  // num_datapoints - expected number of path points of the generated trajectory
  // Output: a DMP_trajs struct, containing trajectories with the size of 3 x 50(num_datapoints)

  // Compute relative position trajectories
  MatrixXd y_lrw = this->generate_trajectory(lrw_new_goal, lrw_new_start, Yr_lrw, num_datapoints);
  MatrixXd y_lew = this->generate_trajectory(lew_new_goal, lew_new_start, Yr_lew, num_datapoints);
  MatrixXd y_rew = this->generate_trajectory(rew_new_goal, rew_new_start, Yr_rew, num_datapoints);
  MatrixXd y_rw = this->generate_trajectory(rw_new_goal, rw_new_start, Yr_rw, num_datapoints);
  
  MatrixXd y_re = y_rw + y_rew;
  MatrixXd y_lw = y_rw + y_lrw;
  MatrixXd y_le = y_lw + y_lew;

  //std::cout << "debug: new traj size = " << y_lrw.rows() << " x " << y_lrw.cols() << std::endl;


  // Return the result
  DMP_trajs result;
  result.y_lrw = y_lrw;
  result.y_lew = y_lew;
  result.y_rew = y_rew;
  result.y_rw = y_rw;

  result.y_re = y_re;
  result.y_lw = y_lw;
  result.y_le = y_le;
  
  return result;
  
}



/*
int main(int argc, char **argv)
{


  std::string file_name = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/test_imi_data_YuMi.h5";
  std::string group_name = "fengren_1";

  DMPTrajectoryGenerator DMPTrajGen(file_name, group_name);

  // debug: Reproduction
  DMPTrajGen.debug_repro();
  //DMPTrajGen.debug_generalize();


  return 0;

}

*/

