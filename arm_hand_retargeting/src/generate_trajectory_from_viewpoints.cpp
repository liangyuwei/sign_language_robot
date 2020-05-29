#include <iostream>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> // for Map()

// For file write and read
#include <string>
#include "H5Cpp.h"
//#include "H5Location.h"


#include "generate_trajectory_from_viewpoints.h"


using namespace std;
using namespace Eigen;
using namespace H5;


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
TrajectoryGenerator::TrajectoryGenerator(std::string file_name, std::string group_name)
{

  this->file_name = file_name;
  this->group_name = group_name;

  // Read from h5 file
  this->f_seq = convert_to_matrixxd(read_h5(file_name, group_name, "f_seq"));
  this->kp_list = convert_to_matrixxd(read_h5(file_name, group_name, "keypoint_list")); 
  this->pos_and_glove_id = convert_to_matrixxd(read_h5(file_name, group_name, "pos_and_glove_id")); 
  this->quat_id = convert_to_matrixxd(read_h5(file_name, group_name, "quat_id")); 
  this->pass_time = convert_to_matrixxd(read_h5(file_name, group_name, "pass_time")); 
  this->time_range = convert_to_matrixxd(read_h5(file_name, group_name, "time_range")); 
  this->pass_points = convert_to_matrixxd(read_h5(file_name, group_name, "pass_points"));

}

void TrajectoryGenerator::debug_print()
{

  // plot 
  MatrixXd tmp_seq = this->h_seq; //this->f_seq;
  
  for(unsigned int r = 0; r < tmp_seq.rows(); r++)
  {
    for(unsigned int c = 0; c < tmp_seq.cols(); c++)
      cout << tmp_seq(r, c) << ", ";
    cout << endl;
  }

  cout << "f_seq size is " << tmp_seq.rows() << " x " << tmp_seq.cols() << " ." << endl;


}


void TrajectoryGenerator::debug_store()
{

  
  /* test interpolate_hseq code */
  cout << "Interpolate hseq..." << endl;
  this->interpolate_hseq(this->pass_points);
  // convert to 2D std::vector
  cout << "Convert to 2D std::vector" << endl;
  unsigned int nrow = this->h_seq.rows();
  unsigned int ncol = this->h_seq.cols();
  std::vector<std::vector<double> > h_seq_vec(nrow, std::vector<double>(ncol));
  for (unsigned int i = 0; i < nrow; i++)
  {
    for (unsigned int j = 0; j < ncol; j++)
    {
      h_seq_vec[i][j] = this->h_seq(i, j);
    }
  }
  write_h5(this->file_name, this->group_name, "tmp_hseq", nrow, ncol, h_seq_vec);
  cout << "h_seq result exported to h5 file!" << endl;
  
  
  
  /* test compute yseq code */
  cout << "Computing yseq..." << endl;
  this->compute_yseq();
  std::vector<std::vector<double> > y_seq_vec(nrow, std::vector<double>(ncol));
  for (unsigned int i = 0; i < nrow; i++)
  {
    for (unsigned int j = 0; j < ncol; j++)
    {
      y_seq_vec[i][j] = this->y_seq(i, j);
    }
  }
  write_h5(this->file_name, this->group_name, "tmp_yseq", nrow, ncol, y_seq_vec);
  cout << "y_seq result exported to h5 file!" << endl;
  
  
}


void TrajectoryGenerator::debug_test_noise()
{

  /* test interpolate_hseq code */
  cout << "Interpolate hseq..." << endl;
  MatrixXd pass_points_noisy = convert_to_matrixxd(read_h5(file_name, group_name, "pass_points_noise"));  
  this->interpolate_hseq(pass_points_noisy);
  // convert to 2D std::vector
  cout << "Convert to 2D std::vector" << endl;
  unsigned int nrow = this->h_seq.rows();
  unsigned int ncol = this->h_seq.cols();
  std::vector<std::vector<double> > h_seq_vec(nrow, std::vector<double>(ncol));
  for (unsigned int i = 0; i < nrow; i++)
  {
    for (unsigned int j = 0; j < ncol; j++)
    {
      h_seq_vec[i][j] = this->h_seq(i, j);
    }
  }
  write_h5(this->file_name, this->group_name, "tmp_hseq_noisy", nrow, ncol, h_seq_vec);
  cout << "h_seq result exported to h5 file!" << endl;
  
  
  
  /* test compute yseq code */
  cout << "Computing yseq..." << endl;
  this->compute_yseq();
  std::vector<std::vector<double> > y_seq_vec(nrow, std::vector<double>(ncol));
  for (unsigned int i = 0; i < nrow; i++)
  {
    for (unsigned int j = 0; j < ncol; j++)
    {
      y_seq_vec[i][j] = this->y_seq(i, j);
    }
  }
  write_h5(this->file_name, this->group_name, "tmp_yseq_noisy", nrow, ncol, y_seq_vec);
  cout << "y_seq result exported to h5 file!" << endl;
  
  
}



MatrixXd TrajectoryGenerator::convert_to_matrixxd(std::vector<std::vector<double>> input)
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
void TrajectoryGenerator::interpolate_hseq(MatrixXd pass_points)
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



/* Comput y(x) using this->h_seq and this->f_seq */
void TrajectoryGenerator::compute_yseq()
{
  
  // Preparation
  unsigned int num_datapoints = this->f_seq.rows();
  MatrixXd y_seq(this->f_seq.rows(), this->f_seq.cols());
  
  // For pos/angle data
  for (unsigned int pg_id = 0; pg_id < this->pos_and_glove_id.rows(); pg_id++)
  {
    unsigned int dof_id = (unsigned int) this->pos_and_glove_id(pg_id) - 1; 
    // simply add up
    y_seq.col(dof_id) = this->h_seq.col(dof_id) + this->f_seq.col(dof_id);
  }
      
  // For quaternion data
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    for (unsigned int q_id = 0; q_id < 2; q_id++)   
    {
      unsigned int dof_id = (unsigned int) quat_id(q_id*4) - 1; // start from 0
      // get quaternion
      Quaterniond hq(h_seq(n, dof_id), h_seq(n, dof_id+1), h_seq(n, dof_id+2), h_seq(n, dof_id+3));  
      Quaterniond fq(f_seq(n, dof_id), f_seq(n, dof_id+1), f_seq(n, dof_id+2), f_seq(n, dof_id+3)); 
      
      Quaterniond yq = hq * fq;
      // store result
      y_seq.block(n, dof_id, 1, 4) << yq.w(), yq.x(), yq.y(), yq.z();
    
    }
  
  }
  
  this->y_seq = y_seq;

}


MatrixXd TrajectoryGenerator::generate_trajectory_from_passpoints(MatrixXd pass_points)
{

  // Interpolate through pass_points to make up h(x)
  this->interpolate_hseq(pass_points);

  // Call compute_yseq
  this->compute_yseq();
  
  // Return the result
  return this->y_seq;
  
}


/*
int main(int argc, char **argv)
{

  TrajectoryGenerator TrajGen("test_imi_data_YuMi.h5", "fengren_1");

  // debug 1
  //TrajGen.debug_print();

  // debug 2: h_seq
  //cout << "Calling debug_store function" << endl;
  //TrajGen.debug_store();

  // debug 3: y_seq
  cout << "Calling debug_test_noise function" << endl;
  TrajGen.debug_test_noise();

  return 0;

}
*/


