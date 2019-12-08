// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> // for Map()


// For file write and read
#include <vector>
#include <string>
#include "H5Cpp.h"


// Others
#include <iostream>


using namespace Eigen;
using namespace H5;

// Copied directly from use_hdf5.cpp
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
    H5File file( FILE_NAME, H5F_ACC_TRUNC );
      
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




int main(void)
{

  // Set via-points
  const int num_viapoints = 3;
  std::vector<Vector3d> elbow_pos(num_viapoints);
  std::vector<Vector3d> wrist_pos(num_viapoints);
  std::vector<Quaterniond> wrist_ori(num_viapoints); // use quaternion(double) for specifying via-points(can be read directly from RViz), w,x,y,z

  /* For Eigen, Quaterniond is (w,x,y,z); For tf, quaternion is [x,y,z,w] */
  wrist_pos[0] = Vector3d(0.59467, 0.269, 0.3562);
  wrist_ori[0] = Quaterniond(0.4132, 0.58313, -0.5671, -0.4094).normalized(); // positive unit quaternion is unique and has no singularity
  elbow_pos[0] = Vector3d(0.338419, 0.414076, 0.551237);
  
  wrist_pos[1] = Vector3d(0.426576, 0.306569, 0.389701);
  wrist_ori[1] = Quaterniond(0.5165, 0.50248, -0.45483, -0.52332).normalized();
  elbow_pos[1] = Vector3d(0.240105, 0.52223, 0.659454);

  wrist_pos[2] = Vector3d(0.307, 0.2979, 0.2708);
  wrist_ori[2] = Quaterniond(0.5383995, 0.54271975, -0.44683197, -0.46467457).normalized();
  elbow_pos[2] = Vector3d(0.24467, 0.558, 0.6012); 


  // Do interpolation on path points
  const int num_interp_points = 10; // with num_interp_points+1 segments
  std::vector<std::vector<double>> path; // use push_back() to record data

  Matrix<double, 15, 1> path_point; // store the combined vector
  Vector3d elbow_pos_last, elbow_pos_next, elbow_pos_tmp; // for elbow pos
  Vector3d wrist_pos_last, wrist_pos_next, wrist_pos_tmp; // for wrist pos
  Quaterniond wrist_ori_last, wrist_ori_next, wrist_ori_tmp; // for wrist ori(quaternion)
  Matrix<double, 9, 1> wrist_rot_tmp; // for wrist ori(rotation matrix)

  unsigned int path_point_len = wrist_pos_tmp.size() + wrist_rot_tmp.size() + elbow_pos_tmp.size();

  int count = 0;


  for (int i = 0; i < num_viapoints - 1; ++i)
  {
    // Set `last` and `next` points for interpolation
    wrist_pos_last = wrist_pos[i];
    wrist_pos_next = wrist_pos[i+1];
    wrist_ori_last = wrist_ori[i];
    wrist_ori_next = wrist_ori[i+1];
    elbow_pos_last = elbow_pos[i];
    elbow_pos_next = elbow_pos[i+1];
    for (int j = 0; j <= num_interp_points; ++j) // including the first point(`last`)
    {
      // quaternion
      wrist_ori_tmp = wrist_ori_last.slerp( (double)j / (num_interp_points+1), wrist_ori_next);
      Matrix3d wrist_rot_from_quat = wrist_ori_tmp.matrix().transpose(); // transpose of ColMajor matrix
      //Matrix<double, 9, 1> wrist_rot_tmp = Map<Matrix<double, 9, 1>>(wrist_rot_from_quat.data());
      wrist_rot_tmp = Map<Matrix<double, 9, 1>>(wrist_rot_from_quat.data());
      // positions
      wrist_pos_tmp = (wrist_pos_next - wrist_pos_last) * j / (num_interp_points+1) + wrist_pos_last;
      elbow_pos_tmp = (elbow_pos_next - elbow_pos_last) * j / (num_interp_points+1) + elbow_pos_last;
      // combine and append
      path_point.block<3, 1>(0, 0) = wrist_pos_tmp;
      path_point.block<9, 1>(3, 0) = wrist_rot_tmp; // flatten matrix
      path_point.block<3, 1>(12, 0) = elbow_pos_tmp;
      // From Eigen Matrix to raw array
      double *wrist_pos_tmp_arr = new double[wrist_pos_tmp.size()];
      double *wrist_rot_tmp_arr = new double[wrist_rot_tmp.size()];
      double *elbow_pos_tmp_arr = new double[elbow_pos_tmp.size()];
      double *path_point_arr = new double[path_point_len];
      Map<Vector3d>(wrist_pos_tmp_arr, wrist_pos_tmp.size()) = wrist_pos_tmp; // Map can be lvalue
      Map<Matrix<double, 9, 1>>(wrist_rot_tmp_arr, wrist_rot_tmp.size()) = wrist_rot_tmp;
      Map<Vector3d>(elbow_pos_tmp_arr, elbow_pos_tmp.size()) = elbow_pos_tmp;
      Map<Matrix<double, 15, 1>>(path_point_arr, path_point.size()) = path_point;
      // convert to std::vector and store it
      std::vector<double> path_point_vec(path_point_len);
      for (int i = 0; i < path_point_vec.size(); ++i)  path_point_vec[i] = path_point_arr[i];  
      path.push_back(path_point_vec);
      // show results
      std::cout << "===== Point " << count + 1 << " =====" << std::endl;
      count++;
      std::cout << "Ratio is: " << j * 1.0 / (num_interp_points+1) << " (check the division on integer type)" << std::endl;
      std::cout << "Quaternion(x,y,z,w) is: " << wrist_ori_tmp.x() << ", "
                                              << wrist_ori_tmp.y() << ", "
                                              << wrist_ori_tmp.z() << ", "
                                              << wrist_ori_tmp.w() << "; " << std::endl;
      std::cout << "Flatten rotation matrix is: ";
      for (int i = 0; i < wrist_rot_tmp.size(); ++i)  std::cout << wrist_rot_tmp_arr[i] << " ";
      std::cout << std::endl << "Elbow pos: " << std::endl;
      for (int i = 0; i < elbow_pos_tmp.size(); ++i)  std::cout << elbow_pos_tmp_arr[i] << " ";
      std::cout << std::endl << "Wrist pos: " << std::endl;
      for (int i = 0; i < wrist_pos_tmp.size(); ++i)  std::cout << wrist_pos_tmp_arr[i] << " ";
      std::cout << std::endl << "Combined path point(array) is: ";
      for (int i = 0; i < path_point_len; ++i)  std::cout << path_point_arr[i] << " ";
      std::cout << std::endl << "Combined path point(std::vector) is: ";
      for (int i = 0; i < path_point_len; ++i)  std::cout << path_point_vec[i] << " ";
      std::cout << std::endl;

    } 

  }
  // Add the last via point(the last `next` point) here
  // quaternion
  wrist_ori_tmp = wrist_ori_next;
  Matrix3d wrist_rot_from_quat = wrist_ori_tmp.matrix().transpose();//.toRotationMatrix();
  //Matrix<double, 9, 1> wrist_rot_tmp = Map<Matrix<double, 9, 1>>(wrist_rot_from_quat.data());
  wrist_rot_tmp = Map<Matrix<double, 9, 1>>(wrist_rot_from_quat.data());
  // positions
  wrist_pos_tmp = wrist_pos_next;
  elbow_pos_tmp = elbow_pos_next;
  // combine and append
  path_point.block<3, 1>(0, 0) = wrist_pos_tmp;
  path_point.block<9, 1>(3, 0) = wrist_rot_tmp; // flatten matrix
  path_point.block<3, 1>(12, 0) = elbow_pos_tmp;
  // From Eigen Matrix to raw array
  double *wrist_pos_tmp_arr = new double[wrist_pos_tmp.size()];
  double *wrist_rot_tmp_arr = new double[wrist_rot_tmp.size()];
  double *elbow_pos_tmp_arr = new double[elbow_pos_tmp.size()];
  double *path_point_arr = new double[path_point_len];
  Map<Vector3d>(wrist_pos_tmp_arr, wrist_pos_tmp.size()) = wrist_pos_tmp; // Map can be used as lvalue
  Map<Matrix<double, 9, 1>>(wrist_rot_tmp_arr, wrist_rot_tmp.size()) = wrist_rot_tmp;
  Map<Vector3d>(elbow_pos_tmp_arr, elbow_pos_tmp.size()) = elbow_pos_tmp;
  Map<Matrix<double, 15, 1>>(path_point_arr, path_point.size()) = path_point;
  // convert to std::vector and store it
  std::vector<double> path_point_vec(path_point_len);
  for (int i = 0; i < path_point_vec.size(); ++i)  path_point_vec[i] = path_point_arr[i];  
  path.push_back(path_point_vec);
  // show results
  std::cout << "===== Point " << count + 1 << " =====" << std::endl;
  count++;
  std::cout << "Quaternion(x,y,z,w) is: " << wrist_ori_tmp.x() << ", "
                                          << wrist_ori_tmp.y() << ", "
                                          << wrist_ori_tmp.z() << ", "
                                          << wrist_ori_tmp.w() << "; " << std::endl;
  std::cout << "Flatten rotation matrix is: ";
  for (int i = 0; i < wrist_rot_tmp.size(); ++i)  std::cout << wrist_rot_tmp_arr[i] << " ";
  std::cout << std::endl << "Elbow pos: " << std::endl;
  for (int i = 0; i < elbow_pos_tmp.size(); ++i)  std::cout << elbow_pos_tmp_arr[i] << " ";
  std::cout << std::endl << "Wrist pos: " << std::endl;
  for (int i = 0; i < wrist_pos_tmp.size(); ++i)  std::cout << wrist_pos_tmp_arr[i] << " ";
  std::cout << std::endl << "Combined path point(array) is: ";
  for (int i = 0; i < path_point_len; ++i)  std::cout << path_point_arr[i] << " ";
  std::cout << std::endl << "Combined path point(std::vector) is: ";
  for (int i = 0; i < path_point_len; ++i)  std::cout << path_point_vec[i] << " ";
  std::cout << std::endl;


  // Display data in `path` variable
  std::cout << "====== Display `path` results =====" << std::endl
            << "Path size is: " << path.size() << std::endl
            << "Path data is: " << std::endl;
  for (auto it = path.cbegin(); it != path.cend(); ++it)
  {
    for (auto ip = (*it).cbegin(); ip != (*it).cend(); ++ip)
      std::cout << *ip << " ";
    std::cout << std::endl;
  }
  


  // Write to h5 file
  std::string file_name = "fake_elbow_wrist_path_1.h5";
  std::string dataset_name = "fake_path_1";
  int ROW =  path.size();
  int COL = path_point_len;
  bool result = write_h5(file_name, dataset_name, ROW, COL, path);
  if (result)
    std::cout << "Storage successful!" << std::endl;



  return 0;

}
