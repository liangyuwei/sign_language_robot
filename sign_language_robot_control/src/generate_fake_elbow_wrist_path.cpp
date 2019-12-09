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




int main(void)
{

  // Set via-points
  const int num_viapoints = 8;
  std::vector<Vector3d> elbow_pos(num_viapoints);
  std::vector<Vector3d> wrist_pos(num_viapoints);
  std::vector<Quaterniond> wrist_ori(num_viapoints); // use quaternion(double) for specifying via-points(can be read directly from RViz), w,x,y,z
  std::vector<Matrix<double, 12, 1>> finger_pos(num_viapoints);


  /* For Eigen, Quaterniond is (w,x,y,z); For tf, quaternion is [x,y,z,w] */
  /** Left arm + hand **
  wrist_pos[0] = Vector3d(0.495918, 0.119251, 0.371701);
  wrist_ori[0] = Quaterniond(-0.079195, 0.600824, -0.465971, 0.644678).normalized(); // positive unit quaternion is unique and has no singularity
  elbow_pos[0] = Vector3d(0.301041, 0.422758, 0.64508);
  finger_pos[0] << 0.0, 0.0, -1.0, -2.0, -1.0, -2.0, -1.0, -2.0, 0.38, 0.0, -0.71, -0.52; 


  wrist_pos[1] = Vector3d(0.485239, 0.174573, 0.385977);
  wrist_ori[1] = Quaterniond(-0.160029, 0.710388, -0.363795, 0.580855).normalized();
  elbow_pos[1] = Vector3d(0.254965, 0.484654, 0.674516);
  finger_pos[1] << 0.0, 0.0, -1.0, -2.0, -1.0, -2.0, -1.0, -2.0, 0.38, 0.0, -0.71, -0.52; 


  wrist_pos[2] = Vector3d(0.578741, 0.191479, 0.36143);
  wrist_ori[2] = Quaterniond(-0.185247, 0.774455, -0.285139, 0.533478).normalized();
  elbow_pos[2] = Vector3d(0.285758, 0.469814, 0.636463); 
  finger_pos[2] << 0.0, 0.0, -1.0, -2.0, -1.0, -2.0, -1.0, -2.0, 0.38, 0.0, -0.71, -0.52; 


  wrist_pos[3] = Vector3d(0.647241, 0.114291, 0.353222);
  wrist_ori[3] = Quaterniond(-0.158918, 0.793782, -0.288242, 0.51144).normalized(); 
  elbow_pos[3] = Vector3d(0.328502, 0.394175, 0.601958);
  finger_pos[3] << 0.0, 0.0, -1.0, -2.0, -1.0, -2.0, -1.0, -2.0, 0.38, 0.0, -0.71, -0.52; 


  wrist_pos[4] = Vector3d(0.622429, 0.0230071, 0.357378);
  wrist_ori[4] = Quaterniond(-0.112641, 0.795489, -0.366878, 0.468946).normalized(); 
  elbow_pos[4] = Vector3d(0.336637, 0.336568, 0.60664);
  finger_pos[4] << 0.0, 0.0, -1.0, -2.0, -1.0, -2.0, -1.0, -2.0, 0.38, 0.0, -0.71, -0.52; 


  wrist_pos[5] = Vector3d(0.548205, -0.035932, 0.3451617);
  wrist_ori[5] = Quaterniond(-0.0641356, 0.741666, -0.452042, 0.491402).normalized(); 
  elbow_pos[5] = Vector3d(0.331544, 0.308633, 0.623776);
  finger_pos[5] << 0.0, 0.0, -1.0, -2.0, -1.0, -2.0, -1.0, -2.0, 0.38, 0.0, -0.71, -0.52; 


  wrist_pos[6] = Vector3d(0.460226, -0.0169357, 0.347799);
  wrist_ori[6] = Quaterniond(-0.0971217, 0.714394, -0.447286, 0.529286).normalized(); 
  elbow_pos[6] = Vector3d(0.314848, 0.336756, 0.655215);
  finger_pos[6] << 0.0, 0.0, -1.0, -2.0, -1.0, -2.0, -1.0, -2.0, 0.38, 0.0, -0.71, -0.52; 

  wrist_pos[7] = Vector3d(0.39143, 0.112259, 0.355113);
  wrist_ori[7] = Quaterniond(0.0497201, 0.65778, -0.587034, 0.469302).normalized(); 
  elbow_pos[7] = Vector3d(0.28595,0.434396,0.664474);
  finger_pos[7] << 0.0, 0.0, -1.0, -2.0, -1.0, -2.0, -1.0, -2.0, 0.38, 0.0, -0.71, -0.52; 
  */
  
  /** Right arm + hand **/
  wrist_pos[0] = Vector3d(0.459286, -0.17724, 0.338785);
  wrist_ori[0] = Quaterniond(0.0671244, 0.702087, 0.458296, 0.540864).normalized(); // positive unit quaternion is unique and has no singularity
  elbow_pos[0] = Vector3d(0.277208, -0.47985, 0.642771);
  finger_pos[0] << 0.0, 0.0, -1.45, -1.63, -1.5, -1.58, -1.51, -1.64, -0.72, 0.17, -0.29, -0.82;

  wrist_pos[1] = Vector3d(0.515995, -0.131873, 0.356376);
  wrist_ori[1] = Quaterniond(-0.00511485, 0.676775, 0.53541, 0.505258).normalized();
  elbow_pos[1] = Vector3d(0.32, -0.42, 0.60);
  finger_pos[1] << 0.0, 0.0, -1.45, -1.63, -1.5, -1.58, -1.51, -1.64, -0.72, 0.17, -0.29, -0.82;

  wrist_pos[2] = Vector3d(0.620217, -0.102566, 0.329752);
  wrist_ori[2] = Quaterniond(0.0732544, 0.717278, 0.459966, 0.518245).normalized();
  elbow_pos[2] = Vector3d(0.344016, -0.36957, 0.570032); 
  finger_pos[2] << 0.0, 0.0, -1.45, -1.63, -1.5, -1.58, -1.51, -1.64, -0.72, 0.17, -0.29, -0.82;

  wrist_pos[3] = Vector3d(0.682532, -0.182324, 0.329345);
  wrist_ori[3] = Quaterniond(0.0863313, 0.705692, 0.437065, 0.550926).normalized(); 
  elbow_pos[3] = Vector3d(0.35206, -0.376849, 0.527919);
  finger_pos[3] << 0.0, 0.0, -1.45, -1.63, -1.5, -1.58, -1.51, -1.64, -0.72, 0.17, -0.29, -0.82;

  wrist_pos[4] = Vector3d(0.638973, -0.281634, 0.326374);
  wrist_ori[4] = Quaterniond(0.0264457, 0.735369, 0.462025, 0.495041).normalized(); 
  elbow_pos[4] = Vector3d(0.32372, -0.456209, 0.550935);
  finger_pos[4] << 0.0, 0.0, -1.45, -1.63, -1.5, -1.58, -1.51, -1.64, -0.72, 0.17, -0.29, -0.82;

  wrist_pos[5] = Vector3d(0.547948, -0.33214, 0.300531);
  wrist_ori[5] = Quaterniond(-0.0378739, 0.748082, 0.480499, 0.456136).normalized(); 
  elbow_pos[5] = Vector3d(0.285922, -0.519192, 0.569029);
  finger_pos[5] << 0.0, 0.0, -1.45, -1.63, -1.5, -1.58, -1.51, -1.64, -0.72, 0.17, -0.29, -0.82;

  wrist_pos[6] = Vector3d(0.437774, -0.315823, 0.298068);
  wrist_ori[6] = Quaterniond(-0.112386, 0.777825, 0.507696, 0.35299).normalized(); 
  elbow_pos[6] = Vector3d(0.250023, -0.551359, 0.601584);
  finger_pos[6] << 0.0, 0.0, -1.45, -1.63, -1.5, -1.58, -1.51, -1.64, -0.72, 0.17, -0.29, -0.82;

  wrist_pos[7] = Vector3d(0.376681, -0.181348, 0.316947);
  wrist_ori[7] = Quaterniond(-0.0913673, 0.64774, 0.568812, 0.498535).normalized(); 
  elbow_pos[7] = Vector3d(0.281622, -0.482747, 0.631438);
  finger_pos[7] << 0.0, 0.0, -1.45, -1.63, -1.5, -1.58, -1.51, -1.64, -0.72, 0.17, -0.29, -0.82;


  // Do interpolation on path points
  const int num_interp_points = 5; // with num_interp_points+1 segments
  std::vector<std::vector<double>> path; // use push_back() to record data


  Matrix<double, 27, 1> path_point; // store the combined vector
  Vector3d elbow_pos_last, elbow_pos_next, elbow_pos_tmp; // for elbow pos
  Vector3d wrist_pos_last, wrist_pos_next, wrist_pos_tmp; // for wrist pos
  Quaterniond wrist_ori_last, wrist_ori_next, wrist_ori_tmp; // for wrist ori(quaternion)
  Matrix<double, 9, 1> wrist_rot_tmp; // for wrist ori(rotation matrix)
  Matrix<double, 12, 1> finger_pos_last, finger_pos_next, finger_pos_tmp; // for one hand pose


  unsigned int path_point_len = wrist_pos_tmp.size() + wrist_rot_tmp.size() + elbow_pos_tmp.size() + finger_pos_tmp.size();


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
    finger_pos_last = finger_pos[i];
    finger_pos_next = finger_pos[i+1];
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
      // finger pos
      finger_pos_tmp = (finger_pos_next - finger_pos_last) * j / (num_interp_points+1) + finger_pos_last;
      // combine and append
      path_point.block<3, 1>(0, 0) = wrist_pos_tmp;
      path_point.block<9, 1>(3, 0) = wrist_rot_tmp; // flatten matrix
      path_point.block<3, 1>(12, 0) = elbow_pos_tmp;
      path_point.block<12, 1>(15, 0) = finger_pos_tmp;
      // From Eigen Matrix to raw array
      double *wrist_pos_tmp_arr = new double[wrist_pos_tmp.size()];
      double *wrist_rot_tmp_arr = new double[wrist_rot_tmp.size()];
      double *elbow_pos_tmp_arr = new double[elbow_pos_tmp.size()];
      double *finger_pos_tmp_arr = new double[finger_pos_tmp.size()];
      double *path_point_arr = new double[path_point_len];
      Map<Vector3d>(wrist_pos_tmp_arr, wrist_pos_tmp.size()) = wrist_pos_tmp; // Map can be lvalue
      Map<Matrix<double, 9, 1>>(wrist_rot_tmp_arr, wrist_rot_tmp.size()) = wrist_rot_tmp;
      Map<Vector3d>(elbow_pos_tmp_arr, elbow_pos_tmp.size()) = elbow_pos_tmp;
      Map<Matrix<double, 12, 1>>(finger_pos_tmp_arr, finger_pos_tmp.size()) = finger_pos_tmp;
      Map<Matrix<double, Dynamic, 1>>(path_point_arr, path_point.size()) = path_point;
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
      std::cout << std::endl << "Finger pos: " << std::endl;
      for (int i = 0; i < finger_pos_tmp.size(); ++i)  std::cout << finger_pos_tmp_arr[i] << " ";
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
  Matrix3d wrist_rot_from_quat = wrist_ori_tmp.matrix().transpose(); // transpose of ColMajor matrix
  //Matrix<double, 9, 1> wrist_rot_tmp = Map<Matrix<double, 9, 1>>(wrist_rot_from_quat.data());
  wrist_rot_tmp = Map<Matrix<double, 9, 1>>(wrist_rot_from_quat.data());
  // positions
  wrist_pos_tmp = wrist_pos_next;
  elbow_pos_tmp = elbow_pos_next;
  // finger pos
  finger_pos_tmp = finger_pos_next;
  // combine and append
  path_point.block<3, 1>(0, 0) = wrist_pos_tmp;
  path_point.block<9, 1>(3, 0) = wrist_rot_tmp; // flatten matrix
  path_point.block<3, 1>(12, 0) = elbow_pos_tmp;
  path_point.block<12, 1>(15, 0) = finger_pos_tmp;
  // From Eigen Matrix to raw array
  double *wrist_pos_tmp_arr = new double[wrist_pos_tmp.size()];
  double *wrist_rot_tmp_arr = new double[wrist_rot_tmp.size()];
  double *elbow_pos_tmp_arr = new double[elbow_pos_tmp.size()];
  double *finger_pos_tmp_arr = new double[finger_pos_tmp.size()];
  double *path_point_arr = new double[path_point_len];
  Map<Vector3d>(wrist_pos_tmp_arr, wrist_pos_tmp.size()) = wrist_pos_tmp; // Map can be lvalue
  Map<Matrix<double, 9, 1>>(wrist_rot_tmp_arr, wrist_rot_tmp.size()) = wrist_rot_tmp;
  Map<Vector3d>(elbow_pos_tmp_arr, elbow_pos_tmp.size()) = elbow_pos_tmp;
  Map<Matrix<double, 12, 1>>(finger_pos_tmp_arr, finger_pos_tmp.size()) = finger_pos_tmp;
  Map<Matrix<double, Dynamic, 1>>(path_point_arr, path_point.size()) = path_point;
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
  std::cout << std::endl << "Finger pos: " << std::endl;
  for (int i = 0; i < finger_pos_tmp.size(); ++i)  std::cout << finger_pos_tmp_arr[i] << " ";
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
  std::string file_name = "fake_elbow_wrist_paths.h5";
  std::string dataset_name = "fake_path_right_1";
  int ROW =  path.size();
  int COL = path_point_len;
  bool result = write_h5(file_name, dataset_name, ROW, COL, path);
  if (result)
    std::cout << "Storage successful!" << std::endl;



  return 0;

}
