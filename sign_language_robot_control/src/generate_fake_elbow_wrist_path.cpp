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


int main(void)
{

  // Set via-points
  const int num_viapoints = 3;
  std::vector<Vector3d> elbow_pos(num_viapoints);
  std::vector<Vector3d> wrist_pos(num_viapoints);
  std::vector<Quaterniond> wrist_ori(num_viapoints); // use quaternion(double) for specifying via-points(can be read directly from RViz), w,x,y,z

  /* For Eigen, Quaterniond is w,x,y,z; For tf, quaternion is [x,y,z,w] */
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
  const int num_interp_points = 2;//10; // with num_interp_points+1 segments
  std::vector<Matrix<double, 15, 1>> path(num_interp_points * (num_viapoints - 1) + num_viapoints); // start point -- 20 in-between -- via point -- 20 in-between -- final point  
  Matrix<double, 15, 1> path_point;
  Vector3d elbow_pos_last, elbow_pos_next, elbow_pos_tmp, wrist_pos_last, wrist_pos_next, wrist_pos_tmp;
  Quaterniond wrist_ori_last, wrist_ori_next, wrist_ori_tmp;
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
      wrist_ori_tmp = wrist_ori_last.slerp( j / (num_interp_points+1), wrist_ori_next);
      Matrix<double, 9, 1> wrist_rot_tmp = wrist_ori_tmp.toRotationMatrix().resize(9, 1); // row-major
      std::cout << "Point " << count + 1 << ": " << std::endl;
      std::cout << "Quaternion(x,y,z,w) is: " << wrist_ori_tmp.x() << ", "
                                              << wrist_ori_tmp.y() << ", "
                                              << wrist_ori_tmp.z() << ", "
                                              << wrist_ori_tmp.w() << "; " << std::endl;
      std::cout << "Flatten rotation matrix is: " << wrist_rot_tmp << std::endl;
      count++;
      // positions
      wrist_pos_tmp = (wrist_pos_next - wrist_pos_last) * j / (num_interp_points+1) + wrist_pos_last;
      elbow_pos_tmp = (elbow_pos_next - elbow_pos_last) * j / (num_interp_points+1) + elbow_pos_last;
      // combine and append
      path_point.block<3, 1>(0, 0) = wrist_pos_tmp;
      path_point.block<9, 1>(3, 0) = wrist_rot_tmp; // flatten matrix
      path_point.block<3, 1>(12, 0) = elbow_pos_tmp;
      path.push_back(path_point);
    } 
    
  }
  // Add the last via point(the last `next` point) here
  // quaternion
  wrist_ori_tmp = wrist_ori_next;
  Matrix<double, 9, 1> wrist_rot_tmp = Map<Matrix<double, 9, 1, RowMajor>>(wrist_ori_tmp.toRotationMatrix().data(), wrist_ori_tmp.toRotationMatrix().size()); // row-major
  std::cout << "Point " << count + 1 << ": " << std::endl;
  std::cout << "Quaternion(x,y,z,w) is: " << wrist_ori_tmp.x() << ", "
                                          << wrist_ori_tmp.y() << ", "
                                          << wrist_ori_tmp.z() << ", "
                                          << wrist_ori_tmp.w() << "; " << std::endl;
  std::cout << "Flatten rotation matrix is: " << wrist_rot_tmp << std::endl;
  count++;
  // positions
  wrist_pos_tmp = wrist_pos_next;
  elbow_pos_tmp = elbow_pos_next;
  // combine and append
  path_point.block<3, 1>(0, 0) = wrist_pos_tmp;
  path_point.block<9, 1>(3, 0) = wrist_rot_tmp; // flatten matrix
  path_point.block<3, 1>(12, 0) = elbow_pos_tmp;
  path.push_back(path_point);





  // Write to h5 file
  





  return 0;

}
