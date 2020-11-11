#ifndef _TRANSFORM_MATRIX_HELPER_H
#define _TRANSFORM_MATRIX_HELPER_H
#include <iostream>
#include <chrono>
#include <Eigen/Core>
// #include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <vector>
#include <iterator>
#include "config.h"
#include <typeinfo>

using namespace std;
using namespace Eigen;
using namespace cfg;

namespace matrix_helper{

struct Pose6d{
    Eigen::Vector3d t;
    Eigen::Quaterniond R;
};


// @param original_trajectory:  MatrixXd of shape d x N
MatrixXd interpolate_trajectory(MatrixXd original_trajectory, unsigned int num_datapoints)
{
  int d  = original_trajectory.rows();
  int N = original_trajectory.cols();
  // Prep
  VectorXd t_ori(N);
  double dt_ori = 1.0 / (N-1);
  for (unsigned int i = 0; i < N; i++)
    t_ori[i] = 1 - dt_ori*i;
  //std::cout << "debug: t_ori = " << t_ori.transpose() << std::endl;
  
  VectorXd t_interp(num_datapoints);
  double dt_interp = 1.0 / (num_datapoints-1);
  for (unsigned int i = 0; i < num_datapoints; i++)
    t_interp[i] = 1 - dt_interp*i;
  //std::cout << "debug: t_interp = " << t_interp.transpose() << std::endl;
    
  // Linear interpolation
  MatrixXd interp_trajectory(d, num_datapoints); // initialization
  for (unsigned int t = 0; t < num_datapoints-1; t++) // skip the last
  {
    for (unsigned int n = 0; n < N-1; n++) // skip the last
    {
      if (t_interp[t] <= t_ori[n] && t_interp[t] > t_ori[n+1]) // find the interval
      {
        double ratio = (t_interp[t] - t_ori[n]) / (t_ori[n+1] - t_ori[n]);
        interp_trajectory.block(0, t, d, 1) = original_trajectory.block(0, n, d, 1) 
                                    + ratio * (original_trajectory.block(0, n+1, d, 1) - original_trajectory.block(0, n, d, 1));
        break; // skip to interpolate next point
      }
    }
  }
  // set the last point
  interp_trajectory.block(0, num_datapoints-1, d, 1) = original_trajectory.block(0, N-1, d, 1);

  return interp_trajectory;

}

// Currently only for JOINT_DOF x 1 Matrix
std::vector<Matrix<double,JOINT_DOF,1>> interpolate_trajectory_for_stdvec(std::vector<Matrix<double,JOINT_DOF,1>> original_trajectory, unsigned int num_datapoints)
{
  int d = original_trajectory[0].rows();
  int N = original_trajectory.size();
  // Prep
  VectorXd t_ori(N);
  double dt_ori = 1.0 / (N-1);
  for (unsigned int i = 0; i < N; i++)
    t_ori[i] = 1 - dt_ori*i;
  //std::cout << "debug: t_ori = " << t_ori.transpose() << std::endl;
  
  VectorXd t_interp(num_datapoints);
  double dt_interp = 1.0 / (num_datapoints-1);
  for (unsigned int i = 0; i < num_datapoints; i++)
    t_interp[i] = 1 - dt_interp*i;
  //std::cout << "debug: t_interp = " << t_interp.transpose() << std::endl;
    
  // Linear interpolation
  std::vector< Matrix<double,JOINT_DOF,1>> interp_trajectory(num_datapoints); // initialization
  
  for (unsigned int t = 0; t < num_datapoints-1; t++) // skip the last
  {
    for (unsigned int n = 0; n < N-1; n++) // skip the last
    {
      if (t_interp[t] <= t_ori[n] && t_interp[t] > t_ori[n+1]) // find the interval
      {
        double ratio = (t_interp[t] - t_ori[n]) / (t_ori[n+1] - t_ori[n]);
        interp_trajectory[t] = original_trajectory[n]
                                    + ratio * (original_trajectory[n+1]- original_trajectory[n]);
        break; // skip to interpolate next point
      }
    }
  }
  // set the last point
  interp_trajectory[num_datapoints-1] = original_trajectory[N-1];

  return interp_trajectory;

}

void clamp_to_joint_limits(Eigen::Matrix<double,NUM_OF_JOINTS/2,1> &q,const double lower_limits[], const double upper_limits[]) {
    for (int i = 0; i < NUM_OF_JOINTS/2; i++)
    {
        if(q(i,0)<lower_limits[i]) q(i,0) = lower_limits[i];
        else if(q(i,0)>upper_limits[i]) q(i,0) = upper_limits[i];
        else {};
    }
}
    
std::vector<double> Matrix2stdVec(const Eigen::MatrixXd M) {
    std::vector<double> v(M.rows());
    for (int i = 0; i < M.rows(); i++)
    {
        v[i] = M(i,0);
    }
    return v;
}

Eigen::MatrixXd stdVec2Matrix(const std::vector<double> v) {
    MatrixXd M(v.size(),1);
    for (int i = 0; i < v.size(); i++)
    {
        M(i,0) = v[i];
    }
    return M; 
}

Eigen::Quaterniond stdVec2Quat(const std::vector<double> v) {
    assert(v.size()==4);
    // Assume format of v is [w x y z]
    Eigen::Quaterniond quat(v[0],v[1],v[2],v[3]);
    return quat;
}

std::vector<double> vectorMinus(std::vector<double> a, std::vector<double> b) {
    assert(a.size()==b.size());
    std::vector<double> res(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        res[i] = a[i] - b[i];
    }
    return res;
}

std::vector<double> interpolate_between_stdvec(std::vector<double> a, std::vector<double> b, double t) {
    assert(a.size()==b.size());
    std::vector<double> res(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        res[i] = a[i] + t * (b[i] - a[i]);
    }
    return res;   
}

static double cond(Eigen::MatrixXd A) {
    JacobiSVD<MatrixXd> svd(A);
    double cond = svd.singularValues()(0) 
        / svd.singularValues()(svd.singularValues().size()-1);
    return cond;
}

static void output_rank(Eigen::MatrixXd M, std::string name) {
    Eigen::FullPivLU<MatrixXd> lu(M);
    cout << "By default, the rank of "<<name<<" is found to be " << lu.rank() << endl;
}

Eigen::MatrixXd concat_eigen_vectors(Eigen::MatrixXd v1, Eigen::MatrixXd v2) {
    const int nrow1 = v1.rows();
    const int nrow2 = v2.rows();
    Eigen::MatrixXd res(nrow1+nrow2,1);
    res.block(0,0,nrow1,1) = v1;
    res.block(nrow1,0,nrow2,1) = v2;
    return res;
}

Eigen::Matrix<double,NUM_OF_JOINTS,1> concat_joint_angles(Eigen::Matrix<double,NUM_OF_JOINTS/2,1> joint_angle_left, Eigen::Matrix<double,NUM_OF_JOINTS/2,1> joint_angle_right) {
    Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle;
    joint_angle.block(0,0,NUM_OF_JOINTS/2,1) = joint_angle_left;
    joint_angle.block(NUM_OF_JOINTS/2,0,NUM_OF_JOINTS/2,1) = joint_angle_right;
    return joint_angle;
}

Eigen::Matrix<double,3,1> create_translation_vector(const double x, const double y, const double z) {
    return Eigen::Matrix<double,3,1>(x,y,z);
}

Eigen::Matrix<double,3,1> Txyz(const double x, const double y, const double z) {
    Eigen::Matrix<double,3,1>  translation_vector = Eigen::Matrix<double,3,1>(x,y,z);
    return translation_vector;
}

Eigen::Matrix<double,3,3>  Rxa(const double a) {
    // @param a:    rad
    Eigen::Matrix<double,3,3>  rot = Eigen::Matrix<double,3,3> ::Identity();
    rot(1,1) = double(cos(a));
    rot(1,2) = -double(sin(a));
    rot(2,1) = double(sin(a));
    rot(2,2) = double(cos(a));
    return rot;
}

Eigen::Matrix<double,3,3>  Ryb(const double b) {
    // @param a:    rad
    Eigen::Matrix<double,3,3>  rot = Eigen::Matrix<double,3,3> ::Identity();
    rot(0,0) = double(cos(b));
    rot(0,2) = double(sin(b));
    rot(2,0) = -double(sin(b));
    rot(2,2) = double(cos(b));
    return rot;
}


Eigen::Matrix<double,3,3>  Rzc(const double c) {
    // @param a:    rad
    Eigen::Matrix<double,3,3>  rot = Eigen::Matrix<double,3,3> ::Identity();
    rot(0,0) = double(cos(c));
    rot(0,1) = -double(sin(c));
    rot(1,0) = double(sin(c));
    rot(1,1) = double(cos(c));
    return rot;
}


Eigen::Matrix<double,3,3>  dRzc(const double c) {
    // @param a:    rad
    Eigen::Matrix<double,3,3>  rot = Eigen::Matrix<double,3,3> ::Zero();
    rot(0,0) = -double(sin(c));
    rot(0,1) = -double(cos(c));
    rot(1,0) = double(cos(c));
    rot(1,1) = -double(sin(c));
    return rot;
}


Eigen::Matrix<double,3,3>  Rrpy(const double a, const double b, const double c) {
    Eigen::Matrix<double,3,3>  rot = Rzc(c)*Ryb(b);
    rot = rot * Rxa(a);
    return rot;
}

Eigen::Matrix<double,3,3> euler2rotmat(Eigen::Matrix<double,3,1> euler) {
    Eigen::Matrix<double,3,3> rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * 
                       Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    return rotation_matrix;
}

Eigen::Quaterniond euler2quat(Vector3d euler) {
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * 
                  Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    return quat;
}

void outputQuat(Eigen::Quaterniond quat) {
    cout<<quat.x()<<" "<<quat.y()<<" "<<quat.z()<<" "<<quat.w()<<endl;
}

// Some cost functions
double tracking_pos_err(Vector3d pos1, Vector3d pos2) {
    return (pos1-pos2).norm();
}

double clamp(double val,double min=-1,double max=1) {
    if(val>max) val = max;
    else if(val<min) val = min;
    return val;
}

double tracking_ori_err_with_ea(Eigen::Matrix<double,3,1> ea1, Eigen::Matrix<double,3,1> ea2) {
    Eigen::Matrix<double,3,3> rot1 = euler2rotmat(ea1);
    Eigen::Matrix<double,3,3> rot2 = euler2rotmat(ea2);
    double relative_angle = abs( acos ( clamp( ((rot1 * rot2.transpose()).trace() - double(1.0)) / double(2.0))));
    return relative_angle;
}

double tracking_ori_err_with_quat(Eigen::Quaterniond quat1, Eigen::Quaterniond quat2) {
    Eigen::Matrix<double,3,3> rot1 = quat1.toRotationMatrix();
    Eigen::Matrix<double,3,3> rot2 = quat2.toRotationMatrix();
    double relative_angle = abs( acos ( clamp( ((rot1 * rot2.transpose()).trace() - double(1.0)) / double(2.0))));
    return relative_angle;
}

double relative_angle_cost(Eigen::Matrix<double,3,3> rot_cur, Eigen::Quaternion<double> quat_ref) {
    Eigen::Matrix<double,3,3> rot_ref = quat_ref.toRotationMatrix();
    double relative_angle = abs( acos (( (rot_ref * rot_cur.transpose()).trace() - double(1.0)) / double(2.0)));
    return relative_angle;
}


double distance_cost(Eigen::Matrix<double,3,1> pos_cur, Eigen::Matrix<double,3,1> pos_ref) {
    Eigen::Matrix<double,3,1> pos_err = pos_ref - pos_cur;
    return pos_err.transpose()*pos_err;
}


double over_upper_bound_cost(const double x,const double upper_bound) {
    if(x>upper_bound) {
        return (x - upper_bound) * (x - upper_bound);
    }
    else {
        return double(0.0);
    }
}


double beyond_lower_bound_cost(const double x,const double lower_bound) {
    if(x<lower_bound) {
        return (x - lower_bound) * (x - lower_bound);
    }
    else {
        return double(0.0);
    }
}

}

#endif