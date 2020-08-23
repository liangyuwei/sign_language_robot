#ifndef _TRANSFORM_MATRIX_HELPER_H
#define _TRANSFORM_MATRIX_HELPER_H
#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <vector>
#include <iterator>
#include "config_nullspace.h"
#include <typeinfo>

using namespace std;
using namespace Eigen;
// using namespace cfg;

std::vector<double> Matrix2stdVec(Eigen::MatrixXd M) {
    std::vector<double> v(M.rows());
    for (int i = 0; i < M.rows(); i++)
    {
        v[i] = M(i,0);
    }
    return v;
}

static void output_rank(Eigen::MatrixXd M, std::string name) {
    Eigen::FullPivLU<MatrixXd> lu(M);
    cout << "By default, the rank of "<<name<<" is found to be " << lu.rank() << endl;
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

Quaterniond euler2quat(Vector3d euler) {
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * 
                  Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    return quat;
}

void outputQuat(Quaterniond quat) {
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

double relative_angle_cost(Eigen::Matrix<double,3,3> rot_cur, Quaternion<double> quat_ref) {
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

#endif