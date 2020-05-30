#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;


class SimilarityNetwork
{

  public:
    SimilarityNetwork(std::string model_path, VectorXd original_traj); // traj is 1 x 48N
    ~SimilarityNetwork(){};
    VectorXd eval_feature(VectorXd input_traj);
    double compute_distance(VectorXd new_traj);
    VectorXd normalize(VectorXd original_traj); // pos & angle normalized to 0-1, quaternion normalized to unit quaternion
  
    VectorXd tmp_original_traj;
  
  
  private:
    // for normalize the trajectory
    VectorXd quat_id;
    VectorXd pos_and_angle_id;
    // 
    unsigned int DOF = 48;
    unsigned int traj_length = 4800;
    unsigned int feature_length = 100;
    VectorXd oritraj_feature;

    torch::jit::script::Module module; // typedef torch::jit::Module

};

