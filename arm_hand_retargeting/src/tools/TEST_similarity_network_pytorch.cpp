#include "similarity_network_pytorch.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;

int main(int argc, char **argv)
{

  VectorXd test_input(4800);
  test_input = VectorXd::Ones(4800);
  
  test_input[0] = 0;
  test_input[1] = 1;
  test_input[2] = 2;
  test_input[3] = 3;  
  test_input[4] = 4;
  test_input[4799] = 0.5;
  for (int i = 0; i < 4800; i++)
    test_input[i] = i;
  

  std::cout << "Test input size: " << test_input.size() << std::endl;
  
  
  std::string model_path = "/home/liangyuwei/sign_language_robot_ws/test_imi_data/traced_model_adam_euclidean_epoch500_bs128_group_split_dataset.pt";
  
  
  // 
  std::cout << "Set up Similarity network.." << std::endl;
  SimilarityNetwork similarity_network(model_path, test_input);
  
  
  // Obtain resampled, normalized and flattened original trajectory from test_imi_data_YuMi.h5 which is generated using resample_normalize_flatten.m through MATLAB !!!
  
  
  
  // 
  double dist = similarity_network.compute_distance(similarity_network.tmp_original_traj);
  std::cout << "Test distance using the original trajectory (should be 0): " << dist << std::endl; // checked !
  
  









  return 0;

}
