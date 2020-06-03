#include "similarity_network_pytorch.h"

VectorXd SimilarityNetwork::normalize(VectorXd original_traj)
{

  //std::cout << "size: " << pos_and_angle_id.rows() << " x " << pos_and_angle_id.cols() << std::endl;
  //std::cout << "debug: original_traj = " << original_traj << std::endl;

  // convert to matrixxd for ease of computation
  MatrixXd tmp = Map<Matrix<double, 100, 48, RowMajor>>(original_traj.data(), 100, 48);
  
  //std::cout << "debug: tmp = " << tmp << std::endl;
  /*
  std::cout << "compare" << std::endl;
  std::cout << "size: " << tmp.rows() << " x " << tmp.cols() << std::endl;
  std::cout << original_traj[0] << " " << original_traj[1] << " " << original_traj[2] << " " << std::endl;
  std::cout << tmp(0, 0) << " " << tmp(0, 1) << " " << tmp(0, 2) << std::endl;
  */

  
  
  // for pos and angle data
  //std::cout << "previous: " << std::endl << tmp.block(0, 0, 100, 1).transpose() << std::endl;
  //std::cout << "previous: " << std::endl << tmp.block(0, 1, 100, 1).transpose() << std::endl;
  //std::cout << "before: " << tmp << std::endl;
  double min_num, max_num;
  for (int pg_id = 0; pg_id < pos_and_angle_id.rows(); pg_id++)
  {
    unsigned int dof_id = (unsigned int) pos_and_angle_id(pg_id)-1; // index starts at 0
    min_num = tmp.block(0, dof_id, 100, 1).minCoeff();
    max_num = tmp.block(0, dof_id, 100, 1).maxCoeff();
    //std::cout << "debug: min = " << min_num << ", max = " << max_num << std::endl;

    if (min_num == max_num) // necessary!!!
      continue;

    for (int n = 0; n < tmp.rows(); n++)
      tmp(n, dof_id) = (tmp(n, dof_id) - min_num) / (max_num - min_num); // index starts at 0

  }
  //std::cout << "after: " << std::endl << tmp.block(0, 0, 100, 1).transpose() << std::endl;
  //std::cout << "after: " << std::endl << tmp.block(0, 1, 100, 1).transpose() << std::endl;
  //std::cout << "after: " << tmp << std::endl;

  // for quaternion data
  for(unsigned int q_id = 0; q_id < 2; q_id++)
  { 
    unsigned int dof_id = (unsigned int) quat_id(q_id*4) - 1; // index starts from 0
    for (unsigned int n = 0; n < tmp.rows(); n++)
    {
      Quaterniond q(tmp(n, dof_id), tmp(n, dof_id+1), tmp(n, dof_id+2), tmp(n, dof_id+3));
      //std::cout << "debug: quaternion before = " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
      q.normalize();
      //std::cout << "debug: quaternion after = " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
      tmp.block(n, dof_id, 1, 4) << q.w(), q.x(), q.y(), q.z(); // [w,x,y,z], in consistent with MATLAB quaternion data type
    }
  }


  // convert back to VectorXd
  MatrixXd tmp_tmp = tmp.transpose();
  VectorXd normalized_traj = Map<VectorXd>(tmp_tmp.data(), original_traj.size());
  
  // print for debug
  //std::cout << "matrix first row: " << tmp.block(0, 0, 1, 10) << std::endl;
  //std::cout << "vector:" << normalized_traj.block(0, 0, 10, 1).transpose() << std::endl;
  
  
  return normalized_traj;

}


     
VectorXd SimilarityNetwork::eval_feature(VectorXd input_traj)
{

  // Display the input VectorXd
  /*
  std::cout << "Input size: " << input_traj.rows() << " x " << input_traj.cols() << std::endl;
  std::cout << "Input - VectorXd: " << std::endl << input_traj[0] << " " << input_traj[1] << " "  << input_traj[2] << " "  << input_traj[3] << " "  << input_traj[4] << std::endl; 
  */

  // VectorXd --> std::vector
  std::vector<double> tmp_input_traj(traj_length);
  for (unsigned int it = 0; it < traj_length; it++)
    tmp_input_traj[it] = input_traj[it];
  //std::cout << "Input - std::vector : " << std::endl << tmp_input_traj[0] << " " << tmp_input_traj[1] << " "  << tmp_input_traj[2] << " "  << tmp_input_traj[3] << " "  << tmp_input_traj[4] << std::endl; 


  // std::vector --> torch::Tensor
  torch::Tensor input_tensor = torch::tensor(tmp_input_traj);
  //input_tensor = input_tensor.toType(torch::kFloat);
  input_tensor = torch::unsqueeze(input_tensor, 0);
  //std::cout << "Input tensor size: " << input_tensor.sizes() << std::endl;
  //std::cout << "Input - Tensor: " << std::endl << input_tensor.select(1, 0) << " " << input_tensor.select(1, 1) << " " << input_tensor.select(1, 2) << " " << input_tensor.select(1, 3) << " " << input_tensor.select(1, 4) << " " << std::endl; // select number on dimension 1!!!
 

  // torch::Tensor --> IValue
  std::vector<torch::jit::IValue> input_ivalue; // std::vector to store stack of input samples
  input_ivalue.push_back(input_tensor);
  //std::cout << "Input - IValue: " << input_ivalue[0] << std::endl; // actually the input tensor


  // Forward pass, and convert to a tensor
  //std::cout << "Forward pass: " << std::endl;
  torch::Tensor output = this->module.forward(input_ivalue).toTensor();  // 1 x 100
  //std::cout << "Output tensor size: " << output.sizes() << std::endl;
  //std::cout << "Part of test output: " << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << std::endl;


  // Convert the output
  VectorXd output_feature(feature_length);
  for (unsigned int it = 0; it < feature_length; it++)
    output_feature[it] = output[0][it].item().toFloat();
  //std::cout << "Output VectorXd size: " << output_feature.rows() << " x " << output_feature.cols() << std::endl;
  //std::cout << "Part of test output(VectorXd): " << output_feature[0] << " " << output_feature[1] << " " << output_feature[2] << " " << output_feature[3] << " " << output_feature[4] << " " << std::endl; 
  

  return output_feature; // 100 x 1

}



SimilarityNetwork::SimilarityNetwork(std::string model_path, VectorXd original_traj)
{

  // Set up IDs
  VectorXd tmp_quat_id(8);
  tmp_quat_id << 4, 5, 6, 7, 14, 15, 16, 17;
  this->quat_id = tmp_quat_id;

  VectorXd tmp_pos_and_angle_id(40);
  tmp_pos_and_angle_id << 1, 2, 3, 8, 9, 10, 11, 12, 13, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48;
  this->pos_and_angle_id = tmp_pos_and_angle_id;


  // debug compute_distance()
  this->tmp_original_traj = original_traj;


  // Deserialize the ScriptModule from a file using torch::jit::load().
  //std::cout << "load models" << std::endl;
  try {
    this->module = torch::jit::load(model_path);
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    exit(-1);
  }

  //std::cout << "debug: original_traj = " << original_traj.transpose() << std::endl;
  //std::cout << "debug: original_traj size is: " << original_traj.rows() << " x " << original_traj.cols() << std::endl;


  // Normalize the original trajectory
  //std::cout << "normalize trajectory..." << std::endl;
  original_traj = this->normalize(original_traj);
  //std::cout << "debug: normalized original_traj = " << original_traj << std::endl;
  //std::cout << "debug: normalized original_traj size is: " << original_traj.rows() << " x " << original_traj.cols() << std::endl;


  // Evaluate the original trajectory to get its feature
  //std::cout << "evaluate features..." << std::endl;
  this->oritraj_feature = this->eval_feature(original_traj);
  //std::cout << "debug: oritraj_feature = " << oritraj_feature << std::endl;
  //std::cout << "debug: oritraj_feature size is: " << oritraj_feature.rows() << " x " << oritraj_feature.cols() << std::endl;

  // Print original trajectory's feature
  //std::cout << "Feature of the original trajectory: " << this->oritraj_feature << std::endl;

}



double SimilarityNetwork::compute_distance(VectorXd new_traj)
{

  //std::cout << "debug: new_traj = " << new_traj << std::endl;
  //std::cout << "debug: new_traj size is: " << new_traj.rows() << " x " << new_traj.cols() << std::endl;

  // Normalize the input trajectory, pos/angle to 0-1, quaternion to unit
  VectorXd new_resampled_traj = this->normalize(new_traj);
  //std::cout << "debug: new_resampled_traj = " << new_resampled_traj << std::endl;
  //std::cout << "debug: new_resampled_traj size is: " << new_resampled_traj.rows() << " x " << new_resampled_traj.cols() << std::endl;


  // Compute feature vector corresponding to the given trajectory
  //std::cout << "compute new feature..." << std::endl;
  VectorXd newtraj_feature = this->eval_feature(new_resampled_traj);
  //std::cout << "debug: newtraj_feature = " << newtraj_feature << std::endl;
  //std::cout << "debug: newtraj_feature size is: " << newtraj_feature.rows() << " x " << newtraj_feature.cols() << std::endl;

  // Compute distance
  double dist = (newtraj_feature - this->oritraj_feature).norm();
  //std::cout << "debug: dist = " << dist << std::endl;

  return dist;

}
