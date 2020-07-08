
// header
#include "KDL_FK_Solver.h"


KDL_FK_Solver::KDL_FK_Solver()
{
  // Set up FK solvers
  //left_fk_solver = setup_left_kdl();
  //right_fk_solver = setup_right_kdl();

}


// This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver 
KDL::ChainFkSolverPos_recursive KDL_FK_Solver::setup_left_kdl()
{
  // Params
  const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
  const std::string SHOULDER_LINK = "yumi_link_1_l";
  const std::string ELBOW_LINK = "yumi_link_4_l";
  const std::string WRIST_LINK = "yumi_link_7_l";

  // Get tree
  KDL::Tree kdl_tree; 
   if (!kdl_parser::treeFromFile(this->URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      exit(-1);
   }
  //ROS_INFO("Successfully built a KDL tree from URDF file.");

  // Get chain  
  KDL::Chain kdl_chain; 
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    exit(-1);
  }
  //ROS_INFO("Successfully obtained chain from root to wrist.");

  // Find segment number for wrist and elbow links, store in constraint_dat
  if (l_num_wrist_seg == 0 || l_num_elbow_seg == 0 || l_num_shoulder_seg == 0) // if the IDs not set
  {
    unsigned int num_segments = kdl_chain.getNrOfSegments();
    l_num_wrist_seg = num_segments - 1;
    //ROS_INFO_STREAM("There are " << num_segments << " segments in the kdl_chain");
    for (unsigned int i = 0; i < num_segments; ++i){
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        l_num_elbow_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        break;
      }
      if (kdl_chain.getSegment(i).getName() == SHOULDER_LINK){
        l_num_shoulder_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        //break;
      }
    }
  }

  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  //ROS_INFO_STREAM("Joint dimension is: " << kdl_chain.getNrOfJoints()); // 6 joints, 8 segments, checked!

  return fk_solver;

}


// This function sets elbow ID and wrist ID in constraint_data, and returns the KDL_FK solver 
KDL::ChainFkSolverPos_recursive KDL_FK_Solver::setup_right_kdl()
{
  // Params
  const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
  const std::string SHOULDER_LINK = "yumi_link_1_r";
  const std::string ELBOW_LINK = "yumi_link_4_r";
  const std::string WRIST_LINK = "yumi_link_7_r";

  // Get tree
  KDL::Tree kdl_tree; 
   if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      exit(-1);
   }
  //ROS_INFO("Successfully built a KDL tree from URDF file.");

  // Get chain  
  KDL::Chain kdl_chain; 
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    exit(-1);
  }
  //ROS_INFO("Successfully obtained chain from root to wrist.");


  // Find segment number for wrist and elbow links, store in constraint_dat
  if (r_num_wrist_seg == 0 || r_num_elbow_seg == 0 || r_num_shoulder_seg == 0) // if the IDs not set
  {
    unsigned int num_segments = kdl_chain.getNrOfSegments();
    r_num_wrist_seg = num_segments - 1;
    //ROS_INFO_STREAM("There are " << num_segments << " segments in the kdl_chain");
    for (unsigned int i = 0; i < num_segments; ++i){
      //std::cout << "Segment name: " << kdl_chain.getSegment(i).getName() << std::endl;
      if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
        r_num_elbow_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        //break;
      }
      if (kdl_chain.getSegment(i).getName() == SHOULDER_LINK){
        r_num_shoulder_seg = i;
        //ROS_INFO_STREAM("Elbow link found.");
        //break;
      }
    }
    //std::cout << "Shoulder ID: " << constraint_data.r_num_shoulder_seg << ", Elbow ID: " << constraint_data.r_num_elbow_seg << ", Wrist ID: " << constraint_data.r_num_wrist_seg << std::endl;
  }


  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  //ROS_INFO_STREAM("Joint dimension is: " << kdl_chain.getNrOfJoints()); // 6 joints, 8 segments, checked!

  return fk_solver;

}


/* Return left or right wrist position */
Vector3d KDL_FK_Solver::return_wrist_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, bool left_or_right)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame wrist_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  if (left_or_right) // left arm
  {
    result = fk_solver.JntToCart(q_in, wrist_cart_out, l_num_wrist_seg+1);
  }
  else
  {
    result = fk_solver.JntToCart(q_in, wrist_cart_out, r_num_wrist_seg+1);
  }
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }
 
  // Preparations
  Vector3d wrist_pos_cur = Map<Vector3d>(wrist_cart_out.p.data, 3, 1);
  
  // return results
  return wrist_pos_cur;

}


/* Return left or right elbow position */
Vector3d KDL_FK_Solver::return_elbow_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, bool left_or_right)
{
 // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame elbow_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  if(left_or_right) // left arm
  {
    result = fk_solver.JntToCart(q_in, elbow_cart_out, l_num_elbow_seg+1); 
  }
  else
  {
    result = fk_solver.JntToCart(q_in, elbow_cart_out, r_num_elbow_seg+1); 
  }
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for elbow link.");
  }
 
  // Preparations
  Vector3d elbow_pos_cur = Map<Vector3d>(elbow_cart_out.p.data, 3, 1);
 
  // return results
  return elbow_pos_cur;

}

/* Return left or right wrist orientation (rotation matrix) */
Matrix3d KDL_FK_Solver::return_wrist_ori(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, bool left_or_right)
{
      // Get joint angles
      KDL::JntArray q_in(q_cur.size()); 
      for (unsigned int i = 0; i < q_cur.size(); ++i)
      {
        q_in(i) = q_cur(i);
      }

      // Do FK using KDL, get the current elbow/wrist/shoulder state
      KDL::Frame wrist_cart_out; // Output homogeneous transformation
      int result;
      if(left_or_right) // left arm
      {
        result = fk_solver.JntToCart(q_in, wrist_cart_out, l_num_wrist_seg+1);
      }
      else
      {
        result = fk_solver.JntToCart(q_in, wrist_cart_out, r_num_wrist_seg+1);        
      }
      if (result < 0){
        ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
        exit(-1);
      }
      else{
        //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
      }

      // Preparations
      Matrix3d wrist_ori_cur = Map<Matrix<double, 3, 3, RowMajor> >(wrist_cart_out.M.data, 3, 3); 

  // Return cost function value
  return wrist_ori_cur;

}


/* Return left or right shoulder position */
Vector3d KDL_FK_Solver::return_shoulder_pos(KDL::ChainFkSolverPos_recursive &fk_solver, Matrix<double, 7, 1> q_cur, bool left_or_right)
{
  // Get joint angles
  KDL::JntArray q_in(q_cur.size()); 
  for (unsigned int i = 0; i < q_cur.size(); ++i)
  {
    q_in(i) = q_cur(i);
  }

  // Do FK using KDL, get the current elbow/wrist/shoulder state
  KDL::Frame shoulder_cart_out;//, shoulder_cart_out; // Output homogeneous transformation
  int result;
  if (left_or_right) // left arm
  {
    result = fk_solver.JntToCart(q_in, shoulder_cart_out, l_num_shoulder_seg+1);
  }
  else
  {
    result = fk_solver.JntToCart(q_in, shoulder_cart_out, r_num_shoulder_seg+1);
  }
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing shoulder link, something went wrong");
    exit(-1);
  }
  else{
    //ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }
 
  // Preparations
  Vector3d shoulder_pos_cur = Map<Vector3d>(shoulder_cart_out.p.data, 3, 1);
  
  // return results
  return shoulder_pos_cur;

}


// test usage
int main(int argc, char **argv)
{
  // Initialization
  KDL_FK_Solver fk_solver;
  KDL::ChainFkSolverPos_recursive left_fk_solver = fk_solver.setup_left_kdl();
  KDL::ChainFkSolverPos_recursive right_fk_solver = fk_solver.setup_right_kdl();

  // New joint angles
  Matrix<double, 7, 1> q_test = Matrix<double, 7, 1>::Zero();
  
  // Perform FK
  Vector3d l_wrist_pos = fk_solver.return_wrist_pos(left_fk_solver, q_test, true);
  Vector3d r_wrist_pos = fk_solver.return_wrist_pos(right_fk_solver, q_test, false);

  Vector3d l_elbow_pos = fk_solver.return_elbow_pos(left_fk_solver, q_test, true);
  Vector3d r_elbow_pos = fk_solver.return_elbow_pos(right_fk_solver, q_test, false);

  Vector3d l_shoulder_pos = fk_solver.return_shoulder_pos(left_fk_solver, q_test, true);
  Vector3d r_shoulder_pos = fk_solver.return_shoulder_pos(right_fk_solver, q_test, false);

  Matrix3d l_wrist_rot = fk_solver.return_wrist_ori(left_fk_solver, q_test, true);
  Matrix3d r_wrist_rot = fk_solver.return_wrist_ori(right_fk_solver, q_test, false);
  // convert rotation matrix to quaternion for ease of validation in RViz
  Quaterniond l_wrist_quat(l_wrist_rot);
  Quaterniond r_wrist_quat(r_wrist_rot);

  // Display results
  std::cout << "Test results: " << std::endl;
  std::cout << "l_wrist_pos= " << l_wrist_pos.transpose() 
            << "\n, l_elbow_pos= " << l_elbow_pos.transpose() 
            << "\n, l_shoulder_pos= " << l_shoulder_pos.transpose() 
            << "\n, l_wrist_quat(w,x,y,z)= " << l_wrist_quat.w() << " "
                                             << l_wrist_quat.x() << " "
                                             << l_wrist_quat.y() << " "
                                             << l_wrist_quat.z() << std::endl;
  std::cout << "r_wrist_pos= " << r_wrist_pos.transpose() 
            << "\n, r_elbow_pos= " << r_elbow_pos.transpose() 
            << "\n, r_shoulder_pos= " << r_shoulder_pos.transpose() 
            << "\n, r_wrist_quat(w,x,y,z)= " << r_wrist_quat.w() << " "
                                             << r_wrist_quat.x() << " "
                                             << r_wrist_quat.y() << " "
                                             << r_wrist_quat.z() << std::endl;
  
  std::cout << "Done." << std::endl;

  return 0;

}




