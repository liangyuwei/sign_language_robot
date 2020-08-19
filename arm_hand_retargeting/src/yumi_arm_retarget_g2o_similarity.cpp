#include "yumi_arm_retarget_g2o_similarity.h"

std::stringstream read_file(std::string file_name)
{
  std::ifstream ifs(file_name);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss;
}

void print_debug_output(std::string data_name, std::vector<double> data)
{
  std::cout << "debug: " << data_name << " = ";
  for (unsigned s = 0; s < data.size(); s++)
    std::cout << data[s] << " ";
  std::cout << std::endl << std::endl;
}


/**
 * @brief Compute IK using TRAC-IK.
 * 
 * Perform IK on wrist position and orientation trajectories, so as to provide a fine initial result.
 * If returned int is >= 0, then successful, otherwise failed !
 */
int run_trac_ik(Matrix<double, 7, 1> q_initial, Matrix<double, 7, 1> &q_result, Vector3d pos_goal, Matrix3d ori_goal,
                Matrix<double, 7, 1> lower_joint_limits, Matrix<double, 7, 1> upper_joint_limits, 
                bool left_or_right, double timeout)
{
  double eps = 1e-5;

  // Get joint angles and joint limits
  KDL::JntArray q_in(q_initial.size()); 
  for (unsigned int i = 0; i < q_initial.size(); ++i)
  {
    q_in(i) = q_initial(i);
  }

  KDL::JntArray lb(lower_joint_limits.size()); 
  for (unsigned int i = 0; i < lower_joint_limits.size(); ++i)
  {
    lb(i) = lower_joint_limits(i);
  }

  KDL::JntArray ub(upper_joint_limits.size()); 
  for (unsigned int i = 0; i < upper_joint_limits.size(); ++i)
  {
    ub(i) = upper_joint_limits(i);
  }  

  // Get end effector goal
  KDL::Vector pos(pos_goal[0], pos_goal[1], pos_goal[2]);
  KDL::Vector rot_col_x(ori_goal(0, 0), ori_goal(1, 0), ori_goal(2, 0)); 
  KDL::Vector rot_col_y(ori_goal(0, 1), ori_goal(1, 1), ori_goal(2, 1)); 
  KDL::Vector rot_col_z(ori_goal(0, 2), ori_goal(1, 2), ori_goal(2, 2)); 
  KDL::Rotation rot(rot_col_x, rot_col_y, rot_col_z);
  // KDL::Rotation contains [X, Y, Z] three columns, i.e. [rot_col_x, rot_col_y, rot_col_z]
  KDL::Frame end_effector_pose(rot, pos);
  // debug:
  /*
  std::cout << "debug: original pos_goal = " << pos_goal.transpose() << std::endl;
  std::cout << "debug: original rot_goal = " << ori_goal << std::endl;
  Vector3d tmp_wrist_pos = Map<Vector3d>(end_effector_pose.p.data, 3, 1);
  Matrix3d tmp_wrist_ori = Map<Matrix<double, 3, 3, RowMajor> >(end_effector_pose.M.data, 3, 3); 
  std::cout << "debug: converted pos_goal = " << tmp_wrist_pos.transpose() << std::endl;
  std::cout << "debug: converted rot_goal = " << tmp_wrist_ori << std::endl;
  */

  // Get KDL::Chain
  // const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";
  // const std::string BASE_LINK = "world"; 
  const std::string WRIST_LINK = (left_or_right ? LEFT_WRIST_LINK : RIGHT_WRIST_LINK);

  // Get tree
  KDL::Tree kdl_tree; 
   if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      exit(-1);
   }

  // Get chain  
  KDL::Chain kdl_chain; 
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    exit(-1);
  }

  // Construct a TRAC-IK solver
  // last argument is optional
  // Speed: returns very quickly the first solution found
  // Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
  // Manip1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T))
  // Manip2: runs for full timeout, returns solution that minimizes cond(J) = |J|*|J^-1|
  TRAC_IK::TRAC_IK ik_solver(kdl_chain, lb, ub, timeout, eps, TRAC_IK::Distance);//TRAC_IK::Speed);
  KDL::JntArray q_res;
  int rc = ik_solver.CartToJnt(q_in, end_effector_pose, q_res); //, KDL::Twist tolerances); // optional tolerances, see trac_ik_lib documentation for usage

  /*
  if (rc >= 0)
  {
    std::cout << "TRAC-IK succeeded!" << std::endl;
  }
  else
  {
    std::cout << "TRAC-IK failed!" << std::endl;
    // exit(-1);
  }
  */

  // Return the result
  if (rc >= 0) // if successful
  {
    for (unsigned int i = 0; i < q_result.size(); ++i)
    {
      q_result(i) = q_res(i);
    }
  }
  else // if fail, return the initial values
  {
    q_result = q_initial;
  }
  // return q_res_vec;

  return rc;
}


int main(int argc, char *argv[])
{

  // Initialize a ros node, for the calculation of collision distance
  ros::init(argc, argv, "yumi_sign_language_robot_retargeting");

  // Process the terminal arguments
  static struct option long_options[] = 
  {
    {"in-h5-filename",        required_argument, NULL, 'i'},
    {"in-group-name",         required_argument, NULL, 'g'},
    {"out-h5-filename",       required_argument, NULL, 'o'},
    {"help",                        no_argument, NULL, 'h'},
    {0,                                       0,    0,   0}
  };
  int c;
  while(1)
  {
    int opt_index = 0;
    // Get arguments
    c = getopt_long(argc, argv, "i:g:o:h", long_options, &opt_index);
    if (c == -1)
      break;

    // Process
    switch(c)
    {
      case 'h':
        std::cout << "Help: \n" << std::endl;
        std::cout << "    This program reads imitation data from h5 file and performs optimization on the joint angles. The results are stored in a h5 file at last.\n" << std::endl; 
        std::cout << "Arguments:\n" << std::endl;
        std::cout << "    -i, --in-h5-filename, specify the name of the input h5 file, otherwise a default name specified inside the program will be used. Suffix is required.\n" << std::endl;
        std::cout << "    -g, --in-group-name, specify the group name in the h5 file, which is actually the motion's name.\n" << std::endl;
        std::cout << "    -o, --out-h5-name, specify the name of the output h5 file to store the resultant joint trajectory.\n" << std::endl;
        return 0;
        break;

      case 'i':
        in_file_name = optarg;
        break;

      case 'o':
        out_file_name = optarg;
        break;

      case 'g':
        in_group_name = optarg;
        break;

      default:
        break;
    }

  }
  std::cout << "The input h5 file name is: " << in_file_name << std::endl;
  std::cout << "The motion name is: " << in_group_name << std::endl;
  std::cout << "The output h5 file name is: " << out_file_name << std::endl;


  // Create a struct for storing user-defined data
  my_constraint_struct constraint_data; 


  // Variables' bounds
  std::cout << ">>>> Setting up joint position limits" << std::endl;
  const std::vector<double> q_l_arm_lb = {-2.8, -2.49, -1.2, -1.7, -2.0, -1.5, -2.0};
  //{-2.9, -2.49, 0.0, -1.7, -1.578, -1.5, -1.57};//{-2.94, -2.5, -2.94, -2.16, -5.06, -1.54, -4.0};
  const std::vector<double> q_l_arm_ub = {0.5, 0.75, 2.2, 1.4, 1.578, 2.1, 1.578};//{2.94, 0.76, 2.94, 1.4, 5.06, 2.41, 4.0};

  const std::vector<double> q_r_arm_lb = {-0.5, -2.49, -2.2, -1.7, -2.0, -1.5, -2.0}; // modified on 2020/07/20
  //{-2.9, -2.49, 0.0, -1.7, -1.578, -1.5, -1.57};//{-2.94, -2.50, -2.94, -2.16, -5.06, -1.54, -4.0};
  const std::vector<double> q_r_arm_ub = {2.8, 0.75, 1.2, 1.4, 1.578, 2.1, 1.578}; // modified on 2020/07/20
  //{0.5, 0.75, 2.9, 1.4, 1.578, 2.1, 1.57};//{2.94, 0.76, 2.94, 1.4, 5.06, 2.41, 4.0};

  // remember to keep consistent with _robot_finger_start and _robot_finger_goal
  const std::vector<double> q_l_finger_lb = {-1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -0.75, 0.0, -0.2, -0.15};
                                            //{-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0};
  const std::vector<double> q_l_finger_ub = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.1,  0.0,  0.0};
                                            //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0};
  const std::vector<double> q_r_finger_lb = {-1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.0, 0.0, -0.2, -0.15};
                                            //{-1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0};
  const std::vector<double> q_r_finger_ub = {0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3, 0.1,  0.0,  0.0}; 
                                            //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.4, 0.0, 0.0};

  
  std::vector<double> qlb = q_l_arm_lb;
  std::vector<double> qub = q_l_arm_ub;
  qlb.insert(qlb.end(), q_r_arm_lb.cbegin(), q_r_arm_lb.cend());
  qlb.insert(qlb.end(), q_l_finger_lb.cbegin(), q_l_finger_lb.cend());
  qlb.insert(qlb.end(), q_r_finger_lb.cbegin(), q_r_finger_lb.cend());
  qub.insert(qub.end(), q_r_arm_ub.cbegin(), q_r_arm_ub.cend());
  qub.insert(qub.end(), q_l_finger_ub.cbegin(), q_l_finger_ub.cend());
  qub.insert(qub.end(), q_r_finger_ub.cbegin(), q_r_finger_ub.cend());
  Matrix<double, JOINT_DOF, 1> q_pos_lb = Map<Matrix<double, JOINT_DOF, 1> >(qlb.data(), JOINT_DOF, 1);
  Matrix<double, JOINT_DOF, 1> q_pos_ub = Map<Matrix<double, JOINT_DOF, 1> >(qub.data(), JOINT_DOF, 1);
  constraint_data.q_pos_lb = q_pos_lb;
  constraint_data.q_pos_ub = q_pos_ub;


  // Setup KDL FK solver( and set IDs for wrist, elbow and shoulder)
  std::cout << ">>>> Setting KDL FK Solvers " << std::endl;
  KDL::ChainFkSolverPos_recursive left_fk_solver = setup_kdl(constraint_data, true);
  KDL::ChainFkSolverPos_recursive right_fk_solver = setup_kdl(constraint_data, false); 


  // Robot configuration-related data
  // robot's shoulder position
  // constraint_data.l_robot_shoulder_pos = Vector3d(0.05355, 0.0725, 0.51492); //Vector3d(-0.06, 0.235, 0.395);
  // constraint_data.r_robot_shoulder_pos = Vector3d(0.05255, -0.0725, 0.51492); //Vector3d(-0.06, -0.235, 0.395);
  constraint_data.l_robot_finger_start <<  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.1,  0.0,  0.0; 
                                        // 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3, 0.4,  0.0,  0.0; 
  constraint_data.l_robot_finger_final << -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -0.75, 0.0, -0.2, -0.15;  
                                       // -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0;
  constraint_data.r_robot_finger_start <<  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3, 0.1,  0.0,  0.0;
                                        // 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 0.3, 0.4,  0.0,  0.0;
  constraint_data.r_robot_finger_final << -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.6, -1.0, 0.0, -0.2, -0.15; // right and left hands' joint ranges are manually set to be the same, but according to Inspire Hand Inc, this will keep changing in the future.
                                        // -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.6, -1.7, -1.0, 0.0, -0.4, -1.0;
  constraint_data.glove_start << 0,    0, 53,  0,   0, 22,  0,   0, 22,  0,   0, 35,  0,   0;
  constraint_data.glove_start = constraint_data.glove_start * M_PI / 180.0; // in radius  
  constraint_data.glove_final << 45, 100,  0, 90, 120,  0, 90, 120,  0, 90, 120,  0, 90, 120;
  constraint_data.glove_final = constraint_data.glove_final * M_PI / 180.0; 


  // Construct a graph optimization problem 
  std::cout << ">>>> Constructing an optimization graph " << std::endl;
  // Construct solver (be careful, use unique_ptr instead!!! )
  typedef BlockSolver<BlockSolverTraits<-1, -1> > Block; // BlockSolverTraits<_PoseDim, _LandmarkDim>

  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverDense<Block::PoseMatrixType>()); // dense solution
  std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverCholmod<Block::PoseMatrixType>()); // Cholmod
  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverCSparse<Block::PoseMatrixType>()); // CSparse
  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverPCG<Block::PoseMatrixType>()); // PCG
  //std::unique_ptr<Block::LinearSolverType> linearSolver( new LinearSolverEigen<Block::PoseMatrixType>()); // Eigen

  std::unique_ptr<Block> solver_ptr( new Block(std::move(linearSolver)) ); // Matrix block solver

  // Choose update rule
  OptimizationAlgorithmLevenberg *solver = new OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  // OptimizationAlgorithmGaussNewton *solver = new OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
  //OptimizationAlgorithmDogleg *solver = new OptimizationAlgorithmDogleg(std::move(solver_ptr));

  // Construct an optimizer (a graph model)
  SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);


  // Prepare collision checker
  std::cout << ">>>> Preparing collision checker " << std::endl;
  // Get URDF and SRDF for distance computation class
  std::stringstream urdf_string = read_file(URDF_FILE);
  std::stringstream srdf_string = read_file(SRDF_FILE);
  boost::shared_ptr<DualArmDualHandCollision> dual_arm_dual_hand_collision_ptr;
  dual_arm_dual_hand_collision_ptr.reset( new DualArmDualHandCollision(urdf_string.str(), srdf_string.str()) );


  // Prepare trajectory generator
  std::cout << ">>>> Preparing trajectory generator " << std::endl;
  boost::shared_ptr<DMPTrajectoryGenerator> trajectory_generator_ptr;
  trajectory_generator_ptr.reset( new DMPTrajectoryGenerator(in_file_name, in_group_name) );  


  // Add vertices and edges
  std::vector<DualArmDualHandVertex*> v_list(NUM_DATAPOINTS);
  DMPStartsGoalsVertex* dmp_vertex;
  // std::vector<MyUnaryConstraints*> unary_edges;
  std::vector<CollisionConstraint*> collision_edges;
  std::vector<SmoothnessConstraint*> smoothness_edges;  
  TrackingConstraint* tracking_edge;
  DMPConstraints* dmp_edge;

 
  tracking_edge = new TrackingConstraint(trajectory_generator_ptr, 
                                         left_fk_solver, right_fk_solver, 
                                         dual_arm_dual_hand_collision_ptr,
                                         NUM_DATAPOINTS);
  tracking_edge->setId(1);


  // For DMP
  std::cout << ">>>> Adding DMP vertex and tracking edge " << std::endl;
  // preparation
  dmp_vertex = new DMPStartsGoalsVertex();
  Matrix<double, DMPPOINTS_DOF, 1> DMP_ori_starts_goals; // lrw, lew, rew, rw; goal, start. 

  // Manually set an initial state for DMPs
  std::cout << ">>>> Manually set initial state for DMPs" << std::endl;
  MatrixXd lw_set_start(1, 3); //lw_set_start(0, 0) = 0.409; lw_set_start(0, 1) = 0.181; lw_set_start(0, 2) = 0.191; 
  MatrixXd le_set_start(1, 3); le_set_start(0, 0) = 0.218; le_set_start(0, 1) = 0.310; le_set_start(0, 2) = 0.378; 
  MatrixXd rw_set_start(1, 3); rw_set_start(0, 0) = 0.410; rw_set_start(0, 1) = -0.179; rw_set_start(0, 2) = 0.191; 
  MatrixXd re_set_start(1, 3); re_set_start(0, 0) = 0.218; re_set_start(0, 1) = -0.310; re_set_start(0, 2) = 0.377; 
  // set rw start to ensure rw start and lw start symmetric to x-z plane, and leave the others
  lw_set_start = rw_set_start + trajectory_generator_ptr->lrw_start;
  double dist_y = std::abs(lw_set_start(0, 1) - rw_set_start(0,1));
  lw_set_start(0, 1) = dist_y / 2.0;
  rw_set_start(0, 1) = -dist_y / 2.0;

  // compute starts for corresponding relative DMPs
  MatrixXd lrw_set_start(1, 3); lrw_set_start = lw_set_start - rw_set_start;
  MatrixXd lew_set_start(1, 3); lew_set_start = le_set_start - lw_set_start;
  MatrixXd rew_set_start(1, 3); rew_set_start = re_set_start - rw_set_start;
  // compute offsets for starts and goals of DMPs
  MatrixXd lrw_tmp_offset(1, 3); lrw_tmp_offset = lrw_set_start - trajectory_generator_ptr->lrw_start;
  MatrixXd lew_tmp_offset(1, 3); lew_tmp_offset = lew_set_start - trajectory_generator_ptr->lew_start;
  MatrixXd rew_tmp_offset(1, 3); rew_tmp_offset = rew_set_start - trajectory_generator_ptr->rew_start;    
  MatrixXd rw_tmp_offset(1, 3); rw_tmp_offset = rw_set_start - trajectory_generator_ptr->rw_start;
  // add offset to move the whole trajectories
  std::cout << "Original lrw_start = " << trajectory_generator_ptr->lrw_start << std::endl;
  std::cout << "Original lrw_goal = " << trajectory_generator_ptr->lrw_goal << std::endl;
  std::cout << "Original lew_start = " << trajectory_generator_ptr->lew_start << std::endl;
  std::cout << "Original lew_goal = " << trajectory_generator_ptr->lew_goal << std::endl;
  std::cout << "Original rew_start = " << trajectory_generator_ptr->rew_start << std::endl;
  std::cout << "Original rew_goal = " << trajectory_generator_ptr->rew_goal << std::endl;         
  std::cout << "Original rw_start = " << trajectory_generator_ptr->rw_start << std::endl;
  std::cout << "Original rw_goal = " << trajectory_generator_ptr->rw_goal << std::endl;
  trajectory_generator_ptr->lrw_goal += lrw_tmp_offset;
  trajectory_generator_ptr->lrw_start += lrw_tmp_offset;
  trajectory_generator_ptr->lew_goal += lew_tmp_offset;
  trajectory_generator_ptr->lew_start += lew_tmp_offset;
  trajectory_generator_ptr->rew_goal += rew_tmp_offset;
  trajectory_generator_ptr->rew_start += rew_tmp_offset;
  trajectory_generator_ptr->rw_goal += rw_tmp_offset;
  trajectory_generator_ptr->rw_start += rw_tmp_offset;
  std::cout << "Moved lrw_start = " << trajectory_generator_ptr->lrw_start << std::endl;
  std::cout << "Moved lrw_goal = " << trajectory_generator_ptr->lrw_goal << std::endl;
  std::cout << "Moved lew_start = " << trajectory_generator_ptr->lew_start << std::endl;
  std::cout << "Moved lew_goal = " << trajectory_generator_ptr->lew_goal << std::endl;
  std::cout << "Moved rew_start = " << trajectory_generator_ptr->rew_start << std::endl;
  std::cout << "Moved rew_goal = " << trajectory_generator_ptr->rew_goal << std::endl;         
  std::cout << "Moved rw_start = " << trajectory_generator_ptr->rw_start << std::endl;
  std::cout << "Moved rw_goal = " << trajectory_generator_ptr->rw_goal << std::endl;

  DMP_ori_starts_goals.block(0, 0, 3, 1) = trajectory_generator_ptr->lrw_goal.transpose(); // 1 x 3
  DMP_ori_starts_goals.block(3, 0, 3, 1) = trajectory_generator_ptr->lrw_start.transpose();

  DMP_ori_starts_goals.block(6, 0, 3, 1) = trajectory_generator_ptr->lew_goal.transpose();
  DMP_ori_starts_goals.block(9, 0, 3, 1) = trajectory_generator_ptr->lew_start.transpose();

  DMP_ori_starts_goals.block(12, 0, 3, 1) = trajectory_generator_ptr->rew_goal.transpose();
  DMP_ori_starts_goals.block(15, 0, 3, 1) = trajectory_generator_ptr->rew_start.transpose();

  DMP_ori_starts_goals.block(18, 0, 3, 1) = trajectory_generator_ptr->rw_goal.transpose();
  DMP_ori_starts_goals.block(21, 0, 3, 1) = trajectory_generator_ptr->rw_start.transpose();

  dmp_vertex->setEstimate(DMP_ori_starts_goals);
  dmp_vertex->setId(0);
  optimizer.addVertex(dmp_vertex);    


  // connect to edge
  tracking_edge->setVertex(0, optimizer.vertex(0));

  
  std::cout << ">>>> Adding joint angle vertices, unary edges and tracking_error edges " << std::endl;  
  // Set non-colliding initial state for ease of collision avoidance
  Matrix<double, JOINT_DOF, 1> q_initial = Matrix<double, JOINT_DOF, 1>::Zero();
  q_initial.block(0, 0, 14, 1) << -1.5, -1.5, 1.5, 0.0, 0.0, 0.0, 0.0, 1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0; 
  // std::cout << "debug: new initial q = " << q_initial.transpose() << std::endl;
  for (unsigned int it = 0; it < NUM_DATAPOINTS; ++it)
  {
    // set finger joint values directly using linear mapping
    // get human data
    Matrix<double, 14, 1> l_finger_pos_human = trajectory_generator_ptr->l_glove_angle_traj.block(it, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
    Matrix<double, 14, 1> r_finger_pos_human = trajectory_generator_ptr->r_glove_angle_traj.block(it, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF
    // compute robot data through linear mapping
    Matrix<double, 12, 1> l_finger_pos_initial = tracking_edge->map_finger_joint_values(l_finger_pos_human, true, constraint_data);
    Matrix<double, 12, 1> r_finger_pos_initial = tracking_edge->map_finger_joint_values(r_finger_pos_human, false, constraint_data);
    // std::cout << "debug: l_finger_pos_initial = " << l_finger_pos_initial.transpose() << std::endl;
    // std::cout << "debug: r_finger_pos_initial = " << r_finger_pos_initial.transpose() << std::endl;
    // assign to initial data
    q_initial.block(14, 0, 12, 1) = l_finger_pos_initial;
    q_initial.block(26, 0, 12, 1) = r_finger_pos_initial;
    // std::cout << "debug: new initial q = " << q_initial.transpose() << std::endl;

    // add vertices
    //DualArmDualHandVertex *v = new DualArmDualHandVertex();
    v_list[it] = new DualArmDualHandVertex();
    v_list[it]->set_bounds(q_l_arm_lb, q_l_arm_ub, q_r_arm_lb, q_r_arm_ub, 
                           q_l_finger_lb, q_l_finger_ub, q_r_finger_lb, q_r_finger_ub); // set bounds for restraining updates
    // v_list[it]->set_collision_checker(dual_arm_dual_hand_collision_ptr); // set collision checker for use in OplusImpl()                                           
    

    v_list[it]->setEstimate(q_initial);
    
    v_list[it]->setId(1+it); // set a unique id
    optimizer.addVertex(v_list[it]);

    /*
    // set up path point info
    std::vector<double> l_wrist_pos = read_l_wrist_pos_traj[it]; // 3-dim
    std::vector<double> l_wrist_ori = read_l_wrist_ori_traj[it]; // 9-dim
    std::vector<double> l_elbow_pos = read_l_elbow_pos_traj[it]; //end()); // 3-dim
    std::vector<double> l_shoulder_pos = read_l_shoulder_pos_traj[it]; 
  
    std::vector<double> r_wrist_pos = read_r_wrist_pos_traj[it]; // 3-dim
    std::vector<double> r_wrist_ori = read_r_wrist_ori_traj[it]; // 9-dim
    std::vector<double> r_elbow_pos = read_r_elbow_pos_traj[it]; //end()); // 3-dim
    std::vector<double> r_shoulder_pos = read_r_shoulder_pos_traj[it]; 

    std::vector<double> l_finger_pos = read_l_finger_pos_traj[it];
    std::vector<double> r_finger_pos = read_r_finger_pos_traj[it];

    // convert to Eigen data type
    Vector3d l_wrist_pos_goal = Map<Vector3d>(l_wrist_pos.data(), 3, 1);
    Matrix3d l_wrist_ori_goal = Map<Matrix<double, 3, 3, RowMajor>>(l_wrist_ori.data(), 3, 3);
    Vector3d l_elbow_pos_goal = Map<Vector3d>(l_elbow_pos.data(), 3, 1);
    Vector3d l_shoulder_pos_goal = Map<Vector3d>(l_shoulder_pos.data(), 3, 1);

    Vector3d r_wrist_pos_goal = Map<Vector3d>(r_wrist_pos.data(), 3, 1);
    Matrix3d r_wrist_ori_goal = Map<Matrix<double, 3, 3, RowMajor>>(r_wrist_ori.data(), 3, 3);
    Vector3d r_elbow_pos_goal = Map<Vector3d>(r_elbow_pos.data(), 3, 1);
    Vector3d r_shoulder_pos_goal = Map<Vector3d>(r_shoulder_pos.data(), 3, 1);

    Matrix<double, 14, 1> l_finger_pos_goal = Map<Matrix<double, 14, 1>>(l_finger_pos.data(), 14, 1);
    Matrix<double, 14, 1> r_finger_pos_goal = Map<Matrix<double, 14, 1>>(r_finger_pos.data(), 14, 1);

    // Save in constraint_data for use in optimization
    constraint_data.l_wrist_pos_goal = l_wrist_pos_goal;
    constraint_data.l_wrist_ori_goal = l_wrist_ori_goal;
    constraint_data.l_elbow_pos_goal = l_elbow_pos_goal;
    constraint_data.l_shoulder_pos_goal = l_shoulder_pos_goal;

    constraint_data.r_wrist_pos_goal = r_wrist_pos_goal;
    constraint_data.r_wrist_ori_goal = r_wrist_ori_goal;
    constraint_data.r_elbow_pos_goal = r_elbow_pos_goal;
    constraint_data.r_shoulder_pos_goal = r_shoulder_pos_goal;

    constraint_data.l_finger_pos_goal = l_finger_pos_goal * M_PI / 180.0; // remember to convert from degree to radius!!!
    constraint_data.r_finger_pos_goal = r_finger_pos_goal * M_PI / 180.0;
    */


    // add unary edges
    // MyUnaryConstraints *unary_edge = new MyUnaryConstraints(dual_arm_dual_hand_collision_ptr);
    // unary_edge->setId(it+2); // similarity and tracking edges ahead of it
    // unary_edge->setVertex(0, optimizer.vertex(1+it)); //(0, v_list[it]); // set the 0th vertex on the edge to point to v_list[it]
    // unary_edge->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
    // unary_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
    // optimizer.addEdge(unary_edge);
    // unary_edges.push_back(unary_edge);
    
    // add tracking edges
    tracking_edge->setVertex(1+it, optimizer.vertex(1+it)); 
 
  }
  tracking_edge->setMeasurement(constraint_data);
  tracking_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
  optimizer.addEdge(tracking_edge);

  
  std::cout << ">>>> Adding binary edges: smoothness and collision costs " << std::endl;
  unsigned int num_col_checks = 50; // 100; // when set to 100, one iteration takes about 5-6 seconds
  // collision edge for the first path point, since this constraint only processes the latter vertex;
  // set number of check to 1, so as to perform only one collision checking on x0
  CollisionConstraint *first_collision_edge = new CollisionConstraint(dual_arm_dual_hand_collision_ptr, 1);
  first_collision_edge->setId(2); // similarity and tracking edges ahead of it
  first_collision_edge->setVertex(0, optimizer.vertex(1)); 
  first_collision_edge->setVertex(1, optimizer.vertex(1));   
  first_collision_edge->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
  first_collision_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
  
  collision_edges.push_back(first_collision_edge);

  optimizer.addEdge(first_collision_edge);
  for (unsigned int it = 0; it < NUM_DATAPOINTS - 1; ++it)
  {
    // add collision edges
    CollisionConstraint *collision_edge = new CollisionConstraint(dual_arm_dual_hand_collision_ptr, num_col_checks);
    collision_edge->setId(it+3); // the first one with id=2 is already included
    collision_edge->setVertex(0, optimizer.vertex(1+it)); //(0, v_list[it]); // set the 0th vertex on the edge to point to v_list[it]
    collision_edge->setVertex(1, optimizer.vertex(2+it)); 
    collision_edge->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
    collision_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
    optimizer.addEdge(collision_edge);

    collision_edges.push_back(collision_edge);
    
    // Add binary edges
    SmoothnessConstraint *smoothness_edge = new SmoothnessConstraint();
    smoothness_edge->setId(NUM_DATAPOINTS+2+it); // set a unique ID
    smoothness_edge->setVertex(0, optimizer.vertex(it+1)); //v_list[it]);
    smoothness_edge->setVertex(1, optimizer.vertex(it+2)); //v_list[it+1]); // binary edge, connects only 2 vertices, i.e. i=0 and i=1
    smoothness_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // Information type correct
    optimizer.addEdge(smoothness_edge);

    smoothness_edges.push_back(smoothness_edge);

  }
  

  std::cout << ">>>> Adding constraints for DMP starts and goals" << std::endl;
  dmp_edge = new DMPConstraints(trajectory_generator_ptr->lrw_goal, trajectory_generator_ptr->lrw_start,
                                trajectory_generator_ptr->lew_goal, trajectory_generator_ptr->lew_start,
                                trajectory_generator_ptr->rew_goal, trajectory_generator_ptr->rew_start,
                                trajectory_generator_ptr->rw_goal, trajectory_generator_ptr->rw_start);
  dmp_edge->setId(2*NUM_DATAPOINTS+1); 
  dmp_edge->setVertex(0, optimizer.vertex(0)); // DMP vertex
  //dmp_edge->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
  dmp_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
  optimizer.addEdge(dmp_edge);


  // Start optimization
  std::cout << ">>>> Start optimization:" << std::endl;
  optimizer.initializeOptimization();
  //optimizer.computeInitialGuess();   
  //optimizer.computeActiveErrors();

  std::cout << "optimizing graph: " << optimizer.vertices().size() << " vertices, " 
                                    << optimizer.edges().size() << " edges." << std::endl;

  std::vector<std::vector<double> > col_cost_history;
  // std::vector<std::vector<double> > pos_limit_cost_history;
  std::vector<std::vector<double> > wrist_pos_cost_history;
  std::vector<std::vector<double> > l_wrist_pos_cost_history;
  std::vector<std::vector<double> > r_wrist_pos_cost_history;

  std::vector<std::vector<double> > wrist_ori_cost_history;
  std::vector<std::vector<double> > elbow_pos_cost_history;
  std::vector<std::vector<double> > l_elbow_pos_cost_history;
  std::vector<std::vector<double> > r_elbow_pos_cost_history;
  
  std::vector<std::vector<double> > finger_cost_history;
  std::vector<std::vector<double> > l_finger_cost_history;
  std::vector<std::vector<double> > r_finger_cost_history;
  std::vector<std::vector<double> > smoothness_cost_history;
  std::vector<std::vector<double> > dmp_orien_cost_history;
  std::vector<std::vector<double> > dmp_scale_cost_history;
  std::vector<std::vector<double> > dmp_rel_change_cost_history;

  std::vector<std::vector<double> > track_jacobian_history; // for DMP, store in 2d
  std::vector<std::vector<double> > orien_jacobian_history; // for DMP orientation cost (vectors pointing from starts to goals)
  std::vector<std::vector<double> > scale_jacobian_history; // for DMP scale cost (scale margin of vectors)
  std::vector<std::vector<double> > rel_change_jacobian_history;

  std::vector<std::vector<double> > dmp_update_history;


  // Start optimization and store cost history
  std::cout << ">>>> Start optimization of the whole graph" << std::endl;

  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  
  unsigned int num_rounds = 1;//20;//1;//10;//20;//200;
  unsigned int dmp_per_iterations = 10;//5;//10;
  // unsigned int q_per_iterations = 50;//10;//30;//50;//5;//50;
  
  unsigned int q_trk_per_iterations = 20;//10;//20;//50;//300;//10;//20; //50;
  
  unsigned int max_round; // record for ease

  // coefficients search space
  // double K_COL_MAX = 100.0;//300.0;//100.0;//1000.0;//1000000.0;//200.0;//1000;//20.0;//15.0;//10.0;
  // double K_POS_LIMIT_MAX = 50.0;//30.0;//20;//20.0;//15.0;//10.0;
  // double K_SMOOTHNESS_MAX = 50.0;//30.0;//20.0;//15.0;//10.0;//10.0;

  double K_DMPSTARTSGOALS_MAX = 2.0;
  double K_DMPSCALEMARGIN_MAX = 2.0;
  double K_DMPRELCHANGE_MAX = 2.0;

  // double K_WRIST_POS_MAX = 30.0;//20.0;//10.0;//20;//50.0;//100.0;//20.0;//10.0;
  // double K_ELBOW_POS_MAX = 30.0;//20.0;//10.0;//20;//50.0;//100.0;//20.0;//10.0;
  // double K_WRIST_ORI_MAX = 30.0;//20.0;//10.0;//20;//50.0;//100.0;
  
  // Sets of selectable coefficients
  // Matrix<double, 20, 1> K_COL_set; K_COL_set << 0.0, 0.1, 0.5, 1.0, 1.5, 2.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0; //0.1, 0.5, 1.0, 2.5, 5.0, 10.0;
  Matrix<double, 35, 1> K_COL_set; //K_COL_set[0] = 0.0;
  double k_col_init = 0.1;
  for (unsigned int s = 0; s < 35; s++)
  {
    K_COL_set[s] = k_col_init;
    k_col_init = k_col_init * 1.5;
  }
  
  // Matrix<double, 2, 1> K_COL_set; K_COL_set << 0.0, 1.0;
  //K_COL_set = K_COL_set * 10; // set large penalty coefficient for bound (starts from colliding-free state?)
  // double k_pos_limit_init = 0.1;
  // Matrix<double, 15, 1> K_POS_LIMIT_set; //K_POS_LIMIT_set << 0.1, 1.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0; //0.1, 0.5, 1.0, 2.5, 5.0, 10.0;
  // for (unsigned int s = 0; s < 15; s++)
  // {
  //   K_POS_LIMIT_set[s] = k_pos_limit_init;
  //   k_pos_limit_init = k_pos_limit_init * 1.5;
  // }
  double k_smoothness_init = 0.1;
  Matrix<double, 15, 1> K_SMOOTHNESS_set; //K_SMOOTHNESS_set << 0.1, 1.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0; //0.1, 0.5, 1.0, 2.5, 5.0, 10.0;
  for (unsigned int s = 0; s < 15; s++)
  {
    K_SMOOTHNESS_set[s] = k_smoothness_init;
    k_smoothness_init = k_smoothness_init * 1.5;
  }


  Matrix<double, 10, 1> K_WRIST_POS_set; K_WRIST_POS_set << 0.1, 1.0, 3.0, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0;
  Matrix<double, 10, 1> K_WRIST_ORI_set; K_WRIST_ORI_set << 0.1, 1.0, 3.0, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0;
  Matrix<double, 10, 1> K_ELBOW_POS_set; K_ELBOW_POS_set << 0.1, 1.0, 3.0, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0;
  
  std::cout << ">>>> Selectable coefficients: ";
  std::cout << "K_COL = " << K_COL_set.transpose() << std::endl;
  // std::cout << "K_POS_LIMIT = " << K_POS_LIMIT_set.transpose() << std::endl;
  std::cout << "K_SMOOTHNESS = " << K_SMOOTHNESS_set.transpose() << std::endl;
  std::cout << "K_WRIST_POS = " << K_WRIST_POS_set.transpose() << std::endl;
  std::cout << "K_WRIST_ORI = " << K_WRIST_ORI_set.transpose() << std::endl;
  std::cout << "K_ELBOW_POS = " << K_ELBOW_POS_set.transpose() << std::endl;
  
  unsigned int id_k_col = 0;
  // unsigned int id_k_pos_limit = 0;
  unsigned int id_k_smoothness = 0;

  unsigned int id_k_wrist_pos = 0;
  unsigned int id_k_wrist_ori = 0;
  unsigned int id_k_elbow_pos = 0;


  // constraints bounds
  double eps = 1e-6; // numeric error
  double col_cost_bound = 0.5; // to cope with possible numeric error (here we still use check_self_collision() for estimating, because it might not be easy to keep minimum distance outside safety margin... )
  double smoothness_bound = std::sqrt(std::pow(3.0*M_PI/180.0, 2) * JOINT_DOF) * (NUM_DATAPOINTS-1); // in average, 3 degree allowable difference for each joint 
  // double pos_limit_bound = std::sqrt(std::pow(eps, 2) * JOINT_DOF); // 0.0, on account of numric error

  double dmp_orien_cost_bound = eps; // 0.0, on account of numeric error //0.0; // better be 0, margin is already set in it!!!
  double dmp_scale_cost_bound = eps; // 0.0, on account of numeric error 0.0; // better be 0
  double dmp_rel_change_cost_bound = eps; // 0.0, on account of numeric error//0.0; // better be 0

  double wrist_pos_cost_bound = std::sqrt( (std::pow(0.02, 2) * 3) ) * NUM_DATAPOINTS * 2; // 2 cm allowable error; note that for dual-arm, there are NUM_DATAPOINTS * 2 elbow position goals
  double elbow_pos_cost_bound = std::sqrt( (std::pow(0.03, 2) * 3) ) * NUM_DATAPOINTS * 2; // 3 cm allowable error;
  double wrist_ori_cost_bound = 5.0 * M_PI / 180.0 * NUM_DATAPOINTS * 2; // 5.0 degree allowable error, in radius; for dual-arm, there are NUM_DATAPOINTS * 2 wrist orientation goals!!!

  double scale = 1.5;  // increase coefficients by 20% 
  double outer_scale = 1.5;//2.0;
  double inner_scale = 1.5;//2.0;//1.5;


  // store initial DMP starts and goals
  // initialization
  Matrix<double, DMPPOINTS_DOF, 1> dmp_starts_goals_initial;
  std::vector<std::vector<double> > dmp_starts_goals_initial_vec_vec;
  std::vector<double> dmp_starts_goals_initial_vec(DMPPOINTS_DOF);
  // get data and convert
  DMPStartsGoalsVertex* vertex_tmp_initial = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));
  dmp_starts_goals_initial = vertex_tmp_initial->estimate();
  for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
      dmp_starts_goals_initial_vec[d] = dmp_starts_goals_initial[d];
  dmp_starts_goals_initial_vec_vec.push_back(dmp_starts_goals_initial_vec);
  // store and display results
  std::cout << "dmp_results size is: " << dmp_starts_goals_initial_vec_vec.size() << " x " << dmp_starts_goals_initial_vec_vec[0].size() << std::endl;
  std::cout << "dmp_results = " << dmp_starts_goals_initial.transpose() << std::endl;
  bool result_initial = write_h5(out_file_name, in_group_name, "dmp_starts_goals_initial", 
                                dmp_starts_goals_initial_vec_vec.size(), dmp_starts_goals_initial_vec_vec[0].size(), 
                                dmp_starts_goals_initial_vec_vec);
  std::cout << "initial dmp starts and goals stored " << (result_initial ? "successfully" : "unsuccessfully") << "!" << std::endl;


  for (unsigned int n = 0; n < num_rounds; n++)
  {
    // 1 - Optimize q vertices
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", q stage: "<< std::endl;

    // Fix DMP starts and goals
    std::cout << "Fix DMP starts and goals." << std::endl;
    DMPStartsGoalsVertex* dmp_vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));    
    dmp_vertex_tmp->setFixed(true);

    // Generate desired trajectories using DMP and pass into TrackingConstraint edge
    Matrix<double, DMPPOINTS_DOF, 1> x = dmp_vertex_tmp->estimate();
    MatrixXd lrw_new_goal(3, 1); lrw_new_goal = x.block(0, 0, 3, 1);
    MatrixXd lrw_new_start(3, 1); lrw_new_start = x.block(3, 0, 3, 1);
    MatrixXd lew_new_goal(3, 1); lew_new_goal = x.block(6, 0, 3, 1);
    MatrixXd lew_new_start(3, 1); lew_new_start = x.block(9, 0, 3, 1);
    MatrixXd rew_new_goal(3, 1); rew_new_goal = x.block(12, 0, 3, 1);
    MatrixXd rew_new_start(3, 1); rew_new_start = x.block(15, 0, 3, 1);
    MatrixXd rw_new_goal(3, 1); rw_new_goal = x.block(18, 0, 3, 1);
    MatrixXd rw_new_start(3, 1); rw_new_start = x.block(21, 0, 3, 1);
    // generate trajectories
    DMP_trajs result = trajectory_generator_ptr->generate_trajectories(lrw_new_goal.transpose(), lrw_new_start.transpose(), // should be row vectors
                                                                       lew_new_goal.transpose(), lew_new_start.transpose(),
                                                                       rew_new_goal.transpose(), rew_new_start.transpose(),
                                                                       rw_new_goal.transpose(), rw_new_start.transpose(),
                                                                       NUM_DATAPOINTS); // results are 3 x N
    // store in user data
    constraint_data.DMP_rw = result.y_rw;
    constraint_data.DMP_lw = result.y_lw;
    constraint_data.DMP_re = result.y_re;
    constraint_data.DMP_le = result.y_le;    
    // pass in DMP generated trajectories
    tracking_edge->setMeasurement(constraint_data);
    

    // signal flag for adjusting coefficients
    double col_cost_before_optim;
    double col_cost_after_optim;
    // double pos_limit_cost_before_optim;
    // double pos_limit_cost_after_optim;
    double smoothness_cost_before_optim;
    double smoothness_cost_after_optim;
    double wrist_pos_cost_before_optim;
    double wrist_pos_cost_after_optim;
    double elbow_pos_cost_before_optim;
    double elbow_pos_cost_after_optim;
    double wrist_ori_cost_before_optim;
    double wrist_ori_cost_after_optim;
    double finger_cost_before_optim;
    double finger_cost_after_optim;

    double ftol = 0.5;//0.1;//0.01;//0.005; //0.001; // update tolerance

    // for storing the best tracking result
    std::vector<Eigen::Matrix<double, JOINT_DOF, 1> > best_q(NUM_DATAPOINTS);    
    double best_elbow_pos_cost = 10000;
    double best_wrist_pos_cost = 10000;
    double best_wrist_ori_cost = 10000;
    double best_col_cost = NUM_DATAPOINTS;
    // double best_pos_limit_cost = 10000;
    double best_smoothness_cost = 10000;
    double best_dist = std::max(best_wrist_pos_cost - wrist_pos_cost_bound, 0.0) / wrist_pos_cost_bound + 
                       std::max(best_wrist_ori_cost - wrist_ori_cost_bound, 0.0) / wrist_ori_cost_bound +
                       std::max(best_elbow_pos_cost - elbow_pos_cost_bound, 0.0) / elbow_pos_cost_bound; // normalize it 
  
  
  // way 1 - Use TRAC-IK, solve IK for each path point, one by one  
  std::chrono::steady_clock::time_point t0_wrist_trac_ik_loop = std::chrono::steady_clock::now();
  // the results are set as initial state for later use
  std::vector<Eigen::Matrix<double, JOINT_DOF, 1> > q_initial_trac_ik(NUM_DATAPOINTS);
  std::vector<Eigen::Matrix<double, JOINT_DOF, 1> > q_initial_collision_fix(NUM_DATAPOINTS);
  Matrix<double, 7, 1> q_l_arm_lb_mat, q_l_arm_ub_mat;
  Matrix<double, 7, 1> q_r_arm_lb_mat, q_r_arm_ub_mat;
  for (unsigned int t = 0; t < q_l_arm_lb.size(); t++)
  {
    q_l_arm_lb_mat(t) = q_l_arm_lb[t];
    q_l_arm_ub_mat(t) = q_l_arm_ub[t];
    q_r_arm_lb_mat(t) = q_r_arm_lb[t];
    q_r_arm_ub_mat(t) = q_r_arm_ub[t];
  }
  unsigned int num_queries = NUM_DATAPOINTS * 2;
  unsigned int num_succeed = 0;
  for (unsigned int p = 0; p < NUM_DATAPOINTS; p++)
  {
    // get the current vertex
    DualArmDualHandVertex* q_vertex = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+p));
    Matrix<double, JOINT_DOF, 1> q_cur_whole = q_vertex->estimate();
    Matrix<double, JOINT_DOF, 1> q_last_whole;    

    // get initial q_arm values
    if (p == 0) // first vertex, use its own vertex
    {
      q_last_whole = q_cur_whole; 
    }
    else // not the first vertex, use the result of last path point !
    {
      q_last_whole = (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(p)))->estimate(); 
    }
    Matrix<double, 7, 1> q_cur_l, q_cur_r;
    q_cur_l = q_last_whole.block<7, 1>(0, 0);
    q_cur_r = q_last_whole.block<7, 1>(7, 0);

    // get goal
    Vector3d rw_pos_goal = result.y_rw.block(0, p, 3, 1);
    Vector3d lw_pos_goal = result.y_lw.block(0, p, 3, 1);
    Quaterniond q_l(trajectory_generator_ptr->l_wrist_quat_traj(p, 0), 
                    trajectory_generator_ptr->l_wrist_quat_traj(p, 1),
                    trajectory_generator_ptr->l_wrist_quat_traj(p, 2),
                    trajectory_generator_ptr->l_wrist_quat_traj(p, 3));
    Matrix3d lw_ori_goal = q_l.toRotationMatrix();
    Quaterniond q_r(trajectory_generator_ptr->r_wrist_quat_traj(p, 0), 
                    trajectory_generator_ptr->r_wrist_quat_traj(p, 1),
                    trajectory_generator_ptr->r_wrist_quat_traj(p, 2),
                    trajectory_generator_ptr->r_wrist_quat_traj(p, 3));
    Matrix3d rw_ori_goal = q_r.toRotationMatrix();
    
    // Perform TRAC-IK on the path point
    Matrix<double, 7, 1> q_res_l = q_cur_l;
    Matrix<double, 7, 1> q_res_r = q_cur_r;
    double timeout = 0.005; // doesn't make much difference with 0.01...
    int res_1 = run_trac_ik(q_cur_l, q_res_l, lw_pos_goal, lw_ori_goal, q_l_arm_lb_mat, q_l_arm_ub_mat, true, timeout);
    int res_2 = run_trac_ik(q_cur_r, q_res_r, rw_pos_goal, rw_ori_goal, q_r_arm_lb_mat, q_r_arm_ub_mat, false, timeout);

    if (res_1 >= 0)
      num_succeed++;
    if (res_2 >= 0)
      num_succeed++;

    // store the result
    q_cur_whole.block<7, 1>(0, 0) = q_res_l;
    q_cur_whole.block<7, 1>(7, 0) = q_res_r;
    q_vertex->setEstimate(q_cur_whole); // assign to the current q vertex
    q_initial_trac_ik[p] = q_cur_whole;

  }

  // record statistics
  // wrist pos
  std::vector<double> wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->BOTH_FLAG);
  double tmp_wrist_pos_cost = 0.0;
  for (unsigned s = 0; s < wrist_pos_cost.size(); s++)
    tmp_wrist_pos_cost += wrist_pos_cost[s];
  // wrist ori
  std::vector<double> wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history(tracking_edge->BOTH_FLAG);
  double tmp_wrist_ori_cost = 0.0;
  for (unsigned s = 0; s < wrist_ori_cost.size(); s++)
    tmp_wrist_ori_cost += wrist_ori_cost[s];        
  // elbow pos
  std::vector<double> elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->BOTH_FLAG);
  double tmp_elbow_pos_cost = 0.0;
  for (unsigned s = 0; s < elbow_pos_cost.size(); s++)
    tmp_elbow_pos_cost += elbow_pos_cost[s];
  // finger cost
  std::vector<double> finger_cost = tracking_edge->return_finger_cost_history(tracking_edge->BOTH_FLAG);
  double tmp_finger_pos_cost = 0.0;
  for (unsigned s = 0; s < finger_cost.size(); s++)
    tmp_finger_pos_cost += finger_cost[s];
  // collision 
  double tmp_col_cost = 0.0;
  for (unsigned int t = 0; t < collision_edges.size(); t++)
    tmp_col_cost += collision_edges[t]->return_col_cost(); // return_col_cost() is not affected by K_COL, just the count of collision path points
  // for (unsigned int t = 0; t < unary_edges.size(); t++)
    // tmp_col_cost += unary_edges[t]->return_col_cost(); // return_col_cost() is not affected by K_COL, just the count of collision path points
  // pos_limit
  // double tmp_pos_limit_cost = 0.0;
  // for (unsigned int t = 0; t < unary_edges.size(); t++)
    // tmp_pos_limit_cost += unary_edges[t]->return_pos_limit_cost();
  // smoothness
  double tmp_smoothness_cost = 0.0;
  for (unsigned int t = 0; t < smoothness_edges.size(); t++)
    tmp_smoothness_cost += smoothness_edges[t]->return_smoothness_cost();
  

  std::cout << std::endl << std::endl;  
  std::chrono::steady_clock::time_point t1_wrist_trac_ik_loop = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent_wrist_trac_ik_loop = std::chrono::duration_cast<std::chrono::duration<double>>(t1_wrist_trac_ik_loop - t0_wrist_trac_ik_loop);
  
  std::cout << ">>>> TRAC-IK done!" << std::endl;
  std::cout << "IK results: " << num_succeed << " of " << num_queries << " queries succeeded !" << std::endl;
  std::cout << "Cost: wrist_pos_cost = " << tmp_wrist_pos_cost << std::endl;
  std::cout << "Cost: wrist_ori_cost = " << tmp_wrist_ori_cost << std::endl;
  std::cout << "Cost: elbow_pos_cost = " << tmp_elbow_pos_cost << std::endl;
  std::cout << "Cost: finger_pos_cost = " << tmp_finger_pos_cost << std::endl;
  std::cout << "Cost: col_cost = " << tmp_col_cost << std::endl;
  // std::cout << "Cost: pos_limit_cost = " << tmp_pos_limit_cost << std::endl;
  std::cout << "Cost: smoothness_cost = " << tmp_smoothness_cost << std::endl;
  std::cout << "Total time spent: " << t_spent_wrist_trac_ik_loop.count() << " s." << std::endl;

  // store TRAC-IK result as the best for now
  std::cout << "Storing TRAC-IK result (as the best) for later comparison..." << std::endl;
  best_dist = std::max(tmp_wrist_pos_cost - wrist_pos_cost_bound, 0.0) / wrist_pos_cost_bound +
              std::max(tmp_wrist_ori_cost - wrist_ori_cost_bound, 0.0) / wrist_ori_cost_bound +
              std::max(tmp_elbow_pos_cost - elbow_pos_cost_bound, 0.0) / elbow_pos_cost_bound; 
  best_wrist_pos_cost = tmp_wrist_pos_cost;
  best_wrist_ori_cost = tmp_wrist_ori_cost;
  best_elbow_pos_cost = tmp_elbow_pos_cost;
  best_col_cost = tmp_col_cost;
  // best_pos_limit_cost = tmp_pos_limit_cost;
  best_smoothness_cost = tmp_smoothness_cost;
  // record the q values
  for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
  {
    DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
    best_q[s] = vertex_tmp->estimate();
  }


  // Solve collision 
  std::chrono::steady_clock::time_point t0_col_fix = std::chrono::steady_clock::now();  
  std::vector<double> finger_cost_tmp = tracking_edge->return_finger_cost_history(tracking_edge->BOTH_FLAG); 
  finger_cost_before_optim = 0.0;
  for (unsigned s = 0; s < finger_cost_tmp.size(); s++)
    finger_cost_before_optim += finger_cost_tmp[s];

  // perform collision fix directly on q vertices
  // process TRAC-IK results
  K_COL = 1.0; // set collision checker on!!!
  // best_q = unary_edges[0]->resolve_path_collisions(best_q);
  best_q = collision_edges[0]->resolve_path_collisions(best_q);


  // assign to q vertices
  for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
  {
    DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
    vertex_tmp->setEstimate(best_q[s]);
  }
  
  // Evaluate costs
  col_cost_after_optim = 0.0;
  for (unsigned int t = 0; t < collision_edges.size(); t++)
    col_cost_after_optim += collision_edges[t]->return_col_cost();
  // for (unsigned int t = 0; t < unary_edges.size(); t++)
    // col_cost_after_optim += unary_edges[t]->return_col_cost();
  // pos_limit
  // pos_limit_cost_after_optim = 0.0;
  // for (unsigned int t = 0; t < unary_edges.size(); t++)
  //   pos_limit_cost_after_optim += unary_edges[t]->return_pos_limit_cost();
  // smoothness
  smoothness_cost_after_optim = 0.0;
  for (unsigned int t = 0; t < smoothness_edges.size(); t++)
    smoothness_cost_after_optim += smoothness_edges[t]->return_smoothness_cost();
  // elbow pos
  std::vector<double> elbow_pos_cost_tmp = tracking_edge->return_elbow_pos_cost_history(tracking_edge->BOTH_FLAG);           
  elbow_pos_cost_after_optim = 0.0;
  for (unsigned s = 0; s < elbow_pos_cost_tmp.size(); s++)
    elbow_pos_cost_after_optim += elbow_pos_cost_tmp[s];
  // wrist pos 
  std::vector<double> wrist_pos_cost_tmp = tracking_edge->return_wrist_pos_cost_history(tracking_edge->BOTH_FLAG);
  wrist_pos_cost_after_optim = 0.0;
  for (unsigned s = 0; s < wrist_pos_cost_tmp.size(); s++)
    wrist_pos_cost_after_optim += wrist_pos_cost_tmp[s];
  // wrist ori
  std::vector<double> wrist_ori_cost_tmp = tracking_edge->return_wrist_ori_cost_history(tracking_edge->BOTH_FLAG);
  wrist_ori_cost_after_optim = 0.0;
  for (unsigned s = 0; s < wrist_ori_cost_tmp.size(); s++)
    wrist_ori_cost_after_optim += wrist_ori_cost_tmp[s]; 
  // finger pos
  finger_cost_tmp = tracking_edge->return_finger_cost_history(tracking_edge->BOTH_FLAG); 
  finger_cost_after_optim = 0.0;
  for (unsigned s = 0; s < finger_cost_tmp.size(); s++)
    finger_cost_after_optim += finger_cost_tmp[s];

  std::chrono::steady_clock::time_point t1_col_fix = std::chrono::steady_clock::now();  
  std::chrono::duration<double> t_spent_col_fix = std::chrono::duration_cast<std::chrono::duration<double>>(t1_col_fix - t0_col_fix);
  
  std::cout << ">>>> Collision fix done!" << std::endl;
  // std::cout << "Costs: col_cost = " << col_cost_after_optim << ", pos_limit_cost = " << pos_limit_cost_after_optim << ", smoothness_cost = " << smoothness_cost_after_optim << std::endl;
  std::cout << "Costs: col_cost = " << col_cost_after_optim << ", smoothness_cost = " << smoothness_cost_after_optim << std::endl;
  std::cout << "Costs: wrist_pos_cost = " << wrist_pos_cost_after_optim << std::endl;
  std::cout << "Costs: wrist_ori_cost = " << wrist_ori_cost_after_optim << std::endl;
  std::cout << "Costs: elbow_pos_cost = " << elbow_pos_cost_after_optim << std::endl;
  std::cout << "Costs: finger_cost = " << finger_cost_after_optim << std::endl;  
  std::cout << "Time spent for collision fix is " << t_spent_col_fix.count() << " s." << std::endl;
  if (col_cost_after_optim >= col_cost_bound)
    std::cerr << "Collision path points are not all resolved, re-adjust the process!!!" << std::endl;


  // store Collision-fix result as the best for now
  std::cout << "Storing Collision-fix results (as the best) for later comparison..." << std::endl;
  best_dist = std::max(wrist_pos_cost_after_optim - wrist_pos_cost_bound, 0.0) / wrist_pos_cost_bound +
              std::max(wrist_ori_cost_after_optim - wrist_ori_cost_bound, 0.0) / wrist_ori_cost_bound +
              std::max(elbow_pos_cost_after_optim - elbow_pos_cost_bound, 0.0) / elbow_pos_cost_bound; 
  best_wrist_pos_cost = wrist_pos_cost_after_optim;
  best_wrist_ori_cost = wrist_ori_cost_after_optim;
  best_elbow_pos_cost = elbow_pos_cost_after_optim;
  best_col_cost = col_cost_after_optim;
  // best_pos_limit_cost = pos_limit_cost_after_optim;
  best_smoothness_cost = smoothness_cost_after_optim;
  // record the q values
  for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
  {
    DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
    best_q[s] = vertex_tmp->estimate();
    q_initial_collision_fix[s] = vertex_tmp->estimate();
  }


  // set collision checker on, so as to count the number of colliding path points
  // id_k_col = 1;

  // Start optimization, using different combinations of coefficients
  double t_spent_inner_loop = 0.0;
  double t_spent_outer_loop = 0.0;
  unsigned int count_inner_loop = 0;
  unsigned int count_outer_loop = 0;
  do // Outer loop, resolve collision, pos limit and smoothness first
  {
      count_outer_loop++;
      std::chrono::steady_clock::time_point t0_outer_loop = std::chrono::steady_clock::now();

      // Report condition
      std::cout << ">>>> Outer loop: adjust K_WRIST_POS, K_WRIST_ORI and K_ELBOW_POS." << std::endl;


      // Evaluate cost before optimization,
      // elbow pos
      std::vector<double> elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->BOTH_FLAG);           
      elbow_pos_cost_before_optim = 0.0;
      for (unsigned s = 0; s < elbow_pos_cost.size(); s++)
        elbow_pos_cost_before_optim += elbow_pos_cost[s];
      // wrist pos 
      std::vector<double> wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->BOTH_FLAG);
      wrist_pos_cost_before_optim = 0.0;
      for (unsigned s = 0; s < wrist_pos_cost.size(); s++)
        wrist_pos_cost_before_optim += wrist_pos_cost[s];
      // wrist ori
      std::vector<double> wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history(tracking_edge->BOTH_FLAG);
      wrist_ori_cost_before_optim = 0.0;
      for (unsigned s = 0; s < wrist_ori_cost.size(); s++)
        wrist_ori_cost_before_optim += wrist_ori_cost[s];  


      // Reset K_COL, K_POS_LIMIT and K_SMOOTHNESS (maybe it's ok to keep them after the inner loop finishes, so as to reduce time usage)
      // id_k_col = 0;
      // id_k_pos_limit = 0;
      // id_k_smoothness = 0;


      // Reset/Use initial trajector computed by collision fix 
      for (unsigned int id = 0; id < NUM_DATAPOINTS; id++)
        // (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+id)))->setEstimate(q_initial_trac_ik[id]); // assign to the current q vertex
        (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+id)))->setEstimate(q_initial_collision_fix[id]); // assign to the current q vertices

      do // Inner loop
      {

        std::chrono::steady_clock::time_point t0_inner_loop = std::chrono::steady_clock::now();

        count_inner_loop++;

        // Set coefficients
        K_COL = K_COL_set[(id_k_col <= K_COL_set.size()-1 ? id_k_col : K_COL_set.size()-1)];
        // K_COL = 0.0; // set collision checker off
        // K_POS_LIMIT = K_POS_LIMIT_set[(id_k_pos_limit <= K_POS_LIMIT_set.size()-1 ? id_k_pos_limit : K_POS_LIMIT_set.size()-1)];
        K_SMOOTHNESS = K_SMOOTHNESS_set[(id_k_smoothness <= K_SMOOTHNESS_set.size()-1 ? id_k_smoothness : K_SMOOTHNESS_set.size()-1)];
        K_WRIST_POS = K_WRIST_POS_set[(id_k_wrist_pos <= K_WRIST_POS_set.size()-1 ? id_k_wrist_pos : K_WRIST_POS_set.size()-1)];
        K_WRIST_ORI = K_WRIST_ORI_set[(id_k_wrist_ori <= K_WRIST_ORI_set.size()-1 ? id_k_wrist_ori : K_WRIST_ORI_set.size()-1)];
        K_ELBOW_POS = K_ELBOW_POS_set[(id_k_elbow_pos <= K_ELBOW_POS_set.size()-1 ? id_k_elbow_pos : K_ELBOW_POS_set.size()-1)];


        // Evaluate costs before optimization
        // collision counts
        col_cost_before_optim = 0.0;
        for (unsigned int t = 0; t < collision_edges.size(); t++)
          col_cost_before_optim += collision_edges[t]->return_col_cost();
        // for (unsigned int t = 0; t < unary_edges.size(); t++)
        //   col_cost_before_optim += unary_edges[t]->return_col_cost(); // not affected by K_COL, no need to worry
        // pos_limit
        // pos_limit_cost_before_optim = 0.0;
        // for (unsigned int t = 0; t < unary_edges.size(); t++)
        //   pos_limit_cost_before_optim += unary_edges[t]->return_pos_limit_cost();
        // smoothness
        smoothness_cost_before_optim = 0.0;
        for (unsigned int t = 0; t < smoothness_edges.size(); t++)
          smoothness_cost_before_optim += smoothness_edges[t]->return_smoothness_cost();


        // Report current condition
        std::cout << ">>>> Inner loop: adjust K_COL and K_SMOOTHNESS." << std::endl;
        // std::cout << "Current coefficients: K_COL = " << K_COL << ", K_POS_LIMIT = " << K_POS_LIMIT << ", K_SMOOTHNESS = " << K_SMOOTHNESS << std::endl;
        std::cout << "Current coefficients: K_COL = " << K_COL << ", K_SMOOTHNESS = " << K_SMOOTHNESS << std::endl;
        std::cout << "Costs before optimization: col_cost = " << col_cost_before_optim 
                  // << ", pos_limit_cost = " << pos_limit_cost_before_optim 
                  << ", smoothness_cost = " << smoothness_cost_before_optim << std::endl;
        std::cout << "Current coefficient: K_ELBOW_POS = " << K_ELBOW_POS << std::endl;
        std::cout << "Current coefficient: K_WRIST_POS = " << K_WRIST_POS << std::endl;
        std::cout << "Current coefficient: K_WRIST_ORI = " << K_WRIST_ORI << std::endl;

 
        // optimize for a few iterations
        std::cout << "Optimizing q..." << std::endl;
        unsigned int q_iter = optimizer.optimize(q_trk_per_iterations); 


        // Store cost results
        std::cout << ">> Costs <<" << std::endl;
        // collision 
        std::vector<double> col_cost;
        for (unsigned int t = 0; t < collision_edges.size(); t++)
          col_cost.push_back(collision_edges[t]->return_col_cost()); // store
        // for (unsigned int t = 0; t < unary_edges.size(); t++)
        //   col_cost.push_back(unary_edges[t]->return_col_cost()); // store
        col_cost_history.push_back(col_cost);
        print_debug_output("col_cost", col_cost);
        // pos_limit
        // std::vector<double> pos_limit_cost;
        // for (unsigned int t = 0; t < unary_edges.size(); t++)
        //   pos_limit_cost.push_back(unary_edges[t]->return_pos_limit_cost()); // store
        // pos_limit_cost_history.push_back(pos_limit_cost);
        // print_debug_output("pos_limit_cost", pos_limit_cost);
        // smoothness
        std::vector<double> smoothness_cost;
        for (unsigned int t = 0; t < smoothness_edges.size(); t++)
          smoothness_cost.push_back(smoothness_edges[t]->return_smoothness_cost()); // store
        smoothness_cost_history.push_back(smoothness_cost);  
        print_debug_output("smoothness_cost", smoothness_cost);
        // tracking   
        std::vector<double> l_wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->LEFT_FLAG);
        std::vector<double> r_wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->RIGHT_FLAG);
        std::vector<double> l_elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->LEFT_FLAG);
        std::vector<double> r_elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->RIGHT_FLAG);
        std::vector<double> l_finger_cost = tracking_edge->return_finger_cost_history(tracking_edge->LEFT_FLAG); 
        std::vector<double> r_finger_cost = tracking_edge->return_finger_cost_history(tracking_edge->RIGHT_FLAG); 
        wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->BOTH_FLAG);
        wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history(tracking_edge->BOTH_FLAG);
        elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->BOTH_FLAG);
        std::vector<double> finger_cost = tracking_edge->return_finger_cost_history(tracking_edge->BOTH_FLAG); 
        wrist_pos_cost_history.push_back(wrist_pos_cost);
        l_wrist_pos_cost_history.push_back(l_wrist_pos_cost);
        r_wrist_pos_cost_history.push_back(r_wrist_pos_cost);
        wrist_ori_cost_history.push_back(wrist_ori_cost);
        elbow_pos_cost_history.push_back(elbow_pos_cost);
        l_elbow_pos_cost_history.push_back(l_elbow_pos_cost);
        r_elbow_pos_cost_history.push_back(r_elbow_pos_cost);
        finger_cost_history.push_back(finger_cost); // store
        l_finger_cost_history.push_back(l_finger_cost);
        r_finger_cost_history.push_back(r_finger_cost);        
        // display
        print_debug_output("wrist_pos_cost", wrist_pos_cost);
        print_debug_output("l_wrist_pos_cost", l_wrist_pos_cost);
        print_debug_output("r_wrist_pos_cost", r_wrist_pos_cost);
        print_debug_output("wrist_ori_cost", wrist_ori_cost);
        print_debug_output("elbow_pos_cost", elbow_pos_cost);
        print_debug_output("l_elbow_pos_cost", l_elbow_pos_cost);
        print_debug_output("r_elbow_pos_cost", r_elbow_pos_cost);
        print_debug_output("finger_cost", finger_cost);
 
        // Store jacobians results
        std::cout << ">> Jacobians <<" << std::endl;
        // output jacobians of wrist_pos, wrist_ori and elbow_pos costs for q vertices
        Matrix<double, 14, NUM_DATAPOINTS> wrist_pos_jacobian_for_q_arm;
        Matrix<double, 14, NUM_DATAPOINTS> wrist_ori_jacobian_for_q_arm;
        Matrix<double, 14, NUM_DATAPOINTS> elbow_pos_jacobian_for_q_arm;
        Matrix<double, 24, NUM_DATAPOINTS> finger_pos_jacobian_for_q_finger;
        wrist_pos_jacobian_for_q_arm = tracking_edge->wrist_pos_jacobian_for_q_arm;
        wrist_ori_jacobian_for_q_arm = tracking_edge->wrist_ori_jacobian_for_q_arm;
        elbow_pos_jacobian_for_q_arm = tracking_edge->elbow_pos_jacobian_for_q_arm;
        finger_pos_jacobian_for_q_finger = tracking_edge->finger_pos_jacobian_for_q_finger;
        std::cout << "Norms of wrist_pos_jacobian_for_q_arm = ";
        for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        {
          std::cout << wrist_pos_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
        }
        std::cout << std::endl << std::endl;    
        std::cout << "Norms of wrist_ori_jacobian_for_q_arm = ";
        for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        {
          std::cout << wrist_ori_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
        }
        std::cout << std::endl << std::endl;    
        std::cout << "Norms of elbow_pos_jacobian_for_q_arm = ";
        for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        {
          std::cout << elbow_pos_jacobian_for_q_arm.block(0, n, 14, 1).norm() << " ";
        }
        std::cout << std::endl << std::endl;    
        std::cout << "Norms of finger_pos_jacobian_for_q_finger = ";
        for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        {
          std::cout << finger_pos_jacobian_for_q_finger.block(0, n, 24, 1).norm() << " ";
        }
        std::cout << std::endl << std::endl;           
        // output collision jacobians for debug
        std::cout << "Norms of col_jacobians = ";
        for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        {
          // std::cout << unary_edges[n]->col_jacobians.norm() << " ";
          std::cout << collision_edges[n]->col_jacobians.norm() << " ";
        }
        std::cout << std::endl << std::endl;    
        // output position limit jacobians for debug
        // std::cout << "Norms of pos_limit_jacobians = ";
        // for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        // {
        //   std::cout << unary_edges[n]->pos_limit_jacobians.norm() << " ";
        // }
        // std::cout << std::endl << std::endl;    
        // output jacobians of Smoothness edge for q vertices
        std::cout << "Norms of smoothness_q_jacobian = ";
        Matrix<double, 2, JOINT_DOF> smoothness_q_jacobian;
        for (unsigned int n = 0; n < NUM_DATAPOINTS-1; n++)
        {
          smoothness_q_jacobian = smoothness_edges[n]->output_q_jacobian();
          std::cout << smoothness_q_jacobian.norm() << " ";
        }
        std::cout << std::endl << std::endl;  
        // output jacobians of unary edges for q vertices
        // std::cout << "Norms of unary_jacobians = ";
        // for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        // {
        //   // std::cout << unary_edges[n]->whole_jacobians.norm() << " ";
        //   std::cout << collision_edges[n]->whole_jacobians.norm() << " ";
        // }
        // std::cout << std::endl << std::endl;    
        // output jacobians of Tracking edge for q vertices
        std::cout << "Norms of tracking_q_jacobians = ";
        Matrix<double, NUM_DATAPOINTS, JOINT_DOF> q_jacobian = tracking_edge->output_q_jacobian();
        for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        {
          std::cout << q_jacobian.block(n, 0, 1, JOINT_DOF).norm() << " ";
        }
        std::cout << std::endl << std::endl;    


        // Evaluate costs after optimization, for loop termination checks
        // wrist pos
        wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->BOTH_FLAG);
        wrist_pos_cost_after_optim = 0.0;
        for (unsigned s = 0; s < wrist_pos_cost.size(); s++)
          wrist_pos_cost_after_optim += wrist_pos_cost[s];
        // wrist ori
        wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history(tracking_edge->BOTH_FLAG);
        wrist_ori_cost_after_optim = 0.0;
        for (unsigned s = 0; s < wrist_ori_cost.size(); s++)
          wrist_ori_cost_after_optim += wrist_ori_cost[s];
        // elbow pos
        elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->BOTH_FLAG);
        elbow_pos_cost_after_optim = 0.0;
        for (unsigned s = 0; s < elbow_pos_cost.size(); s++)
          elbow_pos_cost_after_optim += elbow_pos_cost[s];
        // collision counts
        col_cost_after_optim = 0.0;
        for (unsigned int t = 0; t < collision_edges.size(); t++)
          col_cost_after_optim += collision_edges[t]->return_col_cost();
        // for (unsigned int t = 0; t < unary_edges.size(); t++)
        //   col_cost_after_optim += unary_edges[t]->return_col_cost();
        // pos_limit
        // pos_limit_cost_after_optim = 0.0;
        // for (unsigned int t = 0; t < unary_edges.size(); t++)
        //   pos_limit_cost_after_optim += unary_edges[t]->return_pos_limit_cost();
        // smoothness
        smoothness_cost_after_optim = 0.0;
        for (unsigned int t = 0; t < smoothness_edges.size(); t++)
          smoothness_cost_after_optim += smoothness_edges[t]->return_smoothness_cost();


        // Adjust K_COL
        if (col_cost_after_optim > col_cost_bound && id_k_col <= K_COL_set.size() - 1 ) 
        { 
          if (col_cost_before_optim - col_cost_after_optim <= 2.0) // if it's not descending or not descending fast enough, increase the coefficient
          {
            if (id_k_col < K_COL_set.size() - 1)
              std::cout << "Coefficient: K_COL = " << K_COL_set[id_k_col] << " ----> " << K_COL_set[id_k_col+1] << std::endl;
            else
              std::cout << "Coefficient: K_COL = " << K_COL_set[id_k_col] << " (Reached the end) " << std::endl;
            id_k_col++; // move to next sample point
          }
          else
          {
            std::cout << "Coefficient: K_COL = " << K_COL_set[id_k_col] << std::endl;
          }
          std::cout << "Cost: col_cost = " << col_cost_before_optim << " ----> " << col_cost_after_optim 
                                          << "(" << (col_cost_before_optim > col_cost_after_optim ? "-" : "+")
                                          << std::abs(col_cost_before_optim - col_cost_after_optim) << ")" 
                                          << " (bound: " << col_cost_bound << ")" << std::endl;
        }
        else
        {
          std::cout << "Coefficient: K_COL = " << K_COL << std::endl;
          std::cout << "Cost: col_cost = " << col_cost_after_optim << " (bound: " << col_cost_bound << ")" << std::endl;
        }

        // Adjust K_SMOOTHNESS
        if (smoothness_cost_after_optim > smoothness_bound && id_k_smoothness <= K_SMOOTHNESS_set.size() - 1) 
        {
          if (smoothness_cost_before_optim - smoothness_cost_after_optim <= ftol) // if not descending or not descending fast enough, increase the K
          {
            if (id_k_smoothness < K_SMOOTHNESS_set.size() - 1)
              std::cout << "Coefficient: K_SMOOTHNESS = " << K_SMOOTHNESS_set[id_k_smoothness] << " ----> " << K_SMOOTHNESS_set[id_k_smoothness+1] << std::endl;
            else
              std::cout << "Coefficient: K_SMOOTHNESS = " << K_SMOOTHNESS_set[id_k_smoothness] << " (Reached the end) " << std::endl;
            id_k_smoothness++;
          }
          else
          {
            std::cout << "Coefficient: K_SMOOTHNESS = " << K_SMOOTHNESS_set[id_k_smoothness] << std::endl;
          }
          std::cout << "Cost: smoothness_cost = " << smoothness_cost_before_optim << " ----> " << smoothness_cost_after_optim 
                                          << "(" << (smoothness_cost_before_optim > smoothness_cost_after_optim ? "-" : "+")
                                          << std::abs(smoothness_cost_before_optim - smoothness_cost_after_optim) << ")" 
                                          << " (bound: " << smoothness_bound << ")" << std::endl;
        }
        else
        {
          std::cout << "Coefficient: K_SMOOTHNESS = " << K_SMOOTHNESS << std::endl;
          std::cout << "Cost: smoothness_cost = " << smoothness_cost_after_optim << " (bound: " << smoothness_bound << ")" << std::endl;
        }

        // Adjust K_POS_LIMIT
        /*
        if (pos_limit_cost_after_optim > pos_limit_bound && id_k_pos_limit <= K_POS_LIMIT_set.size() - 1) 
        {
          if (pos_limit_cost_before_optim - pos_limit_cost_after_optim <= ftol) // if not descending, or not descending fast enough, increase the K
          {
            if (id_k_pos_limit < K_POS_LIMIT_set.size() - 1)
              std::cout << "Coefficient: K_POS_LIMIT = " << K_POS_LIMIT_set[id_k_pos_limit] << " ----> " << K_POS_LIMIT_set[id_k_pos_limit+1] << std::endl;
            else
              std::cout << "Coefficient: K_POS_LIMIT = " << K_POS_LIMIT_set[id_k_pos_limit] << " (Reached the end) " << std::endl;
            id_k_pos_limit++;
          }
          else
          {
            std::cout << "Coefficient: K_POS_LIMIT = " << K_POS_LIMIT_set[id_k_pos_limit] << std::endl;
          }
          std::cout << "Cost: pos_limit_cost = " << pos_limit_cost_before_optim << " ----> " << pos_limit_cost_after_optim 
                                          << "(" << (pos_limit_cost_before_optim > pos_limit_cost_after_optim ? "-" : "+")
                                          << std::abs(pos_limit_cost_before_optim - pos_limit_cost_after_optim) << ")" 
                                          << " (bound: " << pos_limit_bound << ")" << std::endl;
        }
        else
        {
          std::cout << "Coefficient: K_POS_LIMIT = " << K_POS_LIMIT << std::endl;
          std::cout << "Cost: pos_limit_cost = " << pos_limit_cost_after_optim << " (bound: " << pos_limit_bound << ")" << std::endl;
        }
        */

        // Checking collision situation
        /*
        std::cout << ">> Collision condition <<" << std::endl;
        for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
        {
          std::cout << ">> Path point " << (n+1) << "/" << NUM_DATAPOINTS << " <<" << std::endl;
          unary_edges[n]->output_distance_result();
          std::cout << "col_jacobians = " << unary_edges[n]->col_jacobians.transpose() << std::endl;
        }
        std::cout << "last q = " << (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(NUM_DATAPOINTS)))->estimate().transpose() << std::endl;
        */


        // debug: time usage:
        std::cout << ">> Time usage for tracking:" << std::endl;
        std::cout << "Trajectory generation " << count_traj 
                << " times, with total time = " << total_traj 
                << " s, average time = " << total_traj / count_traj << " s." << std::endl;
        std::cout << "Trackig constraint " << count_tracking
                << " times, with total time = " << total_tracking 
                << " s, average time = " << total_tracking / count_tracking << " s." << std::endl;                                                
        // reset count and time
        count_tracking = 0;
        count_traj = 0;
        total_tracking = 0.0;
        total_traj = 0.0; 

      std::chrono::steady_clock::time_point t1_inner_loop = std::chrono::steady_clock::now();
      std::chrono::duration<double> t_spent_inner_loop_cur = std::chrono::duration_cast<std::chrono::duration<double>>(t1_inner_loop - t0_inner_loop);
      t_spent_inner_loop += t_spent_inner_loop_cur.count();

      std::cout << "> Time Usage:" << std::endl;
      std::cout << "Time used for current inner loop is: " << t_spent_inner_loop_cur.count() << " s." << std::endl;
      
      }while( (id_k_col <= (K_COL_set.size()-1) && col_cost_after_optim > col_cost_bound) || 
              (id_k_smoothness <= (K_SMOOTHNESS_set.size()-1) && smoothness_cost_after_optim > smoothness_bound) );// ||
              // (id_k_pos_limit <= (K_POS_LIMIT_set.size()-1) && pos_limit_cost_after_optim > pos_limit_bound)); // when coefficients still within bounds, and costs still out of bounds
      
      std::cout << ">>>> End of Inner loop" << std::endl << std::endl;


      // Check if better than current best
      // Check if the best result from wrist pos+ori loop satisfies the bounds
      std::cout << ">>>> Evaluating feasibility of the processed paths..." << std::endl;
      if (col_cost_after_optim <= col_cost_bound)// &&  // relax a few
          //pos_limit_cost_after_optim <= pos_limit_bound && 
          //smoothness_cost_after_optim <= smoothness_bound) // if satisfying constraints (there may be no need for smoothness cost to be bounded..)
      {
        std::cout << "Feasible paths !!!" << std::endl;
        // check if better than the history-best result
        double tmp_dist = std::max(wrist_pos_cost_after_optim - wrist_pos_cost_bound, 0.0) / wrist_pos_cost_bound +
                          std::max(wrist_ori_cost_after_optim - wrist_ori_cost_bound, 0.0) / wrist_ori_cost_bound +
                          std::max(elbow_pos_cost_after_optim - elbow_pos_cost_bound, 0.0) / elbow_pos_cost_bound; // normalize 
        // if (tmp_dist < best_dist)
        if (wrist_pos_cost_after_optim <= wrist_pos_cost_bound && 
            wrist_ori_cost_after_optim <= wrist_ori_cost_bound &&
            elbow_pos_cost_after_optim < best_elbow_pos_cost) // wrist costs mush be within bounds, and choose the one with the smallest elbow pos cost
        {
          std::cout << "Found a result better than the best ever! Recording..." << std::endl;
          // record the costs
          best_dist = tmp_dist;
          best_wrist_pos_cost = wrist_pos_cost_after_optim;
          best_wrist_ori_cost = wrist_ori_cost_after_optim;
          best_elbow_pos_cost = elbow_pos_cost_after_optim;
          best_col_cost = col_cost_after_optim;
          // best_pos_limit_cost = pos_limit_cost_after_optim;
          best_smoothness_cost = smoothness_cost_after_optim;
          // record the q values
          for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
          {
            DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
            best_q[s] = vertex_tmp->estimate();
          }
        }
      }
      else
      {
        std::cout << "Non-feasible paths, continue to next round, or modify the sets of coefficients !!!" << std::endl;
      }


    // Select different combinations of K_WRIST_POS, K_WRIST_ORI and K_ELBOW_POS
    if (wrist_pos_cost_after_optim > wrist_pos_cost_bound && id_k_wrist_pos <= K_WRIST_POS_set.size() - 1)
    {
      if (wrist_pos_cost_before_optim - wrist_pos_cost_after_optim <= ftol) // require the update to be higher than a tolerance
      {
        if (id_k_wrist_pos < K_WRIST_POS_set.size() - 1)
          std::cout << "Coefficient: K_WRIST_POS = " << K_WRIST_POS_set[id_k_wrist_pos] << " ----> " << K_WRIST_POS_set[id_k_wrist_pos+1] << std::endl;
        else
          std::cout << "Coefficient: K_WRIST_POS = " << K_WRIST_POS_set[id_k_wrist_pos] << " (Reached the end) " << std::endl;
        id_k_wrist_pos++;
      }
      else
      {
        std::cout << "Coefficient: K_WRIST_POS = " << K_WRIST_POS_set[id_k_wrist_pos] << std::endl;
      }
      std::cout << "Cost: wrist_pos_cost = " << wrist_pos_cost_before_optim << " ----> " << wrist_pos_cost_after_optim 
                                              << "(" << (wrist_pos_cost_before_optim > wrist_pos_cost_after_optim ? "-" : "+")
                                              << std::abs(wrist_pos_cost_before_optim - wrist_pos_cost_after_optim) << ")" 
                                              << " (bound: " << wrist_pos_cost_bound << ")" << std::endl;
    }
    else
    {
      std::cout << "Coefficient: K_WRIST_POS = " << K_WRIST_POS << std::endl;
      std::cout << "Cost: wrist_pos_cost = " << wrist_pos_cost_after_optim << " (bound: " << wrist_pos_cost_bound << ")" << std::endl;
    }

    // Adjust K_WRIST_ORI
    if (wrist_ori_cost_after_optim > wrist_ori_cost_bound && id_k_wrist_ori <= K_WRIST_ORI_set.size() - 1)
    {
      if (wrist_ori_cost_before_optim - wrist_ori_cost_after_optim <= ftol) // require the update to be higher than a tolerance
      {
        if (id_k_wrist_ori < K_WRIST_ORI_set.size() - 1)
          std::cout << "Coefficient: K_WRIST_ORI = " << K_WRIST_ORI_set[id_k_wrist_ori] << " ----> " << K_WRIST_ORI_set[id_k_wrist_ori+1] << std::endl;
        else
          std::cout << "Coefficient: K_WRIST_ORI = " << K_WRIST_ORI_set[id_k_wrist_ori] << " (Reached the end) " << std::endl;        
        id_k_wrist_ori++;
      }
      else                                                           
      {
        std::cout << "Coefficient: K_WRIST_ORI = " << K_WRIST_ORI_set[id_k_wrist_ori] << std::endl;
      }
      std::cout << "Cost: wrist_ori_cost = " << wrist_ori_cost_before_optim << " ----> " << wrist_ori_cost_after_optim 
                                              << "(" << (wrist_ori_cost_before_optim > wrist_ori_cost_after_optim ? "-" : "+")
                                              << std::abs(wrist_ori_cost_before_optim - wrist_ori_cost_after_optim) << ")" 
                                              << " (bound: " << wrist_ori_cost_bound << ")" << std::endl;
    }
    else 
    {
      std::cout << "Coefficient: K_WRIST_ORI = " << K_WRIST_ORI << std::endl;
      std::cout << "Cost: wrist_ori_cost = " << wrist_ori_cost_after_optim << " (bound: " << wrist_ori_cost_bound << ")" << std::endl;
    }

      // Adjust K_ELBOW_POS
      if (elbow_pos_cost_after_optim > elbow_pos_cost_bound && id_k_elbow_pos <= K_ELBOW_POS_set.size() - 1)
      {
        if (elbow_pos_cost_before_optim - elbow_pos_cost_after_optim <= ftol) // require the update to be higher than a tolerance
        {
          if (id_k_elbow_pos < K_ELBOW_POS_set.size() - 1)
            std::cout << "Coefficient: K_ELBOW_POS = " << K_ELBOW_POS_set[id_k_elbow_pos] << " ----> " << K_ELBOW_POS_set[id_k_elbow_pos+1] << std::endl;            
          else
            std::cout << "Coefficient: K_ELBOW_POS = " << K_ELBOW_POS_set[id_k_elbow_pos] << " (Reached the end) " << std::endl;            
          id_k_elbow_pos++;
        }
        else
        {
          std::cout << "Coefficient: K_ELBOW_POS = " << K_ELBOW_POS_set[id_k_elbow_pos] << std::endl;
        }
        std::cout << "Cost: elbow_pos_cost = " << elbow_pos_cost_before_optim << " ----> " << elbow_pos_cost_after_optim 
                                                    << "(" << (elbow_pos_cost_before_optim > elbow_pos_cost_after_optim ? "-" : "+")
                                                    << std::abs(elbow_pos_cost_before_optim - elbow_pos_cost_after_optim) << ")" 
                                                    << " (bound: " << elbow_pos_cost_bound << ")" << std::endl;
      }
      else
      {
        std::cout << "Coefficient: K_ELBOW_POS = " << K_ELBOW_POS << std::endl;
        std::cout << "Cost: elbow_pos_cost = " << elbow_pos_cost_after_optim << " (bound: " << elbow_pos_cost_bound << ")" << std::endl;
      }

    

      std::chrono::steady_clock::time_point t1_outer_loop = std::chrono::steady_clock::now();
      std::chrono::duration<double> t_spent_outer_loop_cur = std::chrono::duration_cast<std::chrono::duration<double>>(t1_outer_loop - t0_outer_loop);
      t_spent_outer_loop += t_spent_outer_loop_cur.count();

      std::cout << "> Time Usage:" << std::endl;
      std::cout << "Time used for current outer loop is: " << t_spent_outer_loop_cur.count() << " s." << std::endl;

                
    }while( (id_k_wrist_pos <= (K_WRIST_POS_set.size() - 1) && wrist_pos_cost_after_optim > wrist_pos_cost_bound) ||
            (id_k_wrist_ori <= (K_WRIST_ORI_set.size() - 1) && wrist_ori_cost_after_optim > wrist_ori_cost_bound) ||
            (id_k_elbow_pos <= (K_ELBOW_POS_set.size() - 1) && elbow_pos_cost_after_optim > elbow_pos_cost_bound) ); // Wrist Pos Loop

    std::cout << ">>>> End of Outer loop" << std::endl << std::endl;


    // Display the best result, and assign to q vertces
    std::cout << ">>>> Best Tracking Result: " << std::endl;
    std::cout << "best_dist(to the bounds) = " << best_dist << std::endl;
    std::cout << "best_elbow_pos_cost = " << best_elbow_pos_cost << std::endl;
    std::cout << "best_wrist_pos_cost = " << best_wrist_pos_cost << std::endl;
    std::cout << "best_wrist_ori_cost = " << best_wrist_ori_cost << std::endl;
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setEstimate(best_q[m]);
    }

    // time usage statistics
    std::cout << ">>>> Time Usage Statistics: " << std::endl;
    std::cout << "Inner loop runs " << count_inner_loop 
              << " times, and spent " << t_spent_outer_loop 
              << " s, with average time " << t_spent_outer_loop / count_inner_loop << " s." << std::endl;   
    std::cout << "Outer loop runs " << count_outer_loop 
              << " times, and spent " << t_spent_outer_loop 
              << " s, with average time " << t_spent_outer_loop / count_outer_loop << " s." << std::endl; 
                         
    // final weights, useless in that we perform multiple runs with different combinations of coefficients and store the best results 
    std::cout << "Final weights: K_COL = " << K_COL 
              << ", K_SMOOTHNESS = " << K_SMOOTHNESS 
              // << ", K_POS_LIMIT = " << K_POS_LIMIT 
              << ", K_WRIST_POS = " << K_WRIST_POS
              << ", K_ELBOW_POS = " << K_ELBOW_POS
              << ", with col_cost = " << col_cost_after_optim
              << ", smoothness_cost = " << smoothness_cost_after_optim
              // << ", pos_limit_cost = " << pos_limit_cost_after_optim 
              << ", wrist_pos_cost = " << wrist_pos_cost_after_optim 
              << ", wrist_ori_cost = " << wrist_ori_cost_after_optim 
              << ", elbow_cost = " << elbow_pos_cost_after_optim << std::endl << std::endl;

    // reset
    std::cout << "Re-activate DMP vertex." << std::endl;
    dmp_vertex_tmp->setFixed(false);


    // store actually executed trajectories
    // Perform FK on pre-iteration results (output functions are alread implemented in TrackingConstraint)
    std::vector<std::vector<double>> l_wrist_pos_traj, r_wrist_pos_traj, l_elbow_pos_traj, r_elbow_pos_traj;
    l_wrist_pos_traj = tracking_edge->return_wrist_pos_traj(true);
    r_wrist_pos_traj = tracking_edge->return_wrist_pos_traj(false);
    l_elbow_pos_traj = tracking_edge->return_elbow_pos_traj(true);
    r_elbow_pos_traj = tracking_edge->return_elbow_pos_traj(false);
    std::cout << ">>>> Storing current executed Cartesian trajectories to h5 file" << std::endl;
    write_h5_2d_helper(out_file_name, in_group_name, "actual_l_wrist_pos_traj_"+std::to_string(n), l_wrist_pos_traj);
    write_h5_2d_helper(out_file_name, in_group_name, "actual_r_wrist_pos_traj_"+std::to_string(n), r_wrist_pos_traj);
    write_h5_2d_helper(out_file_name, in_group_name, "actual_l_elbow_pos_traj_"+std::to_string(n), l_elbow_pos_traj);
    write_h5_2d_helper(out_file_name, in_group_name, "actual_r_elbow_pos_traj_"+std::to_string(n), r_elbow_pos_traj);

    // store statistics of the best result
    write_h5_1_helper(out_file_name, in_group_name, "best_dist_"+std::to_string(n), best_dist);
    write_h5_1_helper(out_file_name, in_group_name, "best_col_cost_"+std::to_string(n), best_col_cost);
    // write_h5_1_helper(out_file_name, in_group_name, "best_pos_limit_cost_"+std::to_string(n), best_pos_limit_cost);
    write_h5_1_helper(out_file_name, in_group_name, "best_smoothness_cost_"+std::to_string(n), best_smoothness_cost);
    write_h5_1_helper(out_file_name, in_group_name, "best_wrist_pos_cost_"+std::to_string(n), best_wrist_pos_cost);
    write_h5_1_helper(out_file_name, in_group_name, "best_wrist_ori_cost_"+std::to_string(n), best_wrist_ori_cost);
    write_h5_1_helper(out_file_name, in_group_name, "best_elbow_pos_cost_"+std::to_string(n), best_elbow_pos_cost);


    // Check stopping criteria (q and DMP!!!)
    if (col_cost_after_optim <= col_cost_bound && 
        // pos_limit_cost_after_optim <= pos_limit_bound && 
        smoothness_cost_after_optim <= smoothness_bound)
    {
      if (n!=0) // not for the first round
      {
        // check dmp
        double tmp_dmp_orien_cost = dmp_edge->output_cost(dmp_edge->ORIEN_FLAG);
        double tmp_dmp_scale_cost = dmp_edge->output_cost(dmp_edge->SCALE_FLAG);
        double tmp_dmp_rel_change_cost = dmp_edge->output_cost(dmp_edge->REL_CHANGE_FLAG);

        if (tmp_dmp_orien_cost <= dmp_orien_cost_bound &&
            tmp_dmp_scale_cost <= dmp_scale_cost_bound &&
            tmp_dmp_rel_change_cost <= dmp_rel_change_cost_bound)
          break;
      }
    }


    // 2 - Manually move DMPs starts and goals; 
    /*
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", Manually set initial guess for rw DMP: "<< std::endl;
    // get current estimates on relative DMPs
    DMPStartsGoalsVertex *dmp_vertex_tmp_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0)); // the last vertex connected
    Matrix<double, DMPPOINTS_DOF, 1> xx = dmp_vertex_tmp_tmp->estimate();
    MatrixXd lrw_new_goal(3, 1); lrw_new_goal = xx.block(0, 0, 3, 1);
    MatrixXd lrw_new_start(3, 1); lrw_new_start = xx.block(3, 0, 3, 1);
    MatrixXd lew_new_goal(3, 1); lew_new_goal = xx.block(6, 0, 3, 1);
    MatrixXd lew_new_start(3, 1); lew_new_start = xx.block(9, 0, 3, 1);
    MatrixXd rew_new_goal(3, 1); rew_new_goal = xx.block(12, 0, 3, 1);
    MatrixXd rew_new_start(3, 1); rew_new_start = xx.block(15, 0, 3, 1);
    MatrixXd rw_new_goal(3, 1); rw_new_goal = xx.block(18, 0, 3, 1);
    MatrixXd rw_new_start(3, 1); rw_new_start = xx.block(21, 0, 3, 1);
    // current starts and goals for wrists and elbows
    MatrixXd lw_new_goal(3, 1); lw_new_goal = rw_new_goal + lrw_new_goal;
    MatrixXd lw_new_start(3, 1); lw_new_start = rw_new_start + lrw_new_start;
    MatrixXd re_new_goal(3, 1); re_new_goal = rw_new_goal + rew_new_goal;
    MatrixXd re_new_start(3, 1); re_new_start = rw_new_start + rew_new_start;
    MatrixXd le_new_goal(3, 1); le_new_goal = lw_new_goal + lew_new_goal;
    MatrixXd le_new_start(3, 1); le_new_start = lw_new_start + lew_new_start;
    
    // set new goals and starts
    MatrixXd wrist_pos_offsets(3, NUM_DATAPOINTS), elbow_pos_offsets(3, NUM_DATAPOINTS);    
    // - right wrist
    wrist_pos_offsets = tracking_edge->return_r_wrist_pos_offsets(); 
    Vector3d wrist_pos_offset = wrist_pos_offsets.rowwise().mean();        
    rw_new_goal = rw_new_goal + wrist_pos_offset;
    rw_new_start = rw_new_start + wrist_pos_offset; 
    // - right elbow
    elbow_pos_offsets = tracking_edge->return_r_elbow_pos_offsets(); 
    Vector3d elbow_pos_offset = elbow_pos_offsets.rowwise().mean();    
    re_new_goal = re_new_goal + elbow_pos_offset;
    re_new_start = re_new_start + elbow_pos_offset;
     // - left wrist
    wrist_pos_offsets = tracking_edge->return_l_wrist_pos_offsets(); 
    wrist_pos_offset = wrist_pos_offsets.rowwise().mean();        
    lw_new_goal = lw_new_goal + wrist_pos_offset;
    lw_new_start = lw_new_start + wrist_pos_offset; 
    // - left elbow
    elbow_pos_offsets = tracking_edge->return_l_elbow_pos_offsets(); 
    elbow_pos_offset = elbow_pos_offsets.rowwise().mean();    
    le_new_goal = le_new_goal + elbow_pos_offset;
    le_new_start = le_new_start + elbow_pos_offset;

    // re-calculate DMP starts and goals
    lrw_new_goal = lw_new_goal - rw_new_goal;
    lrw_new_start = lw_new_start - rw_new_start;
    lew_new_goal = le_new_goal - lw_new_goal;
    lew_new_start = le_new_start - lw_new_start;    
    rew_new_goal = re_new_goal - rw_new_goal;
    rew_new_start = re_new_start - rw_new_start;

    // assign initial guess (change only the starts and goals of right wrist traj ?)
    xx.block(0, 0, 3, 1) = lrw_new_goal;
    xx.block(3, 0, 3, 1) = lrw_new_start;
    xx.block(6, 0, 3, 1) = lew_new_goal;
    xx.block(9, 0, 3, 1) = lew_new_start;
    xx.block(12, 0, 3, 1) = rew_new_goal;
    xx.block(15, 0, 3, 1) = rew_new_start;
    xx.block(18, 0, 3, 1) = rw_new_goal;
    xx.block(21, 0, 3, 1) = rw_new_start;
    dmp_vertex_tmp_tmp->setEstimate(xx);

    // store the manually moved DMP starts and goals
    // initialization
    std::vector<std::vector<double> > dmp_starts_goals_moved_vec_vec;
    std::vector<double> dmp_starts_goals_moved_vec(DMPPOINTS_DOF);
    // convert
    for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
        dmp_starts_goals_moved_vec[d] = xx[d];
    dmp_starts_goals_moved_vec_vec.push_back(dmp_starts_goals_moved_vec);
    // store and display results
    std::cout << "dmp_results size is: " << dmp_starts_goals_moved_vec_vec.size() << " x " << dmp_starts_goals_moved_vec_vec[0].size() << std::endl;
    std::cout << "dmp_results = " << xx.transpose() << std::endl;
    bool result_moved = write_h5(out_file_name, in_group_name, "dmp_starts_goals_moved_"+std::to_string(n),
                                 dmp_starts_goals_moved_vec_vec.size(), dmp_starts_goals_moved_vec_vec[0].size(), 
                                 dmp_starts_goals_moved_vec_vec);
    std::cout << "moved dmp starts and goals stored " << (result_moved ? "successfully" : "unsuccessfully") << "!" << std::endl;

    */


    // 3 - optimize DMP for a number of iterations
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", DMP stage: "<< std::endl;
    K_WRIST_ORI = 0.1;  // actually useless since orientation data are specified independent of DMP !!! (the jacobian test checks it out!!)
    K_WRIST_POS = 0.1; 
    K_ELBOW_POS = 0.1; 
    K_FINGER = 0.1;  // actually useless since finger data are specified independent of DMP !!!
    K_DMPSTARTSGOALS = 0.1;//0.5;//1.0;//2.0;
    K_DMPSCALEMARGIN = 0.1; //0.5;//1.0;//2.0;
    K_DMPRELCHANGE = 0.1; //2.0;//1.0; // relax a little to allow small variance
    std::cout << "Fix q vertices." << std::endl;
    // fix q vertices
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setFixed(true);
    }

    double tmp_dmp_orien_cost;
    double tmp_dmp_scale_cost;
    double tmp_dmp_rel_change_cost;

    do
    {
    // iterate to optimize DMP
    std::cout << "Optimizing DMP..." << std::endl;
    unsigned int dmp_iter = optimizer.optimize(dmp_per_iterations);
    // Save cost history
    std::cout << ">>>> Costs <<<<" << std::endl;
    // 1)
    std::cout << "Recording tracking cost and DMP costs..." << std::endl;
    // 3)  
    std::vector<double> l_wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->LEFT_FLAG);
    std::vector<double> r_wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->RIGHT_FLAG);
    std::vector<double> l_elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->LEFT_FLAG);
    std::vector<double> r_elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->RIGHT_FLAG);
    std::vector<double> l_finger_cost = tracking_edge->return_finger_cost_history(tracking_edge->LEFT_FLAG);      
    std::vector<double> r_finger_cost = tracking_edge->return_finger_cost_history(tracking_edge->RIGHT_FLAG);      
    std::vector<double> wrist_pos_cost = tracking_edge->return_wrist_pos_cost_history(tracking_edge->BOTH_FLAG);
    std::vector<double> wrist_ori_cost = tracking_edge->return_wrist_ori_cost_history(tracking_edge->BOTH_FLAG);
    std::vector<double> elbow_pos_cost = tracking_edge->return_elbow_pos_cost_history(tracking_edge->BOTH_FLAG);
    std::vector<double> finger_cost = tracking_edge->return_finger_cost_history(tracking_edge->BOTH_FLAG);      
    wrist_pos_cost_history.push_back(wrist_pos_cost);
    l_wrist_pos_cost_history.push_back(l_wrist_pos_cost);
    r_wrist_pos_cost_history.push_back(r_wrist_pos_cost);
    wrist_ori_cost_history.push_back(wrist_ori_cost);
    elbow_pos_cost_history.push_back(elbow_pos_cost);
    l_elbow_pos_cost_history.push_back(l_elbow_pos_cost);
    r_elbow_pos_cost_history.push_back(r_elbow_pos_cost);
    finger_cost_history.push_back(finger_cost); // store
    l_finger_cost_history.push_back(l_finger_cost); // store
    r_finger_cost_history.push_back(r_finger_cost); // store
    // display for debug:
    print_debug_output("wrist_pos_cost", wrist_pos_cost);
    print_debug_output("l_wrist_pos_cost", l_wrist_pos_cost);
    print_debug_output("r_wrist_pos_cost", r_wrist_pos_cost);
    print_debug_output("wrist_ori_cost", wrist_ori_cost);
    print_debug_output("elbow_pos_cost", elbow_pos_cost);
    print_debug_output("l_elbow_pos_cost", l_elbow_pos_cost);
    print_debug_output("r_elbow_pos_cost", r_elbow_pos_cost);
    print_debug_output("finger_cost", finger_cost);
    // 5)
    std::vector<double> dmp_orien_cost;
    dmp_orien_cost.push_back(dmp_edge->output_cost(dmp_edge->ORIEN_FLAG));
    dmp_orien_cost_history.push_back(dmp_orien_cost);
    std::vector<double> dmp_scale_cost;
    dmp_scale_cost.push_back(dmp_edge->output_cost(dmp_edge->SCALE_FLAG));
    dmp_scale_cost_history.push_back(dmp_scale_cost);
    std::vector<double> dmp_rel_change_cost;
    dmp_rel_change_cost.push_back(dmp_edge->output_cost(dmp_edge->REL_CHANGE_FLAG));
    dmp_rel_change_cost_history.push_back(dmp_rel_change_cost);
    // display for debug:
    print_debug_output("dmp_orien_cost", dmp_orien_cost);
    print_debug_output("dmp_scale_cost", dmp_scale_cost);
    print_debug_output("dmp_rel_change_cost", dmp_rel_change_cost);


    // Save Jacobians of constraints w.r.t DMP starts and goals
    std::cout << ">>>> Jacobians <<<<" << std::endl;
    std::cout << "Recording DMP jacobians.." << std::endl;
    MatrixXd jacobians(1, DMPPOINTS_DOF);    
    // 1)
    std::vector<double> jacobian_vec(DMPPOINTS_DOF); // from MatrixXd to std::vector<std::vector<double>>
    // 2)
    jacobians = tracking_edge->output_dmp_jacobian();
    std::cout << "debug: track_jacobian = ";
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
      std::cout << jacobians(0, j) << " ";
    }
    std::cout << std::endl;
    track_jacobian_history.push_back(jacobian_vec);
    // 3)
    jacobians = dmp_edge->output_jacobian(dmp_edge->ORIEN_FLAG);
    std::cout << "debug: orien_jacobian = ";    
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
      std::cout << jacobians(0, j) << " ";
    }
    std::cout << std::endl;
    orien_jacobian_history.push_back(jacobian_vec);    
    // 4)
    jacobians = dmp_edge->output_jacobian(dmp_edge->SCALE_FLAG);
    std::cout << "debug: scale_jacobian = ";    
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
      std::cout << jacobians(0, j) << " ";      
    }
    std::cout << std::endl;
    scale_jacobian_history.push_back(jacobian_vec);    
    // 5)
    jacobians = dmp_edge->output_jacobian(dmp_edge->REL_CHANGE_FLAG);
    std::cout << "debug: rel_change_jacobian = ";    
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
    {
      jacobian_vec[j] = jacobians(0, j);
      std::cout << jacobians(0, j) << " ";      
    }
    std::cout << std::endl;
    rel_change_jacobian_history.push_back(jacobian_vec);    


    // output jacobians for wrist_pos, wrist_ori and elbow_pos costs for DMP vertex
    Matrix<double, DMPPOINTS_DOF, 1> wrist_pos_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> wrist_ori_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> elbow_pos_jacobian_for_dmp;
    Matrix<double, DMPPOINTS_DOF, 1> finger_pos_jacobian_for_dmp;
    wrist_pos_jacobian_for_dmp = tracking_edge->wrist_pos_jacobian_for_dmp;
    wrist_ori_jacobian_for_dmp = tracking_edge->wrist_ori_jacobian_for_dmp;
    elbow_pos_jacobian_for_dmp = tracking_edge->elbow_pos_jacobian_for_dmp;    
    finger_pos_jacobian_for_dmp = tracking_edge->finger_pos_jacobian_for_dmp;    
    std::cout << "debug: wrist_pos_jacobian_for_dmp = " << wrist_pos_jacobian_for_dmp.transpose() << std::endl;
    std::cout << "debug: wrist_ori_jacobian_for_dmp = " << wrist_ori_jacobian_for_dmp.transpose() << std::endl;
    std::cout << "debug: elbow_pos_jacobian_for_dmp = " << elbow_pos_jacobian_for_dmp.transpose() << std::endl;
    std::cout << "debug: finger_pos_jacobian_for_dmp = " << finger_pos_jacobian_for_dmp.transpose() << std::endl;


    // store dmp updates
    std::cout << "Recording DMP updates.." << std::endl;
    std::vector<double> dmp_update_vec(DMPPOINTS_DOF);
    // DMPStartsGoalsVertex* dmp_vertex_tmp = dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0));    
    Matrix<double, DMPPOINTS_DOF, 1> last_update = dmp_vertex_tmp->last_update;
    for (unsigned int j = 0; j < DMPPOINTS_DOF; j++)
      dmp_update_vec[j] = last_update(j, 0);
    dmp_update_history.push_back(dmp_update_vec);


    // check dmp constratins
    tmp_dmp_orien_cost = dmp_edge->output_cost(dmp_edge->ORIEN_FLAG);
    tmp_dmp_scale_cost = dmp_edge->output_cost(dmp_edge->SCALE_FLAG);
    tmp_dmp_rel_change_cost = dmp_edge->output_cost(dmp_edge->REL_CHANGE_FLAG);

    if (tmp_dmp_orien_cost > dmp_orien_cost_bound && K_DMPSTARTSGOALS <= K_DMPSTARTSGOALS_MAX)
    {
      K_DMPSTARTSGOALS = K_DMPSTARTSGOALS * scale;
    }

    if (tmp_dmp_scale_cost > dmp_scale_cost_bound && K_DMPSCALEMARGIN <= K_DMPSCALEMARGIN_MAX)
    {
      K_DMPSCALEMARGIN = K_DMPSCALEMARGIN * scale;
    }

    if (tmp_dmp_rel_change_cost > dmp_rel_change_cost_bound && K_DMPRELCHANGE <= K_DMPRELCHANGE_MAX)
    {
      K_DMPRELCHANGE = K_DMPRELCHANGE * scale;
    }

    }while( (K_DMPSTARTSGOALS <= K_DMPSTARTSGOALS_MAX && tmp_dmp_orien_cost > dmp_orien_cost_bound) || 
            (K_DMPSCALEMARGIN <= K_DMPSCALEMARGIN_MAX && tmp_dmp_scale_cost > dmp_scale_cost_bound) || 
            (K_DMPRELCHANGE <= K_DMPRELCHANGE_MAX && tmp_dmp_rel_change_cost > dmp_rel_change_cost_bound) );


    // store the optimized DMP starts and goals
    // initialization
    Matrix<double, DMPPOINTS_DOF, 1> dmp_starts_goals_optimed;
    std::vector<std::vector<double> > dmp_starts_goals_optimed_vec_vec;
    std::vector<double> dmp_starts_goals_optimed_vec(DMPPOINTS_DOF);
    dmp_starts_goals_optimed = dmp_vertex_tmp->estimate();
    // convert
    for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
        dmp_starts_goals_optimed_vec[d] = dmp_starts_goals_optimed[d];
    dmp_starts_goals_optimed_vec_vec.push_back(dmp_starts_goals_optimed_vec);
    // store and display results
    std::cout << "dmp_results size is: " << dmp_starts_goals_optimed_vec_vec.size() << " x " << dmp_starts_goals_optimed_vec_vec[0].size() << std::endl;
    std::cout << "dmp_results = " << dmp_starts_goals_optimed.transpose() << std::endl;
    write_h5_2d_helper(out_file_name, in_group_name, "dmp_starts_goals_optimed_"+std::to_string(n), dmp_starts_goals_optimed_vec_vec);


    // Reset q vertices
    std::cout << "Re-activate q vertices." << std::endl;
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setFixed(false);
    }

    // record for ease
    max_round = n;

    // Terminate the process if conditions met
    std::cout << ">>>> Check terminate conditions: " << std::endl;
    // collision 
    double cur_col_cost_check = 0.0;
    std::cout << "Current collision costs = ";
    for (unsigned int t = 0; t < collision_edges.size(); t++)
    {
      double c = collision_edges[t]->return_col_cost();
      cur_col_cost_check += c; // check
      std::cout << c << " "; // display
    }
    // for (unsigned int t = 0; t < unary_edges.size(); t++)
    // {
    //   double c = unary_edges[t]->return_col_cost();
    //   cur_col_cost_check += c; // check
    //   std::cout << c << " "; // display
    // }
    std::cout << "\nTotal col_cost = " << cur_col_cost_check << std::endl;
    // pos_limit
    // std::cout << "Current pos_limit_cost = ";
    // double cur_pos_limit_cost_check = 0.0;
    // for (unsigned int t = 0; t < unary_edges.size(); t++)
    // {
    //   double p = unary_edges[t]->return_pos_limit_cost();
    //   cur_pos_limit_cost_check += p; // check
    //   std::cout << p << " "; // display      
    // }
    // std::cout << "\nTotal pos_limit_cost = " << cur_pos_limit_cost_check << std::endl;    
    // smoothness
    std::cout << "Current smoothness_cost = ";
    double cur_smoothness_cost_check = 0.0;
    for (unsigned int t = 0; t < smoothness_edges.size(); t++)
    {
      double s = smoothness_edges[t]->return_smoothness_cost();
      cur_smoothness_cost_check += s; // check
      std::cout << s << " "; // display
    }
    std::cout << "\nTotal smoothness_cost = " << cur_smoothness_cost_check << std::endl;

    // if(cur_col_cost_check < col_cost_bound && cur_pos_limit_cost_check < pos_limit_bound && cur_smoothness_cost_check < smoothness_bound) 
    if(cur_col_cost_check < col_cost_bound && cur_smoothness_cost_check < smoothness_bound) 
    {
      std::cout << ">>>>>>>> Terminate condition met. Optimization stopped after " << n+1 << " rounds." << std::endl;
      break;
    }

  }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Total time used for optimization: " << t_spent.count() << " s" << std::endl;


  std::cout << ">>>> Optimization done." << std::endl;


  // Store number of rounds reached for ease of debug
  write_h5_1_helper(out_file_name, in_group_name, "max_round", (double)max_round);


  // Store the cost history
  std::cout << ">>>> Storing the cost history..." << std::endl;
  write_h5_2d_helper(out_file_name, in_group_name, "col_cost_history", col_cost_history);
  // write_h5_2d_helper(out_file_name, in_group_name, "pos_limit_cost_history", pos_limit_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "smoothness_cost_history", smoothness_cost_history);

  write_h5_2d_helper(out_file_name, in_group_name, "wrist_pos_cost_history", wrist_pos_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "l_wrist_pos_cost_history", l_wrist_pos_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "r_wrist_pos_cost_history", r_wrist_pos_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "wrist_ori_cost_history", wrist_ori_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "elbow_pos_cost_history", elbow_pos_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "l_elbow_pos_cost_history", l_elbow_pos_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "r_elbow_pos_cost_history", r_elbow_pos_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "finger_cost_history", finger_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "l_finger_cost_history", l_finger_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "r_finger_cost_history", r_finger_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "track_jacobian_history", track_jacobian_history);

  write_h5_2d_helper(out_file_name, in_group_name, "dmp_orien_cost_history", dmp_orien_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "dmp_scale_cost_history", dmp_scale_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "dmp_rel_change_cost_history", dmp_rel_change_cost_history);
  write_h5_2d_helper(out_file_name, in_group_name, "dmp_update_history", dmp_update_history);
  write_h5_2d_helper(out_file_name, in_group_name, "orien_jacobian_history", orien_jacobian_history);
  write_h5_2d_helper(out_file_name, in_group_name, "scale_jacobian_history", scale_jacobian_history);
  write_h5_2d_helper(out_file_name, in_group_name, "rel_change_jacobian_history", rel_change_jacobian_history);


  // Statistics:
  std::cout << ">>>> Statistics: " << std::endl;
  std::cout << "Collision checking " << count_col 
            << " times, with total time = " << total_col 
            << " s, average time = " << total_col / count_col << " s." << std::endl;
  std::cout << "Trajectory generation " << count_traj 
            << " times, with total time = " << total_traj 
            << " s, average time = " << total_traj / count_traj << " s." << std::endl;
  std::cout << "Unary constraint " << count_unary 
            << " times, with total time = " << total_unary
            << " s, average time = " << total_unary / count_unary << " s." << std::endl;
  std::cout << "Smoothness constraint " << count_smoothness
            << " times, with total time = " << total_smoothness 
            << " s, average time = " << total_smoothness / count_smoothness << " s." << std::endl;
  std::cout << "Trackig constraint " << count_tracking
            << " times, with total time = " << total_tracking 
            << " s, average time = " << total_tracking / count_tracking << " s." << std::endl;                                                


  // Store the optimization value
  Matrix<double, JOINT_DOF, 1> q_result = v_list[0]->estimate();


  // Semi-positive definite property
  std::cout << "All information matrices " << (optimizer.verifyInformationMatrices(true) ? "are" : "are not") << " Positive Semi-Definite." << std::endl;


  // Convert and store q results (which can be used later for continuing the optimization!!!)
  std::vector<std::vector<double> > q_results;
  Matrix<double, JOINT_DOF, 1> q_tmp;
  std::vector<double> q_vec(JOINT_DOF);
  for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
  {
    DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+n));
    q_tmp = vertex_tmp->estimate();
    for (unsigned int d = 0; d < JOINT_DOF; d++)
      q_vec[d] = q_tmp[d];
    q_results.push_back(q_vec);
    // display for debug
    /*std::cout << "original, q_tmp: " << q_tmp.transpose() << std::endl;
    std::cout << "pushed_back q_result: ";
    for (unsigned int d = 0; d < JOINT_DOF; d++)
      std::cout << q_results[n][d] << " ";
    std::cout << std::endl;*/
  }
  std::cout << "q_results size is: " << q_results.size() << " x " << q_results[0].size() << std::endl;
  
  bool result1 = write_h5(out_file_name, in_group_name, "arm_traj_1", NUM_DATAPOINTS, JOINT_DOF, q_results);
  std::cout << "q_results stored " << (result1 ? "successfully" : "unsuccessfully") << "!" << std::endl;


  return 0;
}



