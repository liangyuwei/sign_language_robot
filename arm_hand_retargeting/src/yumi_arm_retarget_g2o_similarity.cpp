#include "yumi_arm_retarget_g2o_similarity.h"


/**
 * Read the contents from file.
 * 
 * Used for reading URDF and SRDF.
 */
std::stringstream read_file(std::string file_name)
{
  std::ifstream ifs(file_name);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss;
}


/**
 * Print debug information, input is std::vector<double>. 
 */
void print_debug_output(std::string data_name, std::vector<double> data)
{
  std::cout << "debug: " << data_name << " = ";
  for (unsigned s = 0; s < data.size(); s++)
    std::cout << data[s] << " ";
  std::cout << std::endl << std::endl;
}


/**
 * Print debug information, input is double. (overloaded)
 */
void print_debug_output(std::string data_name, double data)
{
  std::cout << "debug: " << data_name << " = " << data << std::endl << std::endl;
}


/**
 * @brief A template for setting coefficents of constraints.
 *  
 * Assign coefficients to different constaint edges via information matrices. 
 * Since the equation to solve for LS problem is J^T * J * dx = -b = - J^T*e and J is the jacobian of 
 * e, any coefficients assigned wouldn't influence the value of dx. And if we were to assign coefficients 
 * to J or e, the influence of K would be negative correlated in the former case, while in the latter case 
 * it would be wrong not to let K pass from e to J(from the source code of g2o::BaseUnaryEdge(), it can 
 * be seen that linearizeOplus() calculates jacobians by calling computeError() in the way of calculating 
 * numerical differentiation !).
 * @param[in]     edges    The edges to set coefficents (information matrix) for.
 * @param[in]     Ks       The coefficients used to construction diagonal information matrix.
 */
template<typename T> void set_edges_coefficients(std::vector<T*> &edges, VectorXd Ks)
{
  // Check the dimension
  int error_dim = edges[0]->dimension();
  if (error_dim != Ks.size())
  {
    std::cerr << "Error dimension for information matrix! The input is of size " << Ks.size()
              << ", while the requirement is " << error_dim << std::endl;
    exit(-1);
  }

  // Set information matrix
  MatrixXd coefficient_matrix = MatrixXd::Identity(error_dim, error_dim);
  for (unsigned int i = 0; i < error_dim; i++)
    coefficient_matrix(i, i) = Ks[i];

  // Iterate to set coefficients
  for (unsigned int it = 0; it < edges.size(); it++)
  {
    edges[it]->setInformation(coefficient_matrix);
  }
}


/**
 * Store the actually executed trajectories, via the use of FK provided by TrackingConstraint(). 
 * 
 * @param[in]     n_round     The flag to attach to dataset name, and indicating the results of n_round round.
 */
void store_actual_trajs(const std::vector<TrackingConstraint*> &tracking_edges, 
                        std::string file_name, 
                        std::string group_name, 
                        unsigned int n_round)
{
  // Init
  std::vector<std::vector<double>> l_wrist_pos_traj, r_wrist_pos_traj, l_elbow_pos_traj, r_elbow_pos_traj;
  
  // Iterate to get data
  for (unsigned int fk = 0; fk < tracking_edges.size(); fk++)
  {
    // l wrist pos
    Vector3d l_wrist_pos = tracking_edges[fk]->return_wrist_pos(true);
    std::vector<double> l_wrist_pos_vec = {l_wrist_pos[0], l_wrist_pos[1], l_wrist_pos[2]};
    l_wrist_pos_traj.push_back(l_wrist_pos_vec);
    // r wrist pos
    Vector3d r_wrist_pos = tracking_edges[fk]->return_wrist_pos(false);
    std::vector<double> r_wrist_pos_vec = {r_wrist_pos[0], r_wrist_pos[1], r_wrist_pos[2]};
    r_wrist_pos_traj.push_back(r_wrist_pos_vec);
    // l elbow pos
    Vector3d l_elbow_pos = tracking_edges[fk]->return_elbow_pos(true);
    std::vector<double> l_elbow_pos_vec = {l_elbow_pos[0], l_elbow_pos[1], l_elbow_pos[2]};
    l_elbow_pos_traj.push_back(l_elbow_pos_vec);
    // r elbow pos
    Vector3d r_elbow_pos = tracking_edges[fk]->return_elbow_pos(false);
    std::vector<double> r_elbow_pos_vec = {r_elbow_pos[0], r_elbow_pos[1], r_elbow_pos[2]};
    r_elbow_pos_traj.push_back(r_elbow_pos_vec);
  }

  std::cout << ">>>> Storing current executed Cartesian trajectories to h5 file" << std::endl;
  
  // write to h5 file
  write_h5(file_name, group_name, "actual_l_wrist_pos_traj_"+std::to_string(n_round), l_wrist_pos_traj);
  write_h5(file_name, group_name, "actual_r_wrist_pos_traj_"+std::to_string(n_round), r_wrist_pos_traj);
  write_h5(file_name, group_name, "actual_l_elbow_pos_traj_"+std::to_string(n_round), l_elbow_pos_traj);
  write_h5(file_name, group_name, "actual_r_elbow_pos_traj_"+std::to_string(n_round), r_elbow_pos_traj);
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
    {"test-dmp-optim",        required_argument, NULL, 't'},
    {"help",                        no_argument, NULL, 'h'},
    {0,                                       0,    0,   0}
  };
  int c;
  while(1)
  {
    int opt_index = 0;
    // Get arguments
    c = getopt_long(argc, argv, "i:g:o:ht", long_options, &opt_index);
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
        std::cout << "    -t, --test-dmp-optim, Test DMP optimization only.\n" << std::endl;
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

      case 't':
        test_dmp_optim = true;
        break;

      default:
        break;
    }

  }
  std::cout << "The input h5 file name is: " << in_file_name << std::endl;
  std::cout << "The motion name is: " << in_group_name << std::endl;
  std::cout << "The output h5 file name is: " << out_file_name << std::endl;


  // Test DMP optimization, skip the q optimization of the first round
  std::vector<Matrix<double, JOINT_DOF, 1> > q_test_dmp_optim(NUM_DATAPOINTS);
  if (test_dmp_optim)
  {
    std::vector<std::vector<double> > q_results_last = read_h5(out_file_name, in_group_name, "arm_traj_1");
    for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
    {
      Matrix<double, JOINT_DOF, 1> q_tmp;
      for (unsigned int d = 0; d < JOINT_DOF; d++)
        q_tmp[d] = q_results_last[s][d];
      q_test_dmp_optim[s] = q_tmp;
    }
  }


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

  solver->setMaxTrialsAfterFailure(8); //(5); // 10 is default

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
  // vertices
  std::vector<DualArmDualHandVertex*> v_list(NUM_DATAPOINTS);
  DMPStartsGoalsVertex* dmp_vertex;
  // edges
  std::vector<CollisionConstraint*> collision_edges(NUM_DATAPOINTS); // the first one is the first path point with itself, since we only processes the latter point in CollisionConstraint
  std::vector<SmoothnessConstraint*> smoothness_edges(NUM_DATAPOINTS-1);  
  std::vector<TrackingConstraint*> tracking_edges(NUM_DATAPOINTS);
  DMPConstraints* dmp_edge;


  // For DMP vertex
  std::cout << ">>>> Adding DMP vertex " << std::endl;
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

  // For q vertices and tracking edges
  std::cout << ">>>> Adding joint angle vertices and tracking edges " << std::endl;  
  // Set non-colliding initial state for ease of collision avoidance
  Matrix<double, JOINT_DOF, 1> q_initial = Matrix<double, JOINT_DOF, 1>::Zero();
  q_initial.block(0, 0, 14, 1) << -1.5, -1.5, 1.5, 0.0, 0.0, 0.0, 0.0, 1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0; 
  // std::cout << "debug: new initial q = " << q_initial.transpose() << std::endl;
  std::vector<Matrix<double, JOINT_DOF, 1>> q_initial_initial;
  for (unsigned int it = 0; it < NUM_DATAPOINTS; ++it)
  {
    // Set finger joint values directly using linear mapping
    // get human data
    Matrix<double, 14, 1> l_finger_pos_human = trajectory_generator_ptr->l_glove_angle_traj.block(it, 0, 1, 14).transpose() * M_PI / 180.0; // y_seq's glove data is already in radius
    Matrix<double, 14, 1> r_finger_pos_human = trajectory_generator_ptr->r_glove_angle_traj.block(it, 0, 1, 14).transpose() * M_PI / 180.0; // size is 50 x DOF
    // compute robot data through linear mapping
    Matrix<double, 12, 1> l_finger_pos_initial = TrackingConstraint::map_finger_joint_values(l_finger_pos_human, true, constraint_data);
    Matrix<double, 12, 1> r_finger_pos_initial = TrackingConstraint::map_finger_joint_values(r_finger_pos_human, false, constraint_data);
    // assign to initial data
    q_initial.block(14, 0, 12, 1) = l_finger_pos_initial;
    q_initial.block(26, 0, 12, 1) = r_finger_pos_initial;


    // Add q vertex
    v_list[it] = new DualArmDualHandVertex();
    v_list[it]->set_bounds(q_l_arm_lb, q_l_arm_ub, q_r_arm_lb, q_r_arm_ub, 
                           q_l_finger_lb, q_l_finger_ub, q_r_finger_lb, q_r_finger_ub); // set bounds for restraining updates
    v_list[it]->setEstimate(q_initial);
    v_list[it]->setId(1+it); // set a unique id
    optimizer.addVertex(v_list[it]);

    // Store the result (with finger joints already mapped)
    q_initial_initial.push_back(q_initial);
    
    // Add tracking edge
    tracking_edges[it] = new TrackingConstraint(trajectory_generator_ptr, 
                                                left_fk_solver, right_fk_solver, 
                                                dual_arm_dual_hand_collision_ptr,
                                                it+1);  // point_id starts from 1
    tracking_edges[it]->setId(it);
    tracking_edges[it]->setVertex(0, optimizer.vertex(0));  // connects DMP vertex
    tracking_edges[it]->setVertex(1, optimizer.vertex(it+1));  // connects q vertex
    tracking_edges[it]->setInformation(Eigen::Matrix<double, 20, 20>::Identity());
    
    optimizer.addEdge(tracking_edges[it]);   
  }
  

  
  std::cout << ">>>> Adding binary edges: smoothness and collision costs " << std::endl;
  unsigned int num_col_checks = 50; // 100; // when set to 100, one iteration takes about 5-6 seconds
  // collision edge for the first path point, since this constraint only processes the latter vertex;
  // set number of check to 1, so as to perform only one collision checking on x0
  collision_edges[0] = new CollisionConstraint(dual_arm_dual_hand_collision_ptr, 1);
  collision_edges[0]->setId(NUM_DATAPOINTS); // similarity and tracking edges ahead of it
  collision_edges[0]->setVertex(0, optimizer.vertex(1)); 
  collision_edges[0]->setVertex(1, optimizer.vertex(1));   
  collision_edges[0]->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
  collision_edges[0]->setInformation(Eigen::Matrix<double, 6, 6>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
  optimizer.addEdge(collision_edges[0]);
  for (unsigned int it = 0; it < NUM_DATAPOINTS - 1; ++it)
  {
    // add collision edge
    collision_edges[it+1] = new CollisionConstraint(dual_arm_dual_hand_collision_ptr, num_col_checks);
    collision_edges[it+1]->setId(it+NUM_DATAPOINTS+1); // the first one with id=2 is already included
    collision_edges[it+1]->setVertex(0, optimizer.vertex(1+it)); //(0, v_list[it]); // set the 0th vertex on the edge to point to v_list[it]
    collision_edges[it+1]->setVertex(1, optimizer.vertex(2+it)); 
    collision_edges[it+1]->setMeasurement(constraint_data); // set _measurement attribute (by deep copy), can be used to pass in user data, e.g. my_constraint_struct
    collision_edges[it+1]->setInformation(Eigen::Matrix<double, 6, 6>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
    optimizer.addEdge(collision_edges[it+1]);
    
    // Add smoothness edge
    smoothness_edges[it] = new SmoothnessConstraint();
    smoothness_edges[it]->setId(2*NUM_DATAPOINTS+it); // set a unique ID
    smoothness_edges[it]->setVertex(0, optimizer.vertex(it+1)); //v_list[it]);
    smoothness_edges[it]->setVertex(1, optimizer.vertex(it+2)); //v_list[it+1]); // binary edge, connects only 2 vertices, i.e. i=0 and i=1
    smoothness_edges[it]->setInformation(Eigen::Matrix<double, 1, 1>::Identity()); // Information type correct
    optimizer.addEdge(smoothness_edges[it]);
    
  }
  

  // DMP related constraints
  std::cout << ">>>> Adding constraints for DMP starts and goals" << std::endl;
  dmp_edge = new DMPConstraints(trajectory_generator_ptr->lrw_goal, trajectory_generator_ptr->lrw_start,
                                trajectory_generator_ptr->lew_goal, trajectory_generator_ptr->lew_start,
                                trajectory_generator_ptr->rew_goal, trajectory_generator_ptr->rew_start,
                                trajectory_generator_ptr->rw_goal, trajectory_generator_ptr->rw_start);
  dmp_edge->setId(3*NUM_DATAPOINTS-1); 
  dmp_edge->setVertex(0, optimizer.vertex(0)); // DMP vertex
  dmp_edge->setInformation(Eigen::Matrix<double, 3, 3>::Identity()); // information matrix, inverse of covariance.. importance // Information type correct
  optimizer.addEdge(dmp_edge);


  // Start optimization
  std::cout << ">>>> Start optimization:" << std::endl;
  optimizer.initializeOptimization();

  std::cout << "optimizing graph: " << optimizer.vertices().size() << " vertices, " 
                                    << optimizer.edges().size() << " edges." << std::endl;

  // Semi-positive definite property
  std::cout << "All information matrices " << (optimizer.verifyInformationMatrices(true) ? "are" : "are not") << " Positive Semi-Definite." << std::endl;

  // Initialize containers for storing cost and jacobian information
  std::vector<std::vector<double> > col_cost_history; // others
  std::vector<std::vector<double> > smoothness_cost_history;

  // during q optimization
  std::vector<std::vector<double> > wrist_pos_cost_history_q_optim; // wrist pos
  std::vector<std::vector<double> > l_wrist_pos_cost_history_q_optim;
  std::vector<std::vector<double> > r_wrist_pos_cost_history_q_optim;

  std::vector<std::vector<double> > wrist_ori_cost_history_q_optim; // wrist ori
  std::vector<std::vector<double> > l_wrist_ori_cost_history_q_optim;
  std::vector<std::vector<double> > r_wrist_ori_cost_history_q_optim;
  
  std::vector<std::vector<double> > elbow_pos_cost_history_q_optim; // elbow pos
  std::vector<std::vector<double> > l_elbow_pos_cost_history_q_optim;
  std::vector<std::vector<double> > r_elbow_pos_cost_history_q_optim;

  // during DMP optimization
  std::vector<std::vector<double> > wrist_pos_cost_history_dmp_optim; // wrist pos
  std::vector<std::vector<double> > l_wrist_pos_cost_history_dmp_optim;
  std::vector<std::vector<double> > r_wrist_pos_cost_history_dmp_optim;

  std::vector<std::vector<double> > wrist_ori_cost_history_dmp_optim; // wrist ori
  std::vector<std::vector<double> > l_wrist_ori_cost_history_dmp_optim;
  std::vector<std::vector<double> > r_wrist_ori_cost_history_dmp_optim;
  
  std::vector<std::vector<double> > elbow_pos_cost_history_dmp_optim; // elbow pos
  std::vector<std::vector<double> > l_elbow_pos_cost_history_dmp_optim;
  std::vector<std::vector<double> > r_elbow_pos_cost_history_dmp_optim;
  
  // others
  std::vector<std::vector<double> > finger_cost_history;  // finger
  std::vector<std::vector<double> > l_finger_cost_history;
  std::vector<std::vector<double> > r_finger_cost_history;

  
  std::vector<std::vector<double> > dmp_orien_cost_history; // dmp
  std::vector<std::vector<double> > dmp_scale_cost_history;
  std::vector<std::vector<double> > dmp_rel_change_cost_history;

  std::vector<std::vector<double> > orien_jacobian_history; // for DMP orientation cost (vectors pointing from starts to goals)
  std::vector<std::vector<double> > scale_jacobian_history; // for DMP scale cost (scale margin of vectors)
  std::vector<std::vector<double> > rel_change_jacobian_history;

  std::vector<std::vector<double> > dmp_update_history;


  // Start optimization and store cost history
  std::cout << ">>>> Start optimization of the whole graph" << std::endl;
 
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  
  unsigned int num_rounds = 5; //3; // 20; //3; //1; //1;//20;//1;//10;//20;//200;
  unsigned int dmp_per_iterations = 20; //10;//5;//10;
  unsigned int q_trk_per_iterations = 20; //20;//50;//20;//10;//20;//50;//300;//10;//20; //50;
  unsigned int max_round; // record for ease


  // Maximum of coefficients for DMP related constraints
  double K_DMPSTARTSGOALS_MAX = 10.0; 
  double K_DMPSCALEMARGIN_MAX = 10.0; 
  double K_DMPRELCHANGE_MAX = 10.0; 

  
  // Sets of selectable coefficients
  // 1 - K_COL
  Matrix<double, 35, 1> K_COL_set; 
  double k_col_init = 1.0;//0.1;
  for (unsigned int s = 0; s < 35; s++)
  {
    K_COL_set[s] = k_col_init;
    k_col_init = k_col_init * 1.5;
  }
  // 2 - K_SMOOTHNESS
  double k_smoothness_init = 1.0;
  Matrix<double, 10, 1> K_SMOOTHNESS_set; //K_SMOOTHNESS_set << 0.1, 1.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0; //0.1, 0.5, 1.0, 2.5, 5.0, 10.0;
  for (unsigned int s = 0; s < 10; s++)
  {
    K_SMOOTHNESS_set[s] = k_smoothness_init;
    k_smoothness_init = k_smoothness_init * 1.5;
  }
  // 3 - K_ARM_TRACK
  Matrix<double, 10, 1> K_ARM_TRACK_set; K_ARM_TRACK_set << 1.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000;//1.0, 3.0, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 25.0;
  // Display for conclusion
  std::cout << ">>>> Selectable coefficients: ";
  std::cout << "K_COL = " << K_COL_set.transpose() << std::endl;
  std::cout << "K_SMOOTHNESS = " << K_SMOOTHNESS_set.transpose() << std::endl;
  std::cout << "K_ARM_TRACK = " << K_ARM_TRACK_set.transpose() << std::endl;
  

  // ID for indexing the coefficients
  unsigned int id_k_col = 0;
  unsigned int id_k_smoothness = 0;
  unsigned int id_k_arm_track = 0;

  // Constraints bounds, used as termination condition
  double eps = 1e-6; // numeric error
  double col_cost_bound = 0.5; // to cope with possible numeric error (here we still use check_self_collision() for estimating, because it might not be easy to keep minimum distance outside safety margin... )
  double smoothness_bound = std::sqrt(std::pow(5.0*M_PI/180.0, 2) * JOINT_DOF) * (NUM_DATAPOINTS-1); //std::sqrt(std::pow(5.0*M_PI/180.0, 2) * JOINT_DOF) * (NUM_DATAPOINTS-1); // in average, 3 degree allowable difference for each joint 

  double dmp_orien_cost_bound = eps; // 0.0, on account of numeric error //0.0; // better be 0, margin is already set in it!!!
  double dmp_scale_cost_bound = eps; // 0.0, on account of numeric error 0.0; // better be 0
  double dmp_rel_change_cost_bound = 0.01 * 6; // 0.01 for each // eps; // 0.0, on account of numeric error//0.0; // better be 0

  double wrist_pos_cost_bound = std::sqrt( (std::pow(0.05, 2) * 3) ) * NUM_DATAPOINTS * 2; //std::sqrt( (std::pow(0.02, 2) * 3) ) * NUM_DATAPOINTS * 2; // 2 cm allowable error; note that for dual-arm, there are NUM_DATAPOINTS * 2 elbow position goals
  double elbow_pos_cost_bound = std::sqrt( (std::pow(0.15, 2) * 3) ) * NUM_DATAPOINTS * 2; //std::sqrt( (std::pow(0.03, 2) * 3) ) * NUM_DATAPOINTS * 2; // 3 cm allowable error;
  double wrist_ori_cost_bound = 5.0 * M_PI / 180.0 * NUM_DATAPOINTS * 2; // 5.0 degree allowable error, in radius; for dual-arm, there are NUM_DATAPOINTS * 2 wrist orientation goals!!!

  // scale for adjusting DMP related constraints' coefficients
  double scale = 1.5; // increase coefficients by 20% 

  // for storing the best tracking result
  std::vector<Eigen::Matrix<double, JOINT_DOF, 1> > best_q(NUM_DATAPOINTS);    
  unsigned int best_round = 0; // record the round index number corresponding to best_q
  double best_elbow_pos_cost = 10000;
  double best_wrist_pos_cost = 10000;
  double best_wrist_ori_cost = 10000;
  double best_col_cost = NUM_DATAPOINTS;
  // double best_pos_limit_cost = 10000;
  double best_smoothness_cost = 10000;
  double best_dist = std::max(best_wrist_pos_cost - wrist_pos_cost_bound, 0.0) / wrist_pos_cost_bound + 
                      std::max(best_wrist_ori_cost - wrist_ori_cost_bound, 0.0) / wrist_ori_cost_bound +
                      std::max(best_elbow_pos_cost - elbow_pos_cost_bound, 0.0) / elbow_pos_cost_bound; // normalize it 

  // store initial DMP starts and goals
  Matrix<double, DMPPOINTS_DOF, 1> dmp_starts_goals_initial = (dynamic_cast<DMPStartsGoalsVertex*>(optimizer.vertex(0)))->estimate();
  write_h5(out_file_name, in_group_name, "dmp_starts_goals_initial", dmp_starts_goals_initial.transpose());

  // Start optimization
  // In three steps: q optimization -> manually move DMP starts and goals according to the tracking results -> DMP starts and goals optimization
  for (unsigned int n = 0; n < num_rounds; n++)
  {
    // 1 - Optimize q vertices
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", q stage: "<< std::endl;

    // Fix DMP starts and goals
    std::cout << "Fix DMP starts and goals." << std::endl;
    dmp_vertex->setFixed(true);

    // Generate desired trajectories using DMP and pass into TrackingConstraint edge
    Matrix<double, DMPPOINTS_DOF, 1> x = dmp_vertex->estimate();
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
    for (unsigned int tr = 0; tr < NUM_DATAPOINTS; tr++)
      tracking_edges[tr]->set_user_data(constraint_data);


    // signal flag for adjusting coefficients
    double col_cost_before_optim;
    double col_cost_after_optim;
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


    // way 1 - Use TRAC-IK, solve IK for each path point, one by one  
    /*
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
                      trajectory_generator_ptr->l_wrist_quat_traj(p, 3)); // (w,x,y,z)
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
    double tmp_wrist_pos_cost = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      tmp_wrist_pos_cost += tracking_edges[s]->return_wrist_pos_cost(true, true);
    // wrist ori
    double tmp_wrist_ori_cost = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      tmp_wrist_ori_cost += tracking_edges[s]->return_wrist_ori_cost(true, true);        
    // elbow pos
    double tmp_elbow_pos_cost = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      tmp_elbow_pos_cost += tracking_edges[s]->return_elbow_pos_cost(true, true);
    // finger cost
    double tmp_finger_pos_cost = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      tmp_finger_pos_cost += tracking_edges[s]->return_finger_cost(true, true);
    // collision 
    double tmp_col_cost = 0.0;
    for (unsigned int t = 0; t < collision_edges.size(); t++)
      tmp_col_cost += collision_edges[t]->return_col_cost(); 
    // smoothness
    double tmp_smoothness_cost = 0.0;
    for (unsigned int t = 0; t < smoothness_edges.size(); t++)
      tmp_smoothness_cost += smoothness_edges[t]->return_smoothness_cost();
    // time usage
    std::chrono::steady_clock::time_point t1_wrist_trac_ik_loop = std::chrono::steady_clock::now();
    std::chrono::duration<double> t_spent_wrist_trac_ik_loop = std::chrono::duration_cast<std::chrono::duration<double>>(t1_wrist_trac_ik_loop - t0_wrist_trac_ik_loop);
    
    // report
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
    best_smoothness_cost = tmp_smoothness_cost;
    // record the q values
    for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
      best_q[s] = vertex_tmp->estimate();
    }
    */

    // way 2 - Use Nullspace control for pre-processing, solve IK for each path point, one by one  
    std::chrono::steady_clock::time_point t0_nullspace = std::chrono::steady_clock::now();
    // get the reference trajectories
    std::vector<std::vector<double>> l_hd_pos_traj, r_hd_pos_traj, l_fr_pos_traj, r_fr_pos_traj;
    std::vector<double> l_hd_pos(3), r_hd_pos(3), l_fr_pos(3), r_fr_pos(3);
    std::vector<std::vector<double>> l_hd_quat_traj, r_hd_quat_traj;
    std::vector<double> l_hd_quat(4), r_hd_quat(4); // w, x, y, z
    for (unsigned int s = 0; s < result.y_lw.cols(); s++)
    {
      // 1 - Position Part
      // l wrist
      l_hd_pos[0] = result.y_lw(0, s); l_hd_pos[1] = result.y_lw(1, s); l_hd_pos[2] = result.y_lw(2, s); 
      // l elbow
      l_fr_pos[0] = result.y_le(0, s); l_fr_pos[1] = result.y_le(1, s); l_fr_pos[2] = result.y_le(2, s); 
      // r wrist
      r_hd_pos[0] = result.y_rw(0, s); r_hd_pos[1] = result.y_rw(1, s); r_hd_pos[2] = result.y_rw(2, s); 
      // r elbow
      r_fr_pos[0] = result.y_re(0, s); r_fr_pos[1] = result.y_re(1, s); r_fr_pos[2] = result.y_re(2, s); 
      // store
      l_hd_pos_traj.push_back(l_hd_pos); l_fr_pos_traj.push_back(l_fr_pos);
      r_hd_pos_traj.push_back(r_hd_pos); r_fr_pos_traj.push_back(r_fr_pos);
      // 2 - Orientation Part
      // l hand
      l_hd_quat[0] = trajectory_generator_ptr->l_wrist_quat_traj(s, 0);
      l_hd_quat[1] = trajectory_generator_ptr->l_wrist_quat_traj(s, 1);
      l_hd_quat[2] = trajectory_generator_ptr->l_wrist_quat_traj(s, 2);
      l_hd_quat[3] = trajectory_generator_ptr->l_wrist_quat_traj(s, 3);
      // r hand
      r_hd_quat[0] = trajectory_generator_ptr->r_wrist_quat_traj(s, 0);
      r_hd_quat[1] = trajectory_generator_ptr->r_wrist_quat_traj(s, 1);
      r_hd_quat[2] = trajectory_generator_ptr->r_wrist_quat_traj(s, 2);
      r_hd_quat[3] = trajectory_generator_ptr->r_wrist_quat_traj(s, 3);
      // store
      l_hd_quat_traj.push_back(l_hd_quat);
      r_hd_quat_traj.push_back(r_hd_quat);
    }
    Matrix<double, 7, 1> ql_in = q_initial.block(0, 0, 7, 1);
    Matrix<double, 7, 1> qr_in = q_initial.block(7, 0, 7, 1);
    // initialize a nullspace control object and call nullspace control to track the reference trajectoreis
    NullSpaceControl null_space_control;
    std::vector<Matrix<double, 14, 1>> q_initial_nullspace;
    q_initial_nullspace = null_space_control.solve_joint_trajectories(l_hd_pos_traj, r_hd_pos_traj, l_hd_quat_traj, 
                                                                      r_hd_quat_traj, l_fr_pos_traj, r_fr_pos_traj,
                                                                      ql_in, qr_in);
    // assign to q vertices
    for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
    {
      DualArmDualHandVertex* q_vertex = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s));
      Matrix<double, JOINT_DOF, 1> x_whole = q_vertex->estimate();
      x_whole.block(0, 0, 14, 1) = q_initial_nullspace[s];
      q_vertex->setEstimate(x_whole);
    }
    // record statistics
    // wrist pos
    double tmp_wrist_pos_cost = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      tmp_wrist_pos_cost += tracking_edges[s]->return_wrist_pos_cost(true, true);
    // wrist ori
    double tmp_wrist_ori_cost = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      tmp_wrist_ori_cost += tracking_edges[s]->return_wrist_ori_cost(true, true);        
    // elbow pos
    double tmp_elbow_pos_cost = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      tmp_elbow_pos_cost += tracking_edges[s]->return_elbow_pos_cost(true, true);
    // finger cost
    double tmp_finger_pos_cost = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      tmp_finger_pos_cost += tracking_edges[s]->return_finger_cost(true, true);
    // collision 
    double tmp_col_cost = 0.0;
    for (unsigned int t = 0; t < collision_edges.size(); t++)
      tmp_col_cost += collision_edges[t]->return_col_cost(); 
    // smoothness
    double tmp_smoothness_cost = 0.0;
    for (unsigned int t = 0; t < smoothness_edges.size(); t++)
      tmp_smoothness_cost += smoothness_edges[t]->return_smoothness_cost();
    // time usage
    std::chrono::steady_clock::time_point t1_nullspace = std::chrono::steady_clock::now();
    std::chrono::duration<double> t_spent_nullspace = std::chrono::duration_cast<std::chrono::duration<double>>(t1_nullspace - t0_nullspace);
    // report
    std::cout << ">>>> Nullspace Control done!" << std::endl;
    std::cout << "Cost: wrist_pos_cost = " << tmp_wrist_pos_cost << std::endl;
    std::cout << "Cost: wrist_ori_cost = " << tmp_wrist_ori_cost << std::endl;
    std::cout << "Cost: elbow_pos_cost = " << tmp_elbow_pos_cost << std::endl;
    std::cout << "Cost: finger_pos_cost(irrelevant) = " << tmp_finger_pos_cost << std::endl;
    std::cout << "Cost: col_cost = " << tmp_col_cost << std::endl;
    std::cout << "Cost: smoothness_cost = " << tmp_smoothness_cost << std::endl;
    std::cout << "Total time spent: " << t_spent_nullspace.count() << " s." << std::endl;
    // store Nullspace result as the best for now (if not going to use this result, i.e. only for comparison, then no need to store as the best)
    /*
    std::cout << "Storing Nullspace control result (as the best) for later comparison..." << std::endl;
    best_dist = std::max(tmp_wrist_pos_cost - wrist_pos_cost_bound, 0.0) / wrist_pos_cost_bound +
                std::max(tmp_wrist_ori_cost - wrist_ori_cost_bound, 0.0) / wrist_ori_cost_bound +
                std::max(tmp_elbow_pos_cost - elbow_pos_cost_bound, 0.0) / elbow_pos_cost_bound; 
    best_wrist_pos_cost = tmp_wrist_pos_cost;
    best_wrist_ori_cost = tmp_wrist_ori_cost;
    best_elbow_pos_cost = tmp_elbow_pos_cost;
    best_col_cost = tmp_col_cost;
    best_smoothness_cost = tmp_smoothness_cost;
    // record the q values (including fingers)
    for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
      best_q[s] = vertex_tmp->estimate();
    }
    */


    // Solve collision 
    /*
    std::chrono::steady_clock::time_point t0_col_fix = std::chrono::steady_clock::now();  
    finger_cost_before_optim = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      finger_cost_before_optim += tracking_edges[s]->return_finger_cost(true, true);

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
    // smoothness
    smoothness_cost_after_optim = 0.0;
    for (unsigned int t = 0; t < smoothness_edges.size(); t++)
      smoothness_cost_after_optim += smoothness_edges[t]->return_smoothness_cost();
    // elbow pos
    elbow_pos_cost_after_optim = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      elbow_pos_cost_after_optim += tracking_edges[s]->return_elbow_pos_cost(true, true);
    // wrist pos 
    wrist_pos_cost_after_optim = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      wrist_pos_cost_after_optim += tracking_edges[s]->return_wrist_pos_cost(true, true);
    // wrist ori
    wrist_ori_cost_after_optim = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      wrist_ori_cost_after_optim += tracking_edges[s]->return_wrist_pos_cost(true, true); 
    // finger pos
    finger_cost_after_optim = 0.0;
    for (unsigned s = 0; s < tracking_edges.size(); s++)
      finger_cost_after_optim += tracking_edges[s]->return_finger_cost(true, true);
    // time usage
    std::chrono::steady_clock::time_point t1_col_fix = std::chrono::steady_clock::now();  
    std::chrono::duration<double> t_spent_col_fix = std::chrono::duration_cast<std::chrono::duration<double>>(t1_col_fix - t0_col_fix);

    // report results    
    std::cout << ">>>> Collision fix done!" << std::endl;
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
    best_smoothness_cost = smoothness_cost_after_optim;
    // record the q values
    for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
      best_q[s] = vertex_tmp->estimate();
      q_initial_collision_fix[s] = vertex_tmp->estimate();
    }

    // save intermediate results for debug 
    std::vector<std::vector<double> > q_collision_fix_results;
    std::vector<double> q_vec_tmp(JOINT_DOF);
    for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+n));
      Matrix<double, JOINT_DOF, 1> q_tmp = vertex_tmp->estimate();
      for (unsigned int d = 0; d < JOINT_DOF; d++)
        q_vec_tmp[d] = q_tmp[d];
      q_collision_fix_results.push_back(q_vec_tmp);
    }  
    write_h5(out_file_name, in_group_name, "arm_traj_collision_fix", q_collision_fix_results);
    */


    // Start q optimization, using different combinations of coefficients
    double t_spent_tracking_loop = 0.0;
    unsigned int count_tracking_loop = 0;
    do // Tracking loop
    {
      // Statistics
      std::chrono::steady_clock::time_point t0_start = std::chrono::steady_clock::now();
      count_tracking_loop++;

      // Report condition
      std::cout << ">>>> Tracking loop: adjust K_SMOOTHNESS." << std::endl;

      // Reset/Use initial trajector computed by collision fix / TRAC-IK / Original Initial
      if (n == 0){ // only for once
      for (unsigned int id = 0; id < NUM_DATAPOINTS; id++)
        (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+id)))->setEstimate(q_initial_initial[id]);  //(q_initial);//
        // (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+id)))->setEstimate(q_initial_trac_ik[id]); // assign to the current q vertex
        // (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+id)))->setEstimate(q_initial_collision_fix[id]); // assign to the current q vertices
      }
      else{
      for (unsigned int id = 0; id < NUM_DATAPOINTS; id++)
        (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+id)))->setEstimate(best_q[id]);
      }
      
      // Set and assign coefficients
      // K_COL = K_COL_set[(id_k_col <= K_COL_set.size()-1 ? id_k_col : K_COL_set.size()-1)];
      K_COL = 1.0;
      K_SMOOTHNESS = K_SMOOTHNESS_set[(id_k_smoothness <= K_SMOOTHNESS_set.size()-1 ? id_k_smoothness : K_SMOOTHNESS_set.size()-1)];
      // K_SMOOTHNESS = 1.0;
      // K_ARM_TRACK = K_ARM_TRACK_set[(id_k_arm_track <= K_ARM_TRACK_set.size()-1 ? id_k_arm_track : K_ARM_TRACK_set.size()-1)];
      K_ARM_TRACK = 1.0; // fix the tracking coefficients
      K_FINGER = 1.0;
      set_edges_coefficients(collision_edges, K_COL * Matrix<double, 6, 1>::Ones());
      set_edges_coefficients(smoothness_edges, K_SMOOTHNESS * Matrix<double, 1, 1>::Identity());
      Matrix<double, 20, 1> K_TRACK;
      K_TRACK.block(0, 0, 18, 1) = K_ARM_TRACK * Matrix<double, 18, 1>::Ones();
      K_TRACK.block(18, 0, 2, 1) = K_FINGER * Vector2d::Ones();
      set_edges_coefficients(tracking_edges, K_TRACK);


      // Evaluate costs before optimization
      // collision counts
      col_cost_before_optim = 0.0;
      for (unsigned int t = 0; t < collision_edges.size(); t++)
        col_cost_before_optim += collision_edges[t]->return_col_cost();
      // smoothness
      smoothness_cost_before_optim = 0.0;
      for (unsigned int t = 0; t < smoothness_edges.size(); t++)
        smoothness_cost_before_optim += smoothness_edges[t]->return_smoothness_cost();
      // elbow pos
      elbow_pos_cost_before_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        elbow_pos_cost_before_optim += tracking_edges[s]->return_elbow_pos_cost(true, true);
      // wrist pos 
      wrist_pos_cost_before_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        wrist_pos_cost_before_optim += tracking_edges[s]->return_wrist_pos_cost(true, true);
      // wrist ori
      wrist_ori_cost_before_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        wrist_ori_cost_before_optim += tracking_edges[s]->return_wrist_ori_cost(true, true);  
      // finger
      finger_cost_before_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        finger_cost_before_optim += tracking_edges[s]->return_finger_cost(true, true); 


      // Report current condition
      std::cout << ">>>> Tracking loop starts..." << std::endl;
      std::cout << ">> Before optimization <<" << std::endl;
      std::cout << "Current coefficients: K_COL = " << K_COL << ", K_SMOOTHNESS = " << K_SMOOTHNESS << std::endl;
      std::cout << "Costs before inner loop optimization: col_cost = " << col_cost_before_optim 
                << ", smoothness_cost = " << smoothness_cost_before_optim << std::endl;
      std::cout << "Costs before outer loop optimization: wrist_pos_cost = " << wrist_pos_cost_before_optim 
                << ", wrist_ori_cost = " << wrist_ori_cost_before_optim 
                << ", elbow_pos_cost = " << elbow_pos_cost_before_optim << std::endl;
      std::cout << "Costs before outer loop optimization: finger_cost = " << finger_cost_before_optim << std::endl;
      std::cout << "Current coefficient: K_ARM_TRACK = " << K_ARM_TRACK << std::endl;


      // Skip the q optimization of the first round, if attempting to debug DMP optimization
      if (test_dmp_optim && n == 0)
      {
        std::cout << "Skipping q optimization, use the result loaded from h5 file..." << std::endl;
        for (unsigned int tt = 0; tt < NUM_DATAPOINTS; tt++)
          (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+tt)))->setEstimate(q_test_dmp_optim[tt]);
      }
      else{
      // optimize for a few iterations
      std::cout << "Optimizing q..." << std::endl;
      optimizer.optimize(q_trk_per_iterations); 
      }

      // resolve collision, just in case...
      std::cout << "resolve the collision state of the optimized joint trajectories..." << std::endl;
      std::vector<Matrix<double, JOINT_DOF, 1>> tmp_q(NUM_DATAPOINTS);
      for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
      {
        DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
        tmp_q[s] = vertex_tmp->estimate();
      }        
      //
      unsigned int num_intervals = 50; //100;
      double arm_update_scale = 0.01; //0.1; 
      double hand_update_scale = 200.0; //100.0; 
      tmp_q = collision_edges[0]->resolve_path_collisions(tmp_q, num_intervals, arm_update_scale, hand_update_scale);
      //
      for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
      {
        DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
        vertex_tmp->setEstimate(tmp_q[s]);
      }        
      // store the result for display
      // save intermediate results for debug 
      std::vector<std::vector<double> > tmp_q_store_store;
      std::vector<double> tmp_q_store(JOINT_DOF);
      for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
      {
        for (unsigned int d = 0; d < JOINT_DOF; d++)
          tmp_q_store[d] = tmp_q[s][d];
        tmp_q_store_store.push_back(tmp_q_store);
      }  
      write_h5(out_file_name, in_group_name, "arm_traj_collision_fix_" + std::to_string(n), tmp_q_store_store);
      // check results
      // wrist pos
      tmp_wrist_pos_cost = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        tmp_wrist_pos_cost += tracking_edges[s]->return_wrist_pos_cost(true, true);
      // wrist ori
      tmp_wrist_ori_cost = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        tmp_wrist_ori_cost += tracking_edges[s]->return_wrist_ori_cost(true, true);        
      // elbow pos
      tmp_elbow_pos_cost = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        tmp_elbow_pos_cost += tracking_edges[s]->return_elbow_pos_cost(true, true);
      // finger cost
      tmp_finger_pos_cost = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        tmp_finger_pos_cost += tracking_edges[s]->return_finger_cost(true, true);
      // collision 
      tmp_col_cost = 0.0;
      for (unsigned int t = 0; t < collision_edges.size(); t++)
        tmp_col_cost += collision_edges[t]->return_col_cost(); 
      // smoothness
      tmp_smoothness_cost = 0.0;
      for (unsigned int t = 0; t < smoothness_edges.size(); t++)
        tmp_smoothness_cost += smoothness_edges[t]->return_smoothness_cost();
      // display
      std::cout << ">> Results after reactive collision fix <<" << std::endl;
      std::cout << "Cost: wrist_pos_cost = " << tmp_wrist_pos_cost << std::endl;
      std::cout << "Cost: wrist_ori_cost = " << tmp_wrist_ori_cost << std::endl;
      std::cout << "Cost: elbow_pos_cost = " << tmp_elbow_pos_cost << std::endl;
      std::cout << "Cost: finger_pos_cost(irrelevant) = " << tmp_finger_pos_cost << std::endl;
      std::cout << "Cost: col_cost = " << tmp_col_cost << std::endl;
      std::cout << "Cost: smoothness_cost = " << tmp_smoothness_cost << std::endl;


      // Store cost results
      std::cout << ">> Costs <<" << std::endl;
      // collision 
      std::vector<double> col_cost;
      for (unsigned int t = 0; t < collision_edges.size(); t++)
        col_cost.push_back(collision_edges[t]->return_col_cost()); // store
      col_cost_history.push_back(col_cost);
      print_debug_output("col_cost", col_cost);
      // smoothness
      std::vector<double> smoothness_cost;
      for (unsigned int t = 0; t < smoothness_edges.size(); t++)
        smoothness_cost.push_back(smoothness_edges[t]->return_smoothness_cost()); // store
      smoothness_cost_history.push_back(smoothness_cost);  
      print_debug_output("smoothness_cost", smoothness_cost);
      // tracking   
      std::vector<double> wrist_pos_cost, l_wrist_pos_cost, r_wrist_pos_cost;
      std::vector<double> wrist_ori_cost, l_wrist_ori_cost, r_wrist_ori_cost;
      std::vector<double> elbow_pos_cost, l_elbow_pos_cost, r_elbow_pos_cost;
      std::vector<double> finger_cost, l_finger_cost, r_finger_cost; 
      for (unsigned int t = 0; t < tracking_edges.size(); t++)
      {
        // wrist pos
        l_wrist_pos_cost.push_back(tracking_edges[t]->return_wrist_pos_cost(true, false));
        r_wrist_pos_cost.push_back(tracking_edges[t]->return_wrist_pos_cost(false, false));
        wrist_pos_cost.push_back(tracking_edges[t]->return_wrist_pos_cost(true, true));
        // wrist ori
        l_wrist_ori_cost.push_back(tracking_edges[t]->return_wrist_ori_cost(true, false));
        r_wrist_ori_cost.push_back(tracking_edges[t]->return_wrist_ori_cost(false, false));
        wrist_ori_cost.push_back(tracking_edges[t]->return_wrist_ori_cost(true, true));
        // elbow pos
        l_elbow_pos_cost.push_back(tracking_edges[t]->return_elbow_pos_cost(true, false));
        r_elbow_pos_cost.push_back(tracking_edges[t]->return_elbow_pos_cost(false, false));
        elbow_pos_cost.push_back(tracking_edges[t]->return_elbow_pos_cost(true, true));
        // finger 
        l_finger_cost.push_back(tracking_edges[t]->return_finger_cost(true, false));
        r_finger_cost.push_back(tracking_edges[t]->return_finger_cost(false, false));
        finger_cost.push_back(tracking_edges[t]->return_finger_cost(true, true));
      }
      wrist_pos_cost_history_q_optim.push_back(wrist_pos_cost); // wrist pos
      l_wrist_pos_cost_history_q_optim.push_back(l_wrist_pos_cost);
      r_wrist_pos_cost_history_q_optim.push_back(r_wrist_pos_cost);
      wrist_ori_cost_history_q_optim.push_back(wrist_ori_cost); // wrist ori
      l_wrist_ori_cost_history_q_optim.push_back(l_wrist_ori_cost);
      r_wrist_ori_cost_history_q_optim.push_back(r_wrist_ori_cost);
      elbow_pos_cost_history_q_optim.push_back(elbow_pos_cost); // elbow pos
      l_elbow_pos_cost_history_q_optim.push_back(l_elbow_pos_cost);
      r_elbow_pos_cost_history_q_optim.push_back(r_elbow_pos_cost);
      finger_cost_history.push_back(finger_cost); // finger 
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
      // output collision jacobians for debug
      std::cout << "Norms of col_jacobians = ";
      for (unsigned int n = 0; n < NUM_DATAPOINTS; n++)
      {
        // std::cout << unary_edges[n]->col_jacobians.norm() << " ";
        std::cout << collision_edges[n]->col_jacobians.norm() << " ";
      }
      std::cout << std::endl << std::endl;    
      // output jacobians of Smoothness edge for q vertices
      std::cout << "Norms of smoothness_q_jacobian = ";
      Matrix<double, 2, JOINT_DOF> smoothness_q_jacobian;
      for (unsigned int n = 0; n < NUM_DATAPOINTS-1; n++)
      {
        smoothness_q_jacobian = smoothness_edges[n]->output_q_jacobian();
        std::cout << smoothness_q_jacobian.norm() << " ";
      }
      std::cout << std::endl << std::endl;  

      // Evaluate costs after optimization, for loop termination checks
      // wrist pos
      wrist_pos_cost_after_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        wrist_pos_cost_after_optim += tracking_edges[s]->return_wrist_pos_cost(true, true);
      // wrist ori
      wrist_ori_cost_after_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        wrist_ori_cost_after_optim += tracking_edges[s]->return_wrist_ori_cost(true, true);
      // elbow pos
      elbow_pos_cost_after_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        elbow_pos_cost_after_optim += tracking_edges[s]->return_elbow_pos_cost(true, true);
      // finger
      finger_cost_after_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        finger_cost_after_optim += tracking_edges[s]->return_finger_cost(true, true);  
      // collision counts
      col_cost_after_optim = 0.0;
      for (unsigned int t = 0; t < collision_edges.size(); t++)
        col_cost_after_optim += collision_edges[t]->return_col_cost();
      // smoothness
      smoothness_cost_after_optim = 0.0;
      for (unsigned int t = 0; t < smoothness_edges.size(); t++)
        smoothness_cost_after_optim += smoothness_edges[t]->return_smoothness_cost();


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
      


      // debug
      std::cout << ">> After optimization <<" << std::endl;
      std::cout << "Cost: wrist_pos_cost = " << wrist_pos_cost_after_optim << " (bound: " << wrist_pos_cost_bound << ")" << std::endl;
      std::cout << "Cost: wrist_ori_cost = " << wrist_ori_cost_after_optim << " (bound: " << wrist_ori_cost_bound << ")" << std::endl;
      std::cout << "Cost: elbow_pos_cost = " << elbow_pos_cost_after_optim << " (bound: " << elbow_pos_cost_bound << ")" << std::endl;    
      std::cout << "Cost: finger_cost = " << finger_cost_after_optim << std::endl;
      std::cout << "Cost: col_cost = " << col_cost_before_optim << " ----> " << col_cost_after_optim 
                                      << "(" << (col_cost_before_optim > col_cost_after_optim ? "-" : "+")
                                      << std::abs(col_cost_before_optim - col_cost_after_optim) << ")" 
                                      << " (bound: " << col_cost_bound << ")" << std::endl;
      std::cout << "Cost: smoothness_cost = " << smoothness_cost_before_optim << " ----> " << smoothness_cost_after_optim 
                                        << "(" << (smoothness_cost_before_optim > smoothness_cost_after_optim ? "-" : "+")
                                        << std::abs(smoothness_cost_before_optim - smoothness_cost_after_optim) << ")" 
                                        << " (bound: " << smoothness_bound << ")" << std::endl;

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

      std::chrono::steady_clock::time_point t0_end = std::chrono::steady_clock::now();
      std::chrono::duration<double> t_spent_cur_loop = std::chrono::duration_cast<std::chrono::duration<double>>(t0_end - t0_start);
      t_spent_tracking_loop += t_spent_cur_loop.count();
      std::cout << ">> Time Usage <<" << std::endl;
      std::cout << "Time used for current inner loop is: " << t_spent_cur_loop.count() << " s." << std::endl;
      

      // Collision condition
      /*
      std::cout << ">> Collision condition <<" << std::endl;
      for (unsigned int cc = 0; cc < NUM_DATAPOINTS; cc++)
      {
        std::cout << ">> Path point " << (cc+1) << "/" << NUM_DATAPOINTS << " <<" << std::endl;
        collision_edges[cc]->output_distance_result();
        std::cout << "col_jacobians = " << collision_edges[cc]->col_jacobians << std::endl;
      }
      // std::cout << "last q = " << (dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(NUM_DATAPOINTS)))->estimate().transpose() << std::endl;
      */


      // Check if better than current best
      // Check if the best result from wrist pos+ori loop satisfies the bounds
      std::cout << ">>>> Evaluating feasibility of the processed paths..." << std::endl;
      if (col_cost_after_optim <= col_cost_bound)
      {
        std::cout << "Feasible paths !!!" << std::endl;
        // check if better than the history-best result*/
        double tmp_dist = std::max(wrist_pos_cost_after_optim - wrist_pos_cost_bound, 0.0) / wrist_pos_cost_bound +
                          std::max(wrist_ori_cost_after_optim - wrist_ori_cost_bound, 0.0) / wrist_ori_cost_bound +
                          std::max(elbow_pos_cost_after_optim - elbow_pos_cost_bound, 0.0) / elbow_pos_cost_bound; // normalize 
        if ( (wrist_pos_cost_after_optim <= best_wrist_pos_cost) &&//wrist_pos_cost_bound) && 
              (wrist_ori_cost_after_optim <= best_wrist_ori_cost) &&//wrist_ori_cost_bound) &&
              (elbow_pos_cost_after_optim < best_elbow_pos_cost) ) // wrist costs must be within bounds, and choose the one with the smallest elbow pos cost
        {
          std::cout << "Found a result better than the best! Recording..." << std::endl;
          // record the costs
          best_dist = tmp_dist;
          best_wrist_pos_cost = wrist_pos_cost_after_optim;
          best_wrist_ori_cost = wrist_ori_cost_after_optim;
          best_elbow_pos_cost = elbow_pos_cost_after_optim;
          best_col_cost = col_cost_after_optim;
          best_smoothness_cost = smoothness_cost_after_optim;
          // record the q values
          for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
          {
            DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+s)); // get q vertex
            best_q[s] = vertex_tmp->estimate();
          } 

          // record the current round index number, for later extraction of the current DMP starts and goals
          best_round = n;

          // record finger movements as the initial state of next round (because most of the time collision is caused by finger collision)
          for (unsigned int s = 0; s < NUM_DATAPOINTS; s++)
            q_initial_initial[s].block(14, 0, 24, 1) = best_q[s].block(14, 0, 24, 1);
        }
      }
      else
      {
        std::cout << "Non-feasible paths, continue to next round, or modify the sets of coefficients !!!" << std::endl;
      }


      // skip the first-round's q optimization
      if (test_dmp_optim && n == 0)
        break;

    }while( //(id_k_col <= (K_COL_set.size()-1) && col_cost_after_optim > col_cost_bound) || 
            (id_k_smoothness <= (K_SMOOTHNESS_set.size()-1) && smoothness_cost_after_optim > smoothness_bound) );


    // Store the current round's actually executed trajectories via FK provided by TrackingConstraint()
    // note that it should be stored before best_q is assigned...
    store_actual_trajs(tracking_edges, out_file_name, in_group_name, n);


    // Display the best result, and assign to q vertces
    std::cout << ">>>> Best Tracking Result: " << std::endl;
    std::cout << "best_dist(to the bounds) = " << best_dist << std::endl;
    std::cout << "best_wrist_pos_cost = " << best_wrist_pos_cost << std::endl;
    std::cout << "best_wrist_ori_cost = " << best_wrist_ori_cost << std::endl;
    std::cout << "best_elbow_pos_cost = " << best_elbow_pos_cost << std::endl;
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setEstimate(best_q[m]);
    }

    // time usage statistics
    std::cout << ">>>> Time Usage Statistics: " << std::endl;
    std::cout << "Tracking loop runs " << count_tracking_loop 
              << " times, and spent " << t_spent_tracking_loop 
              << " s, with average time " << t_spent_tracking_loop / count_tracking_loop << " s." << std::endl;   
                         

    // reset
    std::cout << "Re-activate DMP vertex." << std::endl;
    dmp_vertex->setFixed(false);


    // store statistics of the best result
    write_h5(out_file_name, in_group_name, "best_dist_"+std::to_string(n), best_dist);
    write_h5(out_file_name, in_group_name, "best_col_cost_"+std::to_string(n), best_col_cost);
    write_h5(out_file_name, in_group_name, "best_smoothness_cost_"+std::to_string(n), best_smoothness_cost);
    write_h5(out_file_name, in_group_name, "best_wrist_pos_cost_"+std::to_string(n), best_wrist_pos_cost);
    write_h5(out_file_name, in_group_name, "best_wrist_ori_cost_"+std::to_string(n), best_wrist_ori_cost);
    write_h5(out_file_name, in_group_name, "best_elbow_pos_cost_"+std::to_string(n), best_elbow_pos_cost);


    // Check stopping criteria (q and DMP!!!)
    std::cout << ">>>> Check terminate conditions: " << std::endl;
    if (best_col_cost <= col_cost_bound && 
        best_smoothness_cost <= smoothness_bound &&
        best_wrist_pos_cost <= wrist_pos_cost_bound &&
        best_wrist_ori_cost <= wrist_ori_cost_bound &&
        best_elbow_pos_cost <= elbow_pos_cost_bound)  // check if the best tracking result is ok...
    {
      std::cout << ">>>>>>>> Tracking terminate condition met!" << std::endl;
      if (n!=0) // not for the first round
      {
        // check dmp
        double tmp_dmp_orien_cost = dmp_edge->output_cost(dmp_edge->ORIEN_FLAG);
        double tmp_dmp_scale_cost = dmp_edge->output_cost(dmp_edge->SCALE_FLAG);
        double tmp_dmp_rel_change_cost = dmp_edge->output_cost(dmp_edge->REL_CHANGE_FLAG);

        if (tmp_dmp_orien_cost <= dmp_orien_cost_bound &&
            tmp_dmp_scale_cost <= dmp_scale_cost_bound &&
            tmp_dmp_rel_change_cost <= dmp_rel_change_cost_bound)
        {
          std::cout << ">>>>>>>> DMP terminate condition met! Optimization stopped after " << n+1 << " rounds." << std::endl;
          // break;
        }
        else
        {
          std::cout << ">> DMP terminate condition not yet met, continue..." << std::endl;
          std::cout << "dmp_orien_cost = " << tmp_dmp_orien_cost << " (bound: " << dmp_orien_cost_bound << ")" << std::endl;
          std::cout << "dmp_scale_cost = " << tmp_dmp_scale_cost << " (bound: " << dmp_scale_cost_bound << ")" << std::endl;
          std::cout << "dmp_rel_change_cost = " << tmp_dmp_rel_change_cost << " (bound: " << dmp_rel_change_cost_bound << ")" << std::endl;
        }
      }
      else
        std::cout << ">> Skip the first round of checking..." << std::endl;
    }
    else
    {
      std::cout << ">> Tracking criteria not yet met. Continue..." << std::endl;
      std::cout << "col_cost = " << col_cost_after_optim << " (bound: " << col_cost_bound << ")" << std::endl;
      std::cout << "smoothness_cost = " << smoothness_cost_after_optim << " (bound: " << smoothness_bound << ")" << std::endl;
      std::cout << "wrist_pos_cost = " << wrist_pos_cost_after_optim << " (bound: " << wrist_pos_cost_bound << ")" << std::endl;
      std::cout << "wrist_ori_cost = " << wrist_ori_cost_after_optim << " (bound: " << wrist_ori_cost_bound << ")" << std::endl;
      std::cout << "elbow_pos_cost = " << elbow_pos_cost_after_optim << " (bound: " << elbow_pos_cost_bound << ")" << std::endl;
    }


    // 2 - Manually move DMPs starts and goals; 
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", Manually set initial guess for rw DMP: "<< std::endl;
    // get current estimates on relative DMPs
    Matrix<double, DMPPOINTS_DOF, 1> xx = dmp_vertex->estimate();
    MatrixXd lrw_new_goal_move(3, 1); lrw_new_goal_move = xx.block(0, 0, 3, 1);
    MatrixXd lrw_new_start_move(3, 1); lrw_new_start_move = xx.block(3, 0, 3, 1);
    MatrixXd lew_new_goal_move(3, 1); lew_new_goal_move = xx.block(6, 0, 3, 1);
    MatrixXd lew_new_start_move(3, 1); lew_new_start_move = xx.block(9, 0, 3, 1);
    MatrixXd rew_new_goal_move(3, 1); rew_new_goal_move = xx.block(12, 0, 3, 1);
    MatrixXd rew_new_start_move(3, 1); rew_new_start_move = xx.block(15, 0, 3, 1);
    MatrixXd rw_new_goal_move(3, 1); rw_new_goal_move = xx.block(18, 0, 3, 1);
    MatrixXd rw_new_start_move(3, 1); rw_new_start_move = xx.block(21, 0, 3, 1);
    // current starts and goals for wrists and elbows
    MatrixXd lw_new_goal_move(3, 1); lw_new_goal_move = rw_new_goal_move + lrw_new_goal_move;
    MatrixXd lw_new_start_move(3, 1); lw_new_start_move = rw_new_start_move + lrw_new_start_move;
    MatrixXd re_new_goal_move(3, 1); re_new_goal_move = rw_new_goal_move + rew_new_goal_move;
    MatrixXd re_new_start_move(3, 1); re_new_start_move = rw_new_start_move + rew_new_start_move;
    MatrixXd le_new_goal_move(3, 1); le_new_goal_move = lw_new_goal_move + lew_new_goal_move;
    MatrixXd le_new_start_move(3, 1); le_new_start_move = lw_new_start_move + lew_new_start_move;
    
    // set new goals and starts
    MatrixXd wrist_pos_offsets(3, NUM_DATAPOINTS), elbow_pos_offsets(3, NUM_DATAPOINTS);    
    // - right wrist
    for (unsigned int tr = 0; tr < tracking_edges.size(); tr++)
      wrist_pos_offsets.block(0, tr, 3, 1) = tracking_edges[tr]->return_wrist_pos_offset(false);
    Vector3d wrist_pos_offset = wrist_pos_offsets.rowwise().mean();        
    rw_new_goal_move = rw_new_goal_move + wrist_pos_offset;
    rw_new_start_move = rw_new_start_move + wrist_pos_offset; 
    // - right elbow
    for (unsigned int tr = 0; tr < tracking_edges.size(); tr++)
      elbow_pos_offsets.block(0, tr, 3, 1) = tracking_edges[tr]->return_elbow_pos_offset(false);    
    Vector3d elbow_pos_offset = elbow_pos_offsets.rowwise().mean();    
    re_new_goal_move = re_new_goal_move + elbow_pos_offset;
    re_new_start_move = re_new_start_move + elbow_pos_offset;
     // - left wrist
    for (unsigned int tr = 0; tr < tracking_edges.size(); tr++)
      wrist_pos_offsets.block(0, tr, 3, 1) = tracking_edges[tr]->return_wrist_pos_offset(true);    
    wrist_pos_offset = wrist_pos_offsets.rowwise().mean();        
    lw_new_goal_move = lw_new_goal_move + wrist_pos_offset;
    lw_new_start_move = lw_new_start_move + wrist_pos_offset; 
    // - left elbow
    for (unsigned int tr = 0; tr < tracking_edges.size(); tr++)
      elbow_pos_offsets.block(0, tr, 3, 1) = tracking_edges[tr]->return_elbow_pos_offset(true);    
    elbow_pos_offset = elbow_pos_offsets.rowwise().mean();    
    le_new_goal_move = le_new_goal_move + elbow_pos_offset;
    le_new_start_move = le_new_start_move + elbow_pos_offset;

    // re-calculate DMP starts and goals
    lrw_new_goal_move = lw_new_goal_move - rw_new_goal_move;
    lrw_new_start_move = lw_new_start_move - rw_new_start_move;
    lew_new_goal_move = le_new_goal_move - lw_new_goal_move;
    lew_new_start_move = le_new_start_move - lw_new_start_move;    
    rew_new_goal_move = re_new_goal_move - rw_new_goal_move;
    rew_new_start_move = re_new_start_move - rw_new_start_move;

    // assign initial guess (change only the starts and goals of right wrist traj ?)
    xx.block(0, 0, 3, 1) = lrw_new_goal_move;
    xx.block(3, 0, 3, 1) = lrw_new_start_move;
    xx.block(6, 0, 3, 1) = lew_new_goal_move;
    xx.block(9, 0, 3, 1) = lew_new_start_move;
    xx.block(12, 0, 3, 1) = rew_new_goal_move;
    xx.block(15, 0, 3, 1) = rew_new_start_move;
    xx.block(18, 0, 3, 1) = rw_new_goal_move;
    xx.block(21, 0, 3, 1) = rw_new_start_move;
    dmp_vertex->setEstimate(xx);

    // store the manually moved DMP starts and goals
    // std::cout << ">>>> Skipping Manually moving of DMP..." << std::endl;
    write_h5(out_file_name, in_group_name, "dmp_starts_goals_moved_"+std::to_string(n), xx.transpose());


    // 3 - optimize DMP for a number of iterations
    std::cout << ">>>> Round " << (n+1) << "/" << num_rounds << ", DMP stage: "<< std::endl;
    // K_ARM_TRACK = 0.01; //0.001;  // doesn't matter since the effect of tracking constraint on DMP is ignored by zeroing out the corresponding jacobians.
    // K_FINGER = 0.001; //0.005; //0.01; //0.1;  // actually useless since finger data are specified independent of DMP !!!
    K_DMPSTARTSGOALS = 1.0; //0.1;//0.5;//1.0;//2.0;
    K_DMPSCALEMARGIN = 1.0; //0.1; //0.5;//1.0;//2.0;
    K_DMPRELCHANGE = 1.0; //0.1; //2.0;//1.0; // relax a little to allow small variance
    
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

    do  // DMP optimization
    {
      // Set coefficients (information matrix) for edges
      // dmp
      Matrix<double, 3, 3> K_DMP_CONSTRAINTS = Matrix<double, 3, 3>::Zero();
      K_DMP_CONSTRAINTS(0, 0) = K_DMPSTARTSGOALS;
      K_DMP_CONSTRAINTS(1, 1) = K_DMPSCALEMARGIN;
      K_DMP_CONSTRAINTS(2, 2) = K_DMPRELCHANGE;
      dmp_edge->setInformation(K_DMP_CONSTRAINTS);
      // others
      set_edges_coefficients(collision_edges, K_COL * Matrix<double, 6, 1>::Ones());
      set_edges_coefficients(smoothness_edges, K_SMOOTHNESS * Matrix<double, 1, 1>::Identity());
      // tracking
      Matrix<double, 20, 1> K_TRACK;
      K_TRACK.block(0, 0, 18, 1) = K_ARM_TRACK * Matrix<double, 18, 1>::Ones();
      K_TRACK.block(18, 0, 2, 1) = K_FINGER * Vector2d::Ones();
      set_edges_coefficients(tracking_edges, K_TRACK);

      // Evaluate cost before optimization,
      // elbow pos
      elbow_pos_cost_before_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        elbow_pos_cost_before_optim += tracking_edges[s]->return_elbow_pos_cost(true, true);
      // wrist pos 
      wrist_pos_cost_before_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        wrist_pos_cost_before_optim += tracking_edges[s]->return_wrist_pos_cost(true, true);
      // wrist ori
      wrist_ori_cost_before_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        wrist_ori_cost_before_optim += tracking_edges[s]->return_wrist_ori_cost(true, true);  
      // finger
      finger_cost_before_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        finger_cost_before_optim += tracking_edges[s]->return_finger_cost(true, true);  

      // debug display
      std::cout << ">> Costs before optimization <<" << std::endl;
      std::cout << "debug: wrist_pos_cost = " << wrist_pos_cost_before_optim 
                  << ", wrist_ori_cost = " << wrist_ori_cost_before_optim 
                  << ", elbow_pos_cost = " << elbow_pos_cost_before_optim << std::endl << std::endl;
      // Display DMP related constraints just for debug
      print_debug_output("dmp_orien_cost", dmp_edge->output_cost(dmp_edge->ORIEN_FLAG));
      print_debug_output("dmp_scale_cost", dmp_edge->output_cost(dmp_edge->SCALE_FLAG));
      print_debug_output("dmp_rel_change_cost", dmp_edge->output_cost(dmp_edge->REL_CHANGE_FLAG));
     

      // iterate to optimize DMP
      std::cout << "Optimizing DMP..." << std::endl;
      unsigned int dmp_iter = optimizer.optimize(dmp_per_iterations);
      // Save cost history
      std::cout << ">>>> Costs After DMP optimization <<<<" << std::endl;
      // 1)
      std::cout << "Recording tracking cost and DMP costs..." << std::endl;
      // 3)  
      std::vector<double> wrist_pos_cost, l_wrist_pos_cost, r_wrist_pos_cost;
      std::vector<double> wrist_ori_cost, l_wrist_ori_cost, r_wrist_ori_cost;
      std::vector<double> elbow_pos_cost, l_elbow_pos_cost, r_elbow_pos_cost;
      std::vector<double> finger_cost, l_finger_cost, r_finger_cost; 
      for (unsigned int t = 0; t < tracking_edges.size(); t++)
      {
        // wrist pos
        l_wrist_pos_cost.push_back(tracking_edges[t]->return_wrist_pos_cost(true, false));
        r_wrist_pos_cost.push_back(tracking_edges[t]->return_wrist_pos_cost(false, false));
        wrist_pos_cost.push_back(tracking_edges[t]->return_wrist_pos_cost(true, true));
        // wrist ori
        l_wrist_ori_cost.push_back(tracking_edges[t]->return_wrist_ori_cost(true, false));
        r_wrist_ori_cost.push_back(tracking_edges[t]->return_wrist_ori_cost(false, false));
        wrist_ori_cost.push_back(tracking_edges[t]->return_wrist_ori_cost(true, true));
        // elbow pos
        l_elbow_pos_cost.push_back(tracking_edges[t]->return_elbow_pos_cost(true, false));
        r_elbow_pos_cost.push_back(tracking_edges[t]->return_elbow_pos_cost(false, false));
        elbow_pos_cost.push_back(tracking_edges[t]->return_elbow_pos_cost(true, true));
        // finger 
        l_finger_cost.push_back(tracking_edges[t]->return_finger_cost(true, false));
        r_finger_cost.push_back(tracking_edges[t]->return_finger_cost(false, false));
        finger_cost.push_back(tracking_edges[t]->return_finger_cost(true, true));
      }
      // no need to store tracking results during DMP optimization
      wrist_pos_cost_history_dmp_optim.push_back(wrist_pos_cost); // wrist pos
      l_wrist_pos_cost_history_dmp_optim.push_back(l_wrist_pos_cost);
      r_wrist_pos_cost_history_dmp_optim.push_back(r_wrist_pos_cost);
      wrist_ori_cost_history_dmp_optim.push_back(wrist_ori_cost); // wrist ori
      l_wrist_ori_cost_history_dmp_optim.push_back(l_wrist_ori_cost);
      r_wrist_ori_cost_history_dmp_optim.push_back(r_wrist_ori_cost);
      elbow_pos_cost_history_dmp_optim.push_back(elbow_pos_cost); // elbow pos
      l_elbow_pos_cost_history_dmp_optim.push_back(l_elbow_pos_cost);
      r_elbow_pos_cost_history_dmp_optim.push_back(r_elbow_pos_cost);
      finger_cost_history.push_back(finger_cost); // finger 
      l_finger_cost_history.push_back(l_finger_cost);
      r_finger_cost_history.push_back(r_finger_cost);  
      
      // display for debug:
      print_debug_output("wrist_pos_cost", wrist_pos_cost);
      print_debug_output("l_wrist_pos_cost", l_wrist_pos_cost);
      print_debug_output("r_wrist_pos_cost", r_wrist_pos_cost);
      print_debug_output("wrist_ori_cost", wrist_ori_cost);
      print_debug_output("elbow_pos_cost", elbow_pos_cost);
      print_debug_output("l_elbow_pos_cost", l_elbow_pos_cost);
      print_debug_output("r_elbow_pos_cost", r_elbow_pos_cost);
      print_debug_output("finger_cost", finger_cost);
      // 5) store DMP related constraints
      std::vector<double> dmp_orien_cost;
      dmp_orien_cost.push_back(dmp_edge->output_cost(dmp_edge->ORIEN_FLAG));
      dmp_orien_cost_history.push_back(dmp_orien_cost);
      std::vector<double> dmp_scale_cost;
      dmp_scale_cost.push_back(dmp_edge->output_cost(dmp_edge->SCALE_FLAG));
      dmp_scale_cost_history.push_back(dmp_scale_cost);
      std::vector<double> dmp_rel_change_cost;
      dmp_rel_change_cost.push_back(dmp_edge->output_cost(dmp_edge->REL_CHANGE_FLAG));
      dmp_rel_change_cost_history.push_back(dmp_rel_change_cost);


      // Save Jacobians of constraints w.r.t DMP starts and goals
      std::cout << ">>>> Jacobians <<<<" << std::endl;
      std::cout << "Recording DMP jacobians.." << std::endl;
      MatrixXd jacobians(1, DMPPOINTS_DOF);    
      // 1)
      std::vector<double> jacobian_vec(DMPPOINTS_DOF); // from MatrixXd to std::vector<std::vector<double>>
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
      // tracking cost jacobians w.r.t DMP starts and goals
      // Matrix<double, DMPPOINTS_DOF, NUM_DATAPOINTS> tracking_jacobians_for_dmp;
      // for (unsigned int tr = 0; tr < tracking_edges.size(); tr++)
      // {
      //   MatrixXd tracking_jacobians_for_dmp_cur = tracking_edges[tr]->output_dmp_jacobian();
      //   for (unsigned int d = 0; d < DMPPOINTS_DOF; d++)
      //     tracking_jacobians_for_dmp(d, tr) = tracking_jacobians_for_dmp_cur.block(0, d, 20, 1).norm();
      // }
      // std::cout << "debug: tracking_jacobians_for_dmp = " << tracking_jacobians_for_dmp << std::endl;


      // store dmp updates
      std::cout << "Recording DMP updates.." << std::endl;
      std::vector<double> dmp_update_vec(DMPPOINTS_DOF);
      Matrix<double, DMPPOINTS_DOF, 1> last_update = dmp_vertex->last_update;
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


      // Evaluate cost after optimization,
      // elbow pos
      elbow_pos_cost_after_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        elbow_pos_cost_after_optim += tracking_edges[s]->return_elbow_pos_cost(true, true);
      // wrist pos 
      wrist_pos_cost_after_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        wrist_pos_cost_after_optim += tracking_edges[s]->return_wrist_pos_cost(true, true);
      // wrist ori
      wrist_ori_cost_after_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        wrist_ori_cost_after_optim += tracking_edges[s]->return_wrist_ori_cost(true, true);  
      // finger
      finger_cost_after_optim = 0.0;
      for (unsigned s = 0; s < tracking_edges.size(); s++)
        finger_cost_after_optim += tracking_edges[s]->return_finger_cost(true, true);  
      // debug display
      std::cout << ">> Costs after optimization <<" << std::endl;
      std::cout << "debug: wrist_pos_cost = " << wrist_pos_cost_after_optim 
                  << ", wrist_ori_cost = " << wrist_ori_cost_after_optim << ", elbow_pos_cost = " << elbow_pos_cost_after_optim << std::endl << std::endl;
      // display for debug:
      print_debug_output("dmp_orien_cost", dmp_orien_cost);
      print_debug_output("dmp_scale_cost", dmp_scale_cost);
      print_debug_output("dmp_rel_change_cost", dmp_rel_change_cost);                  

    }while( (K_DMPSTARTSGOALS <= K_DMPSTARTSGOALS_MAX && tmp_dmp_orien_cost > dmp_orien_cost_bound) || 
            (K_DMPSCALEMARGIN <= K_DMPSCALEMARGIN_MAX && tmp_dmp_scale_cost > dmp_scale_cost_bound) || 
            (K_DMPRELCHANGE <= K_DMPRELCHANGE_MAX && tmp_dmp_rel_change_cost > dmp_rel_change_cost_bound) );


    // store the optimized DMP starts and goals
    Matrix<double, DMPPOINTS_DOF, 1> dmp_starts_goals_optimed = dmp_vertex->estimate();
    write_h5(out_file_name, in_group_name, "dmp_starts_goals_optimed_"+std::to_string(n), dmp_starts_goals_optimed.transpose());


    // Reset q vertices
    std::cout << "Re-activate q vertices." << std::endl;
    for (unsigned int m = 0; m < NUM_DATAPOINTS; m++)
    {
      DualArmDualHandVertex* vertex_tmp = dynamic_cast<DualArmDualHandVertex*>(optimizer.vertex(1+m));
      vertex_tmp->setFixed(false);
    }


    // record for ease
    max_round = n;   

  }    

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> t_spent = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  std::cout << "Total time used for optimization: " << t_spent.count() << " s" << std::endl;
  std::cout << ">>>> Optimization done." << std::endl;


  // Store number of rounds reached for ease of debug
  write_h5(out_file_name, in_group_name, "max_round", (double)max_round);
  write_h5(out_file_name, in_group_name, "best_round", (double)best_round);

  // Store the cost history
  std::cout << ">>>> Storing the cost history..." << std::endl;
  write_h5(out_file_name, in_group_name, "col_cost_history", col_cost_history);
  write_h5(out_file_name, in_group_name, "smoothness_cost_history", smoothness_cost_history);

  write_h5(out_file_name, in_group_name, "wrist_pos_cost_history_q_optim", wrist_pos_cost_history_q_optim);
  write_h5(out_file_name, in_group_name, "l_wrist_pos_cost_history_q_optim", l_wrist_pos_cost_history_q_optim);
  write_h5(out_file_name, in_group_name, "r_wrist_pos_cost_history_q_optim", r_wrist_pos_cost_history_q_optim);
  write_h5(out_file_name, in_group_name, "wrist_ori_cost_history_q_optim", wrist_ori_cost_history_q_optim);
  write_h5(out_file_name, in_group_name, "elbow_pos_cost_history_q_optim", elbow_pos_cost_history_q_optim);
  write_h5(out_file_name, in_group_name, "l_elbow_pos_cost_history_q_optim", l_elbow_pos_cost_history_q_optim);
  write_h5(out_file_name, in_group_name, "r_elbow_pos_cost_history_q_optim", r_elbow_pos_cost_history_q_optim);

  write_h5(out_file_name, in_group_name, "wrist_pos_cost_history_dmp_optim", wrist_pos_cost_history_dmp_optim);
  write_h5(out_file_name, in_group_name, "l_wrist_pos_cost_history_dmp_optim", l_wrist_pos_cost_history_dmp_optim);
  write_h5(out_file_name, in_group_name, "r_wrist_pos_cost_history_dmp_optim", r_wrist_pos_cost_history_dmp_optim);
  write_h5(out_file_name, in_group_name, "wrist_ori_cost_history_dmp_optim", wrist_ori_cost_history_dmp_optim);
  write_h5(out_file_name, in_group_name, "elbow_pos_cost_history_dmp_optim", elbow_pos_cost_history_dmp_optim);
  write_h5(out_file_name, in_group_name, "l_elbow_pos_cost_history_dmp_optim", l_elbow_pos_cost_history_dmp_optim);
  write_h5(out_file_name, in_group_name, "r_elbow_pos_cost_history_dmp_optim", r_elbow_pos_cost_history_dmp_optim);

  write_h5(out_file_name, in_group_name, "finger_cost_history", finger_cost_history);
  write_h5(out_file_name, in_group_name, "l_finger_cost_history", l_finger_cost_history);
  write_h5(out_file_name, in_group_name, "r_finger_cost_history", r_finger_cost_history);

  write_h5(out_file_name, in_group_name, "dmp_orien_cost_history", dmp_orien_cost_history);
  write_h5(out_file_name, in_group_name, "dmp_scale_cost_history", dmp_scale_cost_history);
  write_h5(out_file_name, in_group_name, "dmp_rel_change_cost_history", dmp_rel_change_cost_history);
  write_h5(out_file_name, in_group_name, "dmp_update_history", dmp_update_history);
  write_h5(out_file_name, in_group_name, "orien_jacobian_history", orien_jacobian_history);
  write_h5(out_file_name, in_group_name, "scale_jacobian_history", scale_jacobian_history);
  write_h5(out_file_name, in_group_name, "rel_change_jacobian_history", rel_change_jacobian_history);


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


  // Convert and store q results 
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
  }  
  write_h5(out_file_name, in_group_name, "arm_traj_1", q_results);


  return 0;
}



