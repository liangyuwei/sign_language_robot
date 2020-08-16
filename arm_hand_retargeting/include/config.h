#ifndef CONFIG_H_
#define CONFIG_H_

// Macros
#define JOINT_DOF 38 ///< DOF of sign language robot. Now we use YuMi dual-arm manipulators (2 x 7 DOF) and Inspire-robotics robotic hands (2 x 12 DOF), i.e. 2 x 7 + 2 x 12 = 38 DOF in total.
#define DMPPOINTS_DOF 24 ///< 4 (DMPs) x 2 (start+goal) x 3 (3d pos) = 24 variables.
#define NUM_DATAPOINTS 50 ///< Pre-defined and fixed number of path points. The optimized result can be interpolated or path parametrized.

// Global Variables
double K_COL;                   ///< Coefficient for collision cost.
double K_POS_LIMIT;             ///< Coefficient for position limit cost.
double K_WRIST_ORI;             ///< Coefficient for wrist orientaion cost.
double K_WRIST_POS;             ///< Coefficient for wrist position cost.
double K_ELBOW_POS;             ///< Coefficient for elbow position cost.
double K_FINGER;                ///< Coefficient for finger position cost.
double K_SMOOTHNESS;            ///< Coefficient for smoothness cost.

double K_DMPSTARTSGOALS;        ///< Coefficient for the cost of orientation change of the vector pointing from start to goal.
double K_DMPSCALEMARGIN;        ///< Coefficient for the cost of scale change of the vector pointing from start to goal.
double K_DMPRELCHANGE;          ///< Coefficient for the cost of relative trajectories' changes in the magnitude of vectors connecting the corresponding starts and goals.

unsigned int count_col = 0;         ///< For debug, count the number of times collision checking is performed.
unsigned int count_traj = 0;        ///< For debug, count the number of times (DMP) trajectory generator is called.
unsigned int count_unary = 0;       ///< For debug, count the number of times unary edges are called.
unsigned int count_smoothness = 0;  ///< For debug, count the number of times smoothness cost is computed.
unsigned int count_tracking = 0;    ///< For debug, count the number of times tracking edge is called.

double total_col = 0;         ///< Total time used for collision checking.
double total_traj = 0;        ///< Total time used for trajectory generation.
double total_unary = 0;       ///< Total time used for computing the cost value of unary edge.
double total_smoothness = 0;  ///< Total time used for calculating smoothness cost.
double total_tracking = 0;    ///< Total time used for computing the cost value of tracking edge.

unsigned int num_update = 0;  ///< Number of times DMP starts and goals vertex is updated.
unsigned int num_track = 0;   ///< Number of times tracking edge is called.

/// URDF file path of the sign language robot model
const static std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/yumi_description/urdf/yumi_with_hands.urdf";

/// SRDF file path of the sign language robot model
const static std::string SRDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/yumi_sign_language_robot_moveit_config/config/yumi.srdf";

// Specify link names for KDL FK solver
const std::string BASE_LINK = "world";                  ///< Base link for KDL FK solver.
const std::string LEFT_ELBOW_LINK = "yumi_link_4_l";    ///< Left elbow link for KDL FK solver.
const std::string LEFT_WRIST_LINK = "yumi_link_7_l";    ///< Left wrist link for KDL FK solver.
const std::string RIGHT_ELBOW_LINK = "yumi_link_4_r";   ///< Right elbow link for KDL FK solver.
const std::string RIGHT_WRIST_LINK = "yumi_link_7_r";   ///< Right wrist link for KDL FK solver.

/** @struct 
 *  @brief Self-defined structure. 
 * For storing tracking related information and for passing into relevant g2o edges.
 */
typedef struct {

  // Human motion data
  Vector3d l_elbow_pos_goal;    ///< Target position for left elbow
  Vector3d l_wrist_pos_goal;    ///< Target position for left wrist
  Matrix3d l_wrist_ori_goal;    ///< Target orientation for left wrist
  Vector3d r_elbow_pos_goal;    ///< Target position for right elbow
  Vector3d r_wrist_pos_goal;    ///< Target position for right wrist
  Matrix3d r_wrist_ori_goal;    ///< Target orientation for right wrist
  Matrix<double, 14, 1> l_finger_pos_goal;  ///< Target angle for left hand's fingers
  Matrix<double, 14, 1> r_finger_pos_goal;  ///< Target angle for right hand's fingers

  // Human finger bounds and robotic hand finger bounds
  Matrix<double, 12, 1> l_robot_finger_start;  ///< Joint angle range, for direct scaling and linear mapping
  Matrix<double, 12, 1> l_robot_finger_final;  ///< Joint angle range, for direct scaling and linear mapping
  Matrix<double, 12, 1> r_robot_finger_start;  ///< Joint angle range, for direct scaling and linear mapping
  Matrix<double, 12, 1> r_robot_finger_final;  ///< Joint angle range, for direct scaling and linear mapping
  Matrix<double, 14, 1> glove_start;  ///< Joint angle range for data glove(human), for direct scaling and linear mapping
  Matrix<double, 14, 1> glove_final;  ///< Joint angle range for data glove(human), for direct scaling and linear mapping

  // KDL FK related
  unsigned int l_num_wrist_seg = 0;    ///< Wrist ID on left arm kdl chain
  unsigned int l_num_elbow_seg = 0;    ///< Elbow ID on left arm kdl chain
  unsigned int r_num_wrist_seg = 0;    ///< Wrist ID on right arm kdl chain
  unsigned int r_num_elbow_seg = 0;    ///< Elbow ID on right arm kdl chain

  // Joint position limits in a whole
  Matrix<double, JOINT_DOF, 1> q_pos_lb;
  Matrix<double, JOINT_DOF, 1> q_pos_ub;

  // For storing DMP generated trajectories
  Matrix<double, 3, NUM_DATAPOINTS> DMP_rw;
  Matrix<double, 3, NUM_DATAPOINTS> DMP_lw;
  Matrix<double, 3, NUM_DATAPOINTS> DMP_re;
  Matrix<double, 3, NUM_DATAPOINTS> DMP_le; // used when optimizing q independently
  
} my_constraint_struct;



#endif