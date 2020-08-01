// For ROS
#include <ros/ros.h>
#include <ros/time.h>

// For tf broadcast
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

// Others
#include <vector>
#include <string>

// Message types
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <arm_hand_capture/GloveState.h>
#include <arm_hand_capture/DualArmDualHandStateWithImage.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// For file write and read
#include "H5Cpp.h"

// Process the terminal arguments
#include <getopt.h>


using namespace geometry_msgs;
using namespace H5;

/* Read h5 file for joint path */
std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name)
{
  // Set up file name and dataset name
  const H5std_string FILE_NAME(file_name);
  const H5std_string DATASET_NAME(dataset_name);
  const H5std_string GROUP_NAME(group_name);

  try
  {
    // Open the specified file and the specified dataset in the file.
    H5File file( FILE_NAME, H5F_ACC_RDONLY );
    //DataSet dataset = file.openDataSet(DATASET_NAME)

    // Open a group 
    Group group = file.openGroup(GROUP_NAME);
    DataSet dataset = group.openDataSet(DATASET_NAME);

    // Get the class of the datatype that is used by the dataset.
    H5T_class_t type_class = dataset.getTypeClass();

    // Get dataspace of the dataset.
    DataSpace dataspace = dataset.getSpace();

    // Get the dimension size of each dimension in the dataspace and display them.
    hsize_t dims_out[2];
    int ndims = dataspace.getSimpleExtentDims( dims_out, NULL); // though ndims is not used, this line of code assigns data to dims_out!!!
    int ROW = dims_out[0], COL = dims_out[1];

    // Read data into raw buffer(array) and convert to std::vector
    double data_array[ROW][COL];
    dataset.read(data_array, PredType::NATIVE_DOUBLE);
    std::vector<std::vector<double>> data_vector(ROW, std::vector<double>(COL));
    for (unsigned int j = 0; j < dims_out[0]; j++)
    {
      for (unsigned int i = 0; i < dims_out[1]; i++)
        data_vector[j][i] = data_array[j][i];
    }

    return data_vector;

  } 
   // catch failure caused by the H5File operations
   catch( FileIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSet operations
   catch( DataSetIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSpace operations
   catch( DataSpaceIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSpace operations
   catch( DataTypeIException error )
   {
      error.printErrorStack();
      exit(-1);
   }

}



/* Convert human wrist orientation data to quaternions */
std::vector<Eigen::Quaterniond> rotation_matrix_to_quaternion(std::vector<std::vector<double>> wrist_ori_sequence)
{
  // Prep
  std::vector<Eigen::Quaterniond> quaternion_sequence;
  Eigen::Quaterniond cur_quaternion;
  unsigned int num_datapoints = wrist_ori_sequence.size();

  // Iterate to convert
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // Get rotation matrix
    std::vector<double> wrist_ori = wrist_ori_sequence[n]; // 9-dim
    Eigen::Matrix3d wrist_rot = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(wrist_ori.data(), 3, 3);
    // Convert to quaternion
    cur_quaternion = wrist_rot;
    // Store
    quaternion_sequence.push_back(cur_quaternion);
  }

  return quaternion_sequence;

}

Pose getPose(std::vector<double> pos, std::vector<double> quat)
{
  // Pose msg type is constructed by: position and orientation

  // Copy to a new Pose object
  Pose pose;
  pose.position.x = pos[0];
  pose.position.y = pos[1];
  pose.position.z = pos[2];
  pose.orientation.x = quat[0];
  pose.orientation.y = quat[1];
  pose.orientation.z = quat[2];
  pose.orientation.w = quat[3]; 

  return pose;
}

arm_hand_capture::DualArmDualHandStateWithImage construct_state_msg(std::vector<double> r_up_pos, std::vector<double> r_up_quat,
                                                                    std::vector<double> r_fr_pos, std::vector<double> r_fr_quat,
                                                                    std::vector<double> r_hd_pos, std::vector<double> r_hd_quat,
                                                                    std::vector<double> l_up_pos, std::vector<double> l_up_quat,
                                                                    std::vector<double> l_fr_pos, std::vector<double> l_fr_quat,
                                                                    std::vector<double> l_hd_pos, std::vector<double> l_hd_quat)
{
  // quaternion is (x,y,z,w)

  // Common header
  std_msgs::Header header;
  header.stamp = ros::Time(); // 
  header.frame_id = "world";

  // Construct a combined result
  arm_hand_capture::DualArmDualHandStateWithImage output;
  output.right_upperarm_pose.header = header;
  output.right_upperarm_pose.pose = getPose(r_up_pos, r_up_quat);

  output.right_forearm_pose.header = header;
  output.right_forearm_pose.pose = getPose(r_fr_pos, r_fr_quat);

  output.right_hand_pose.header = header;
  output.right_hand_pose.pose = getPose(r_hd_pos, r_hd_quat);

  output.left_upperarm_pose.header = header;
  output.left_upperarm_pose.pose = getPose(l_up_pos, l_up_quat);

  output.left_forearm_pose.header = header;
  output.left_forearm_pose.pose = getPose(l_fr_pos, l_fr_quat);

  output.left_hand_pose.header = header;
  output.left_hand_pose.pose = getPose(l_hd_pos, l_hd_quat);

  // output.glove_state.header = header;

  // output.image.header = header;

  return output;

}




/* Read human demonstrated and re-adjusted movements from H5 file, and visualize in RViz. 
 * Position is visualized by using ** visualization_msgs markers **,
 * while Orientation(local frame) is visualized by ** tf broadcasting **.
 * Note that all orientation data come from human demonstrations. (for ease of display, not using robot's correpsonding up,fr,hd orientation)
 */
int main(int argc, char** argv)
{


  // Initialize a ROS node
  ros::init(argc, argv, "load_visualization_markers_node");
  ros::NodeHandle n_;
  
  // Initialize a publisher for publishing visualization markers
  ROS_INFO_STREAM("Bring up two publishers for original and adjusted movements...");  
  ros::Publisher original_pub_;
  original_pub_ = n_.advertise<arm_hand_capture::DualArmDualHandStateWithImage>("/dual_arms_dual_hands_state_with_image_original_movement", 100);
  ros::Publisher adjusted_pub_;
  adjusted_pub_ = n_.advertise<arm_hand_capture::DualArmDualHandStateWithImage>("/dual_arms_dual_hands_state_with_image_adjusted_movement", 100);
  ROS_INFO_STREAM("Ready to publish visualization markers.");


  // Settings
  std::string original_in_file_name = "test_imi_data.h5";
  std::string adjusted_in_file_name = "test_imi_data_YuMi.h5";
  std::string in_group_name = "fengren_1";

  // Process the terminal arguments
  static struct option long_options[] = 
  {
    {"original-in-file-name",        required_argument, NULL, 'o'},
    {"adjusted-in-file-name",        required_argument, NULL, 'a'},
    {"in-group-name",         required_argument, NULL, 'g'},
    {"help",                        no_argument, NULL, 'h'},
    {0,                                       0,    0,   0}
  };
  int c;
  while(1)
  {
    int opt_index = 0;
    // Get arguments
    c = getopt_long(argc, argv, "o:a:g:h", long_options, &opt_index);
    if (c == -1)
      break;

    // Process
    switch(c)
    {
      case 'h':
        std::cout << "Help: \n" << std::endl;
        std::cout << "    This program reads imitation data from h5 file and performs optimization on the joint angles. The results are stored in a h5 file at last.\n" << std::endl; 
        std::cout << "Arguments:\n" << std::endl;
        std::cout << "    -o, --original-in-file-name, specify the name of the h5 file that stores original orientation data, otherwise a default name specified inside the program will be used. Suffix is required.\n" << std::endl;
        std::cout << "    -a, --adjusted-in-file-name, specify the name of the h5 file that stores adjusted position data, otherwise a default name specified inside the program will be used. Suffix is required.\n" << std::endl;
        std::cout << "    -g, --in-group-name, specify the group name in the h5 file, which is actually the motion's name.\n" << std::endl;
        return 0;
        break;

      case 'o':
        original_in_file_name = optarg;
        break;

      case 'a':
        adjusted_in_file_name = optarg;
        break;        

      case 'g':
        in_group_name = optarg;
        break;

      default:
        break;
    }

  }
  std::cout << "The input h5 file to load orientation data is: " << original_in_file_name << std::endl;
  std::cout << "The input h5 file to load original and adjusted position data is: " << adjusted_in_file_name << std::endl;
  std::cout << "The motion name is: " << in_group_name << std::endl;


  // Load human demonstrated movements and adjusted movements (wrist, elbow, shoulder inferred from common hand-tip traj)
  // 1 - original ones
  std::vector<std::vector<double> > l_wrist_pos_human = read_h5(adjusted_in_file_name, in_group_name, "l_wrist_pos");
  std::vector<std::vector<double> > r_wrist_pos_human = read_h5(adjusted_in_file_name, in_group_name, "r_wrist_pos");
  std::vector<std::vector<double> > l_elbow_pos_human = read_h5(adjusted_in_file_name, in_group_name, "l_elbow_pos");
  std::vector<std::vector<double> > r_elbow_pos_human = read_h5(adjusted_in_file_name, in_group_name, "r_elbow_pos");
  std::vector<std::vector<double> > l_shoulder_pos_human = read_h5(adjusted_in_file_name, in_group_name, "l_shoulder_pos");
  std::vector<std::vector<double> > r_shoulder_pos_human = read_h5(adjusted_in_file_name, in_group_name, "r_shoulder_pos");
  // 2 - adjusted ones
  std::vector<std::vector<double> > l_wrist_pos_adjusted = read_h5(adjusted_in_file_name, in_group_name, "l_wrist_pos_adjusted");
  std::vector<std::vector<double> > r_wrist_pos_adjusted = read_h5(adjusted_in_file_name, in_group_name, "r_wrist_pos_adjusted");
  std::vector<std::vector<double> > l_elbow_pos_adjusted = read_h5(adjusted_in_file_name, in_group_name, "l_elbow_pos_adjusted");
  std::vector<std::vector<double> > r_elbow_pos_adjusted = read_h5(adjusted_in_file_name, in_group_name, "r_elbow_pos_adjusted");
  std::vector<std::vector<double> > l_shoulder_pos_adjusted = read_h5(adjusted_in_file_name, in_group_name, "l_shoulder_pos_adjusted");
  std::vector<std::vector<double> > r_shoulder_pos_adjusted = read_h5(adjusted_in_file_name, in_group_name, "r_shoulder_pos_adjusted");  // 3 - common orientation
  // 3 - Common, orientation data (using only human demonstration data, i.e. local frames that match motion capture markers... for convenience, so that no need to convert to robot's local frame...)
  std::vector<std::vector<double> > l_shoulder_quat = read_h5(original_in_file_name, in_group_name, "l_up_quat");
  std::vector<std::vector<double> > r_shoulder_quat = read_h5(original_in_file_name, in_group_name, "r_up_quat");
  std::vector<std::vector<double> > l_elbow_quat = read_h5(original_in_file_name, in_group_name, "l_fr_quat");
  std::vector<std::vector<double> > r_elbow_quat = read_h5(original_in_file_name, in_group_name, "r_fr_quat");  
  std::vector<std::vector<double> > l_wrist_quat = read_h5(original_in_file_name, in_group_name, "l_hd_quat");
  std::vector<std::vector<double> > r_wrist_quat = read_h5(original_in_file_name, in_group_name, "r_hd_quat"); // quaternion is (x,y,z,w)

  std::vector<std::vector<double> > timestamp = read_h5(adjusted_in_file_name, in_group_name, "time");

  unsigned int num_datapoints = l_wrist_pos_human.size();

  // convert rotation matrices to quaternions
  // std::vector<Eigen::Quaterniond> l_wrist_quat_sequence = rotation_matrix_to_quaternion(l_wrist_ori);
  // std::vector<Eigen::Quaterniond> r_wrist_quat_sequence = rotation_matrix_to_quaternion(r_wrist_ori);

  // debug:
  // std::cout << "size of time: " << timestamp.size() << " x " << timestamp[0].size() << std::endl;
  // std::cout << "size of pos: " << l_wrist_pos_human.size() << " x " << l_wrist_pos_human[0].size() << std::endl;
  // std::cout << "size of pos: " << l_wrist_pos_adjusted.size() << " x " << l_wrist_pos_adjusted[0].size() << std::endl;
  
  ros::Rate loop_rate(10); // 10 times per second
  for (unsigned int n = 0; n < num_datapoints; n++)
  {
    // Get messages
    arm_hand_capture::DualArmDualHandStateWithImage original_state_msg = construct_state_msg(r_shoulder_pos_human[n], r_shoulder_quat[n],
                                                                           r_elbow_pos_human[n], r_elbow_quat[n],
                                                                           r_wrist_pos_human[n], r_wrist_quat[n],
                                                                           l_shoulder_pos_human[n], l_shoulder_quat[n],
                                                                           l_elbow_pos_human[n], l_elbow_quat[n],
                                                                           l_wrist_pos_human[n], l_wrist_quat[n]);

    arm_hand_capture::DualArmDualHandStateWithImage adjusted_state_msg = construct_state_msg(r_shoulder_pos_adjusted[n], r_shoulder_quat[n],
                                                                           r_elbow_pos_adjusted[n], r_elbow_quat[n],
                                                                           r_wrist_pos_adjusted[n], r_wrist_quat[n],
                                                                           l_shoulder_pos_adjusted[n], l_shoulder_quat[n],
                                                                           l_elbow_pos_adjusted[n], l_elbow_quat[n],
                                                                           l_wrist_pos_adjusted[n], l_wrist_quat[n]);
    // Publish states
    original_pub_.publish(original_state_msg);
    adjusted_pub_.publish(adjusted_state_msg);

    loop_rate.sleep();

  }


  return 0;
}


