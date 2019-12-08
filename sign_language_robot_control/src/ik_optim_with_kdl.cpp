#include "ros/ros.h"
#include <iostream>

// For KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>


const std::string URDF_FILE = "/home/liangyuwei/sign_language_robot_ws/src/ur_description/urdf/ur5_robot_with_hands.urdf";

const std::string BASE_LINK = "world"; // use /world as base_link for convenience in simulation; when transfer across different robot arms, may use mid-point between shoulders as the common base(or world)
const std::string ELBOW_LINK = "left_forearm_link";
const std::string WRIST_LINK = "left_ee_link";

int main(int argc, char** argv){

  // Initialize a node
  ros::init(argc, argv, "ik_optim_with_kdl");

  
  // Build a KDL Tree from a URDF file
  KDL::Tree kdl_tree;
   if (!kdl_parser::treeFromFile(URDF_FILE, kdl_tree)){ 
      ROS_ERROR("Failed to construct kdl tree");
      return -1;
   }
  ROS_INFO("Successfully built a KDL tree from URDF file.");

  
  // Get chain
  KDL::Chain kdl_chain;
  if(!kdl_tree.getChain(BASE_LINK, WRIST_LINK, kdl_chain)){
    ROS_INFO("Failed to obtain chain from root to wrist");
    return -1;
  }
  ROS_INFO("Successfully obtained chain from root to wrist.");
  

  // Find segment number for wrist and elbow links, for calculating FK later
  unsigned int num_segments = kdl_chain.getNrOfSegments();
  unsigned int num_wrist_seg = num_segments - 1;
  unsigned int num_elbow_seg = 0;
  ROS_INFO_STREAM("There are " << num_segments << " segments in the kdl_chain");
  for (unsigned int i = 0; i < num_segments; ++i){
    //std::cout << kdl_chain.getSegment(i).getName() << std::endl;
    if (kdl_chain.getSegment(i).getName() == ELBOW_LINK){
      num_elbow_seg = i;
      ROS_INFO_STREAM("Elbow link found.");
      break;
    }
  }

  std::cout << "Elbow ID: " << num_elbow_seg << ", Wrist ID: " << num_wrist_seg << "." << std::endl;

  // Set up FK solver and compute the homogeneous representations
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  KDL::JntArray q_in(kdl_chain.getNrOfJoints()); // Input joint angles
  KDL::SetToZero(q_in);
  ROS_INFO_STREAM("Joint dimension is: " << kdl_chain.getNrOfJoints()); // 6 joints, 8 segments, checked!
  ROS_INFO_STREAM("Joint angles:");
  for (unsigned int i = 0; i < kdl_chain.getNrOfJoints(); ++i)
  {
    ROS_INFO("%f ", q_in(i));
  }
  // q_in(i) = xx, assignment
  KDL::Frame elbow_cart_out, wrist_cart_out; // Output homogeneous transformation
  int result;
  result = fk_solver.JntToCart(q_in, elbow_cart_out, num_elbow_seg+1); // notice that this is the number of segments, not the segment ID!!!
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing elbow link, something went wrong");
    return -1;
  }
  else{
    ROS_INFO_STREAM("FK solver succeeded for elbow link.");
  }
  result = fk_solver.JntToCart(q_in, wrist_cart_out, num_wrist_seg+1); // notice that this is the number of segments, not the segment ID!!!
  if (result < 0){
    ROS_INFO_STREAM("FK solver failed when processing wrist link, something went wrong");
    return -1;
  }
  else{
    ROS_INFO_STREAM("FK solver succeeded for wrist link.");
  }

  // Output rotation matrix information(checked !!! )
  double x, y, z, w;
  wrist_cart_out.M.GetQuaternion(x, y, z, w);
  std::cout << "Wrist position is: " << wrist_cart_out.p.data[0] << " " << wrist_cart_out.p.data[1] << " " << wrist_cart_out.p.data[2] << std::endl;
  std::cout << "Elbow position is: " << elbow_cart_out.p.data[0] << " " << elbow_cart_out.p.data[1] << " " << elbow_cart_out.p.data[2] << std::endl;
  std::cout << "Quaternion of left_ee_link under current joint configuration is: " << x << " " << y << " " << z << " " << w << std::endl;
  std::cout << "Flatten rotation matrix of the left_ee_link under current joint configuration is: ";
  for (int j = 0; j < 9; ++j)
    std::cout << wrist_cart_out.M.data[j] << " ";


  return 0;

}

