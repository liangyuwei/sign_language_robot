#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <arm_hand_capture/DualArmDualHandState.h>

#include <geometry_msgs/PoseStamped.h>


// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// boost::bind
#include <boost/bind.hpp>

using namespace Eigen;
using namespace geometry_msgs;
using namespace arm_hand_capture;
using namespace tf;

/* The positions of the origins of all the frames have already been transformed to z-up frame, now the work left is to transform the local frames' orientation. */


/* Transform local frames to match the configuration of UR5 */
geometry_msgs::Quaternion transform_to_ur5(geometry_msgs::Quaternion quat_in, Quaterniond quat_shift)
{
  // Quaternion to shift
  Quaterniond quat_tmp(quat_in.w, quat_in.x, quat_in.y, quat_in.z);
  
  // Resultant quaternion
  Quaterniond quat_res = quat_shift * quat_tmp;

  // Return the result
  geometry_msgs::Quaternion quat_out = quat_in;
  quat_out.x = quat_res.x();
  quat_out.y = quat_res.y();
  quat_out.z = quat_res.z();
  quat_out.w = quat_res.w();
  return quat_out;
}


/* Translate the positions of shoulders(base) to match that of UR5 world */
geometry_msgs::Point translate_to_ur5(geometry_msgs::Point pos_in, geometry_msgs::Point cur_base, geometry_msgs::Point new_base)
{
  // A new one
  geometry_msgs::Point pos_out = pos_in;

  // Translate to new base
  pos_out.x = pos_out.x - cur_base.x + new_base.x;
  pos_out.y = pos_out.y - cur_base.y + new_base.y;
  pos_out.z = pos_out.z - cur_base.z + new_base.z;
  
  // Return the result
  return pos_out;
}


class TransformUR5AndRepublish
{
  public:
    TransformUR5AndRepublish();
    ~TransformUR5AndRepublish(){};
    void transformCallback(const arm_hand_capture::DualArmDualHandStateConstPtr& msg);

  protected:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
  
    // fixed transforms
    Matrix3d rotm_shift_l_up;
    Matrix3d rotm_shift_l_fr;
    Matrix3d rotm_shift_l_hd;
    Matrix3d rotm_shift_r_up;
    Matrix3d rotm_shift_r_fr;
    Matrix3d rotm_shift_r_hd;

    Quaterniond quat_shift_l_up;
    Quaterniond quat_shift_l_fr;
    Quaterniond quat_shift_l_hd;
    Quaterniond quat_shift_r_up;
    Quaterniond quat_shift_r_fr;
    Quaterniond quat_shift_r_hd;

};


void TransformUR5AndRepublish::transformCallback(const arm_hand_capture::DualArmDualHandStateConstPtr& msg)
{

  // Initialize a new combined messge here
  arm_hand_capture::DualArmDualHandState output = *msg;
  

  // Transform the local frames
  output.left_upperarm_pose.pose.orientation = transform_to_ur5(msg->left_upperarm_pose.pose.orientation, quat_shift_l_up);
  output.left_forearm_pose.pose.orientation = transform_to_ur5(msg->left_forearm_pose.pose.orientation, quat_shift_l_fr);
  output.left_hand_pose.pose.orientation = transform_to_ur5(msg->left_hand_pose.pose.orientation, quat_shift_l_hd);

  output.right_upperarm_pose.pose.orientation = transform_to_ur5(msg->right_upperarm_pose.pose.orientation, quat_shift_r_up);
  output.right_forearm_pose.pose.orientation = transform_to_ur5(msg->right_forearm_pose.pose.orientation, quat_shift_r_fr);
  output.right_hand_pose.pose.orientation = transform_to_ur5(msg->right_hand_pose.pose.orientation, quat_shift_r_hd);

  // Translate to new base to match the UR5 world
  geometry_msgs::Point left_base, right_base; // locations of shoulders in UR5 world
  left_base.x = -0.06; left_base.y = 0.235; left_base.z = 0.395;
  right_base.x = -0.06; right_base.y = -0.235; right_base.z = 0.395;  

  output.left_upperarm_pose.pose.position = translate_to_ur5(msg->left_upperarm_pose.pose.position, msg->left_upperarm_pose.pose.position, left_base);
  output.left_forearm_pose.pose.position = translate_to_ur5(msg->left_forearm_pose.pose.position, msg->left_upperarm_pose.pose.position, left_base);
  output.left_hand_pose.pose.position = translate_to_ur5(msg->left_hand_pose.pose.position, msg->left_upperarm_pose.pose.position, left_base);

  output.right_upperarm_pose.pose.position = translate_to_ur5(msg->right_upperarm_pose.pose.position, msg->right_upperarm_pose.pose.position, right_base);
  output.right_forearm_pose.pose.position = translate_to_ur5(msg->right_forearm_pose.pose.position, msg->right_upperarm_pose.pose.position, right_base);
  output.right_hand_pose.pose.position = translate_to_ur5(msg->right_hand_pose.pose.position, msg->right_upperarm_pose.pose.position, right_base);


  // Publish the new results
  ROS_INFO_STREAM("Republishing the newly transformed message...");
  pub_.publish(output);

}


TransformUR5AndRepublish::TransformUR5AndRepublish()
{
  // Initialize a subscriber
  ROS_INFO_STREAM("Waiting for /dual_arms_dual_hands_state to come up...");
  sub_ = n_.subscribe<arm_hand_capture::DualArmDualHandState>("/dual_arms_dual_hands_state", 100, boost::bind(&TransformUR5AndRepublish::transformCallback, this, _1));

  // Initialize a publisher
  ROS_INFO_STREAM("Bring up a publisher /dual_arms_dual_hands_UR5_state...");  
  pub_ = n_.advertise<DualArmDualHandState>("dual_arms_dual_hands_UR5_state", 100);
  ROS_INFO_STREAM("Ready to transform the synced message to UR5 local frames.");

  // Initialize the rotation matrices and quaternions
  rotm_shift_l_up << 0.0, -1.0, 0.0,
                     1.0, 0.0, 0.0, 
                     0.0, 0.0, 1.0; // from manual calculation...
  rotm_shift_l_fr << 0.0, -1.0, 0.0,
                     1.0, 0.0, 0.0, 
                     0.0, 0.0, 1.0;
  rotm_shift_l_hd << 0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0, 
                     1.0, 0.0, 0.0;
  rotm_shift_r_up << 0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0, 
                     0.0, 0.0, 1.0;
  rotm_shift_r_fr << 0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0, 
                     0.0, 0.0, 1.0;
  rotm_shift_r_hd << 0.0, 1.0, 0.0,
                     0.0, 0.0, -1.0, 
                    -1.0, 0.0, 0.0;
  quat_shift_l_up = Quaterniond(rotm_shift_l_up);
  quat_shift_l_fr = Quaterniond(rotm_shift_l_fr);
  quat_shift_l_hd = Quaterniond(rotm_shift_l_hd);
  quat_shift_r_up = Quaterniond(rotm_shift_r_up);
  quat_shift_r_fr = Quaterniond(rotm_shift_r_fr);
  quat_shift_r_hd = Quaterniond(rotm_shift_r_hd);


  // Spin, the whole code ends here
  ros::spin();
}


int main(int argc, char** argv){

  // Initialize a ROS node
  ros::init(argc, argv, "transform_to_ur5_node");


  // Instantiate an object of the class to initialize the process
  TransformUR5AndRepublish transform_ur5_and_republish;


  return 0;
}


