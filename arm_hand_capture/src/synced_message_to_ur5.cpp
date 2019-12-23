#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <arm_hand_capture/DualArmDualHandState.h>

#include <geometry_msgs/PoseStamped.h>


// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace geometry_msgs;
using namespace tf;

/* The positions of the origins of all the frames have already been transformed to z-up frame, now the work left is to transform the local frames' orientation. */

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


class TransformUR5AndRepublish
{
  public:
    TransformUR5AndRepublish();
    ~TransformUR5AndRepublish(){};
    void transformCallback(arm_hand_capture::DualArmDualHandStateConstPtr& msg);

  protected:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};


void TransformUR5AndRepublish::transformCallback(arm_hand_capture::DualArmDualHandStateConstPtr& msg)
{

  // Initialize a new combined messge here
  arm_hand_capture::DualArmDualHandState output = *msg;
  

  // Add all the transform work here
  Quaterniond quat_shift_l_up = 
  Quaterniond quat_shift_l_fr = 
  Quaterniond quat_shift_l_hd = 
  Quaterniond quat_shift_r_up = 
  Quaterniond quat_shift_r_fr = 
  Quaterniond quat_shift_r_hd = 


  // Publish the new results
  pub_.publish();

}


TransformUR5AndRepublish::TransformUR5AndRepublish()
{
  // Initialize a subscriber
  ROS_INFO_STREAM("Waiting for /dual_arms_dual_hands_state to come up...");
  sub_ = n.subscribe("/dual_arms_dual_hands_state", 100, &TransformUR5AndRepublish::transformCallback);

  // Initialize a publisher
  ROS_INFO_STREAM("Bring up a publisher /dual_arms_dual_hands_state...");  
  pub_ = n_.advertise<DualArmDualHandState>("dual_arms_dual_hands_UR5_state", 100);
  ROS_INFO_STREAM("Ready to transform the synced message to UR5 local frames.");

  // Spin, the whole code ends here
  ros::spin();
}








void transformCallback(const arm_hand_capture::DualArmDualHandStateConstPtr& msg){

  // Disp
  ROS_INFO_STREAM("Get one message, transform and re-publish...");

  // Prepare transform information -
  static tf::TransformBroadcaster br;
  tf::Transform r_up_transform = transform_to_ur5(msg->right_upperarm_pose);
  tf::Transform r_fr_transform = transform_to_ur5(msg->right_forearm_pose);
  tf::Transform r_hd_transform = transform_to_ur5(msg->right_hand_pose);

  tf::Transform l_up_transform = transform_to_ur5(msg->left_upperarm_pose);
  tf::Transform l_fr_transform = transform_to_ur5(msg->left_forearm_pose);
  tf::Transform l_hd_transform = transform_to_ur5(msg->left_hand_pose);


  // Broadcast the transforms
  ros::Time timestamp = msg->right_upperarm_pose.header.stamp;
  br.sendTransform(tf::StampedTransform(r_up_transform, timestamp, "world", "right_upperarm"));
  br.sendTransform(tf::StampedTransform(r_fr_transform, timestamp, "world", "right_forearm"));
  br.sendTransform(tf::StampedTransform(r_hd_transform, timestamp, "world", "right_hand"));

  br.sendTransform(tf::StampedTransform(l_up_transform, timestamp, "world", "left_upperarm"));
  br.sendTransform(tf::StampedTransform(l_fr_transform, timestamp, "world", "left_forearm"));
  br.sendTransform(tf::StampedTransform(l_hd_transform, timestamp, "world", "left_hand"));

}


int main(int argc, char** argv){

  // Initialize a ROS node
  ros::init(argc, argv, "transform_to_ur5_node");

  return 0;
}


