#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <arm_hand_capture/DualArmDualHandState.h>

#include <geometry_msgs/PoseStamped.h>


using namespace geometry_msgs;
using namespace tf;



tf::Transform setup_tf_transform(geometry_msgs::PoseStamped pose)
{
  // Prepare transform
  tf::Transform transform;

  // Set from pose
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  transform.setRotation(q);

  // Return the result
  return transform;
}


void poseCallback(const arm_hand_capture::DualArmDualHandStateConstPtr& msg){

  // Disp
  ROS_INFO_STREAM("Get one message, unpack and broadcast...");

  // Prepare transform information -
  static tf::TransformBroadcaster br;
  tf::Transform r_up_transform = setup_tf_transform(msg->right_upperarm_pose);
  tf::Transform r_fr_transform = setup_tf_transform(msg->right_forearm_pose);
  tf::Transform r_hd_transform = setup_tf_transform(msg->right_hand_pose);

  tf::Transform l_up_transform = setup_tf_transform(msg->left_upperarm_pose);
  tf::Transform l_fr_transform = setup_tf_transform(msg->left_forearm_pose);
  tf::Transform l_hd_transform = setup_tf_transform(msg->left_hand_pose);


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
  ros::init(argc, argv, "unpack_synced_message_node");
  ros::NodeHandle n;
  ROS_INFO_STREAM("Waiting for /dual_arms_dual_hands_state to come up...");
  ros::Subscriber sub = n.subscribe("/dual_arms_dual_hands_YuMi_state", 100, &poseCallback);
  // /dual_arms_dual_hands_UR5_state
  ROS_INFO_STREAM("Ready to unpack the sync-ed message.");


  ros::spin();
  return 0;
}


