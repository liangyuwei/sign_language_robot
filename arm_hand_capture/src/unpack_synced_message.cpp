#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

#include <arm_hand_capture/DualArmDualHandState.h>


void poseCallback(const arm_hand_capture::DualArmDualHandStateConstPtr& msg){

  // Prepare transforma information 
  static tf::TransformBroadcaster br;
  tf::Transform r_up_transform;

  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);

  // Broadcast the transforms
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}


int main(int argc, char** argv){

  // Initialize a ROS node
  ros::init(argc, argv, "unpack_synced_message_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/dual_arms_dual_hands_state", 100, &poseCallback);

  ros::spin();
  return 0;
};
