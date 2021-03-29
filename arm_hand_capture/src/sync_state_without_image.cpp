// For ROS
#include "ros/ros.h"

// Sync message
#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Messages
#include <arm_hand_capture/GloveState.h>
#include <arm_hand_capture/DualArmDualHandState.h>
#include <geometry_msgs/PoseStamped.h>

// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace message_filters;
using namespace geometry_msgs;
using namespace arm_hand_capture;
using namespace Eigen;

class TimeSyncStateAndPublish
{
  public:
    TimeSyncStateAndPublish();
    ~TimeSyncStateAndPublish(){}; // destructor
    void callback(const PoseStampedConstPtr& right_upperarm_msg, 
                  const PoseStampedConstPtr& right_forearm_msg, 
                  const PoseStampedConstPtr& right_hand_msg, 
                  const PoseStampedConstPtr& left_upperarm_msg, 
                  const PoseStampedConstPtr& left_forearm_msg, 
                  const PoseStampedConstPtr& left_hand_msg, 
                  const GloveStateConstPtr& glove_msg);
    Pose transform_to_z_up_frame(const Pose& pose_y_up, Quaterniond quat_shift);

  protected:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};


Pose TimeSyncStateAndPublish::transform_to_z_up_frame(const Pose& pose_y_up, Quaterniond quat_shift)
{
  // Pose msg type is constructed by: position and orientation, both of which require transformation to z-up frame

  // Transform orientation
  Quaterniond quat_y_up = Quaterniond(pose_y_up.orientation.w, pose_y_up.orientation.x, pose_y_up.orientation.y, pose_y_up.orientation.z).normalized();
  Quaterniond quat_z_up = quat_shift * quat_y_up;//quat_y_up * quat_shift; // ?

  // Transform position
  Vector3d pos_y_up = {pose_y_up.position.x, pose_y_up.position.y, pose_y_up.position.z};
  Vector3d pos_z_up = quat_shift * pos_y_up;

  // Copy to a new Pose object
  Pose pose_z_up;
  pose_z_up.position.x = pos_z_up[0];
  pose_z_up.position.y = pos_z_up[1];
  pose_z_up.position.z = pos_z_up[2];
  pose_z_up.orientation.x = quat_z_up.x();
  pose_z_up.orientation.y = quat_z_up.y();
  pose_z_up.orientation.z = quat_z_up.z();
  pose_z_up.orientation.w = quat_z_up.w();


  return pose_z_up;

}

void TimeSyncStateAndPublish::callback(const PoseStampedConstPtr& right_upperarm_msg, 
                  const PoseStampedConstPtr& right_forearm_msg, 
                  const PoseStampedConstPtr& right_hand_msg, 
                  const PoseStampedConstPtr& left_upperarm_msg, 
                  const PoseStampedConstPtr& left_forearm_msg, 
                  const PoseStampedConstPtr& left_hand_msg, 
                  const GloveStateConstPtr& glove_msg)
{

  // Display the time-synced results
  // ROS_INFO_STREAM("========== Time-Sync Callback invoked ==========");
  //ROS_INFO_STREAM("Topic 1: At time " << right_upperarm_msg->header.stamp.toSec());


  // Transform to z-up frame
  Matrix3d rotm_shift;
  rotm_shift << 0.0, 0.0, 1.0,
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0; // from manual calculation...
  Quaterniond quat_shift(rotm_shift);



  // Construct a combined result
  DualArmDualHandState output;
  output.right_upperarm_pose.header = right_upperarm_msg->header;
  output.right_upperarm_pose.pose = this->transform_to_z_up_frame(right_upperarm_msg->pose, quat_shift);
  output.right_forearm_pose.header = right_forearm_msg->header;
  output.right_forearm_pose.pose = this->transform_to_z_up_frame(right_forearm_msg->pose, quat_shift);
  output.right_hand_pose.header = right_hand_msg->header;
  output.right_hand_pose.pose = this->transform_to_z_up_frame(right_hand_msg->pose, quat_shift);

  output.left_upperarm_pose.header = left_upperarm_msg->header;
  output.left_upperarm_pose.pose = this->transform_to_z_up_frame(left_upperarm_msg->pose, quat_shift);
  output.left_forearm_pose.header = left_forearm_msg->header;
  output.left_forearm_pose.pose = this->transform_to_z_up_frame(left_forearm_msg->pose, quat_shift);
  output.left_hand_pose.header = left_hand_msg->header;
  output.left_hand_pose.pose = this->transform_to_z_up_frame(left_hand_msg->pose, quat_shift);


  output.glove_state.header = glove_msg->header;
  output.glove_state.left_glove_state = glove_msg->left_glove_state;
  output.glove_state.right_glove_state = glove_msg->right_glove_state;

  // Publish the combined data
  pub_.publish(output);
}


TimeSyncStateAndPublish::TimeSyncStateAndPublish()
{
  // Set up a publisher to publish the combined output
  pub_ = n_.advertise<DualArmDualHandState>("dual_arms_dual_hands_state", 100);

  // ApproximateTime synchronizer
  message_filters::Subscriber<PoseStamped> right_upperarm_sub(n_, "/vrpn_client_node/RightUpperarm/pose", 100);
  message_filters::Subscriber<PoseStamped> right_forearm_sub(n_, "/vrpn_client_node/RightForearm/pose", 100);
  message_filters::Subscriber<PoseStamped> right_hand_sub(n_, "/vrpn_client_node/RightHand/pose", 100);
  message_filters::Subscriber<GloveState> glove_sub(n_, "/wiseglove_state_pub", 100); // actually, it contains both hands' data


  message_filters::Subscriber<PoseStamped> left_upperarm_sub(n_, "/vrpn_client_node/LeftUpperarm/pose", 100);
  message_filters::Subscriber<PoseStamped> left_forearm_sub(n_, "/vrpn_client_node/LeftForearm/pose", 100);
  message_filters::Subscriber<PoseStamped> left_hand_sub(n_, "/vrpn_client_node/LeftHand/pose", 100);


  // Approximate Time sync, how accurate is it???
  typedef message_filters::sync_policies::ApproximateTime<PoseStamped, PoseStamped, PoseStamped, PoseStamped, PoseStamped, PoseStamped, GloveState> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), right_upperarm_sub, right_forearm_sub, right_hand_sub, left_upperarm_sub, left_forearm_sub, left_hand_sub, glove_sub);

  sync.registerCallback(boost::bind(&TimeSyncStateAndPublish::callback, this, _1, _2, _3, _4, _5, _6, _7)); 

  // Call this!!
  ros::spin();

}


int main(int argc, char** argv)
{

  // Initialize a ROS node
  ros::init(argc, argv, "dual_arms_dual_hands_state_without_image_node");

  
  // Instantiate an instance of the class to call its default constructor
  TimeSyncStateAndPublish time_sync_state_and_publish;


  return 0;
}



