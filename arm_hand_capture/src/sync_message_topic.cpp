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

using namespace message_filters;
using namespace geometry_msgs;
using namespace arm_hand_capture;



class TimeSyncAndPublish
{
  public:
    TimeSyncAndPublish();
    ~TimeSyncAndPublish(){}; // destructor
    void callback(const PoseStampedConstPtr& right_upperarm_msg, 
                  const PoseStampedConstPtr& right_forearm_msg, 
                  const PoseStampedConstPtr& right_hand_msg, 
                  const GloveStateConstPtr& right_glove_msg);

  protected:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};


void TimeSyncAndPublish::callback(const PoseStampedConstPtr& right_upperarm_msg, 
                  const PoseStampedConstPtr& right_forearm_msg, 
                  const PoseStampedConstPtr& right_hand_msg, 
                  const GloveStateConstPtr& right_glove_msg)
{

  // Display the time-synced results
  ROS_INFO_STREAM("========== Time-Sync Callback invoked ==========");
  //ROS_INFO_STREAM("Topic 1: At time " << right_upperarm_msg->header.stamp.toSec());


  // Construct a combined result
  
  DualArmDualHandState output;
  output.right_upperarm_pose.header = right_upperarm_msg->header;
  output.right_upperarm_pose.pose = right_upperarm_msg->pose;

  output.right_forearm_pose.header = right_forearm_msg->header;
  output.right_forearm_pose.pose = right_forearm_msg->pose;

  output.right_hand_pose.header = right_hand_msg->header;
  output.right_hand_pose.pose = right_hand_msg->pose;

  output.right_glove_state.header = right_glove_msg->header;
  output.right_glove_state.point = right_glove_msg->point;
  

  // Publish the combined data
  pub_.publish(output);

}


TimeSyncAndPublish::TimeSyncAndPublish()
{
  // Set up a publisher to publish the combined output
  pub_ = n_.advertise<DualArmDualHandState>("dual_arms_dual_hands_state", 100);

  // ApproximateTime synchronizer
  message_filters::Subscriber<PoseStamped> right_upperarm_sub(n_, "/vrpn_client_node/RightUpperarm/pose", 100);
  message_filters::Subscriber<PoseStamped> right_forearm_sub(n_, "/vrpn_client_node/RightForearm/pose", 100);
  message_filters::Subscriber<PoseStamped> right_hand_sub(n_, "/vrpn_client_node/RightHand/pose", 100);
  message_filters::Subscriber<GloveState> right_glove_sub(n_, "/wiseglove_state_pub", 100);

  // Approximate Time sync, how accurate is it???
  typedef message_filters::sync_policies::ApproximateTime<PoseStamped, PoseStamped, PoseStamped, GloveState> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), right_upperarm_sub, right_forearm_sub, right_hand_sub, right_glove_sub);

  sync.registerCallback(boost::bind(&TimeSyncAndPublish::callback, this, _1, _2, _3, _4)); 

  // Call this!!
  ros::spin();

}


int main(int argc, char** argv)
{

  // Initialize a ROS node
  ros::init(argc, argv, "arm_hand_time_synchronization_node");

  
  // Instantiate an instance of the class to call its default constructor
  TimeSyncAndPublish time_sync_and_publish;


  return 0;
}



