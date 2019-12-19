// For ROS
#include "ros/ros.h"

// Sync message
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Messages
#include <arm_hand_capture/GloveState.h>
#include <geometry_msgs/PoseStamped.h>

using namespace message_filters;
using namespace geometry_msgs;
using namespace arm_hand_capture;



void callback(const PoseStampedConstPtr& topic_1_msg,
              const PoseStampedConstPtr& topic_2_msg)
{
    // Display the time-synced results
      ROS_INFO_STREAM("========== Message Pack ==========");
      ROS_INFO_STREAM("Topic 1: At time " << topic_1_msg->header.stamp.toSec() << " , with pos = [" << topic_1_msg->pose.position.x << ", " << topic_1_msg->pose.position.y << ", "  << topic_1_msg->pose.position.z << "].");
      ROS_INFO_STREAM("Topic 2: At time " << topic_2_msg->header.stamp.toSec() << " , with pos = [" << topic_2_msg->pose.position.x << ", " << topic_2_msg->pose.position.y << ", "  << topic_2_msg->pose.position.z << "].");

      // Publish the combined data on a another topic
      //pub_.publish(output)

}

void callback(const PoseStampedConstPtr& right_upperarm_pose, 
              const PoseStampedConstPtr& right_forearm_pose, 
              const PoseStampedConstPtr& right_hand_pose, 
              const GloveStateConstPtr& right_finger_state)
{
  // Display the time-synced results
  ROS_INFO_STREAM("========== Message Pack ==========\n");
  ROS_INFO_STREAM("RightUpperarm: At time " << right_upperarm_pose->header.stamp.toSec());
  ROS_INFO_STREAM("RightForearm: At time " << right_forearm_pose->header.stamp.toSec());
  ROS_INFO_STREAM("RightHand: At time " << right_hand_pose->header.stamp.toSec());
  ROS_INFO_STREAM("RightGlove: At time " << right_finger_state->header.stamp.toSec());      

  // Publish the combined data on a another topic
  //pub_.publish(output)

}


class TimeSyncAndPublish
{

  public:
    TimeSyncAndPublish() // Init
    {
      // Time synchronizer
      message_filters::Subscriber<PoseStamped> right_upperarm_sub(n_, "/vrpn_client_node/RightUpperarm/pose", 1);
      message_filters::Subscriber<PoseStamped> right_forearm_sub(n_, "/vrpn_client_node/RightForearm", 1);
      message_filters::Subscriber<PoseStamped> right_hand_sub(n_, "/vrpn_client_node/RightHand", 1);
      message_filters::Subscriber<GloveState> right_finger_sub(n_, "/wiseglove_state_pub", 1);
      //message_filters::Subscriber<PoseStamped> left_upperarm_sub(n_, "/vrpn_client_node/RightUpperarm/pose", 1);
      //message_filters::Subscriber<PoseStamped> left_forearm_sub(n_, "/vrpn_client_node/RightForearm", 1);
      //message_filters::Subscriber<PoseStamped> left_hand_sub(n_, "/vrpn_client_node/RightHand", 1);
      //message_filters::Subscriber<GloveState> left_finger_sub(n_, "/wiseglove_state_pub", 1);

      message_filters::TimeSynchronizer<PoseStamped, PoseStamped, PoseStamped, GloveState> sync(right_upperarm_sub, right_forearm_sub, right_hand_sub, right_finger_sub, 10);

      sync.registerCallback(&TimeSyncAndPublish::callback);//(;boost::bind(&TimeSyncAndPublish::callback, this, _1, _2));


      // Publish to a topic
      //pub_ = n_.advertise<COMBINED_MESSAGE_TYPE>()


    }





  private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;


};






int main(int argc, char** argv)
{

  // Initialize a ROS node
  ros::init(argc, argv, "sync_message_topic");


  // Set up Approximate Time Synchronizer



  ros::spin();

  return 0;
}

