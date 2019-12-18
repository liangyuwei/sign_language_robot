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


    void callback(const PoseStampedConstPtr& right_upperarm_pose, \
                  const PoseStampedConstPtr& right_forearm_pose, \
                  const PoseStampedConstPtr& right_hand_pose, \
                  const GloveStateConstPtr& right_finger_state)
    {
      // Display the time-synced results
      ROS_INFO("========== Message Pack ==========\n");
      ROS_INFO("RightUpperarm: At time %f\n", right_upperarm_pose->header.stamp.toSec());
      ROS_INFO("RightForearm: At time %f\n", right_forearm_pose->header.stamp.toSec());
      ROS_INFO("RightHand: At time %f\n", right_hand_pose->header.stamp.toSec());
      ROS_INFO("RightGlove: At time %f\n", right_finger_state->header.stamp.toSec());      

      // Publish the combined data on a another topic
      //pub_.publish(output)


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



  ros::spin();

  return 0;
}

