// For ROS
#include "ros/ros.h"
#include <iostream>

// Sync message
#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Messages
//#include <arm_hand_capture/GloveState.h>
//#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseStamped.h"


using namespace message_filters;
using namespace geometry_msgs;



class TimeSyncAndPublish
{
  public:
    TimeSyncAndPublish();
    ~TimeSyncAndPublish(){}; // destructor
    void callback(const PoseStampedConstPtr& topic_1_msg, const PoseStampedConstPtr& topic_2_msg, const PoseStampedConstPtr& topic_3_msg);

  protected:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};


void TimeSyncAndPublish::callback(const PoseStampedConstPtr& topic_1_msg, const PoseStampedConstPtr& topic_2_msg, const PoseStampedConstPtr& topic_3_msg)
{

  // Display the time-synced results
  ROS_INFO_STREAM("========== Message Pack ==========");
  ROS_INFO_STREAM("Topic 1: At time " << topic_1_msg->header.stamp.toSec() << " , with pos = [" << topic_1_msg->pose.position.x << ", " << topic_1_msg->pose.position.y << ", "  << topic_1_msg->pose.position.z << "].");
  ROS_INFO_STREAM("Topic 2: At time " << topic_2_msg->header.stamp.toSec() << " , with pos = [" << topic_2_msg->pose.position.x << ", " << topic_2_msg->pose.position.y << ", "  << topic_2_msg->pose.position.z << "].");
  ROS_INFO_STREAM("Topic 3: At time " << topic_3_msg->header.stamp.toSec() << " , with pos = [" << topic_3_msg->pose.position.x << ", " << topic_3_msg->pose.position.y << ", "  << topic_3_msg->pose.position.z << "].");

  // Construct a combined result
  PoseStamped output;
  output.header.stamp = topic_1_msg->header.stamp;
  output.pose.position.x = topic_1_msg->pose.position.x;
  output.pose.position.y = topic_2_msg->pose.position.y;
  output.pose.position.z = topic_3_msg->pose.position.z;

  // Publish the combined data
  pub_.publish(output);

}


TimeSyncAndPublish::TimeSyncAndPublish()
{

  // Set up a publisher
  pub_ = n_.advertise<geometry_msgs::PoseStamped>("time_synced_arm_hand_state", 100);
  

  // ApproximateTime synchronizer
  message_filters::Subscriber<PoseStamped> topic_1_sub(n_, "/topic_to_sync_1", 100);
  message_filters::Subscriber<PoseStamped> topic_2_sub(n_, "/topic_to_sync_2", 100);
  message_filters::Subscriber<PoseStamped> topic_3_sub(n_, "/topic_to_sync_3", 100);

  // Approximate Time sync, how accurate is it???
  typedef message_filters::sync_policies::ApproximateTime<PoseStamped, PoseStamped, PoseStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), topic_1_sub, topic_2_sub, topic_3_sub);

  sync.registerCallback(boost::bind(&TimeSyncAndPublish::callback, this, _1, _2, _3)); 

  // Call this!!
  ros::spin();


}


int main(int argc, char** argv)
{

  // Initialize a ROS node
  ros::init(argc, argv, "sync_message_topic");

  
  // Instantiate an instance of the class to call its default constructor
  TimeSyncAndPublish time_sync_and_publish;


  return 0;
}

