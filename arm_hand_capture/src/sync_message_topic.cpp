// For ROS
#include "ros/ros.h"

// Sync message
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Messages
#include <arm_hand_capture/GloveState.h>
#include <.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
  // Solve all of perception here...
}


int main(int argc, char** argv)
{

  // Initialize a ROS node
  ros::init(argc, argv, "sync_message_topic");
  ros::NodeHandle nh;

  // Time synchronizer
  message_filters::Subscriber<Image> right_upperarm_sub(nh, "image", 1);
  message_filters::Subscriber<Image> right_forearm_sub(nh, "image", 1);
  message_filters::Subscriber<Image> right_hand_sub(nh, "image", 1);
  message_filters::Subscriber<CameraInfo> right_finger_sub(nh, "camera_info", 1);


  message_filters::TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

