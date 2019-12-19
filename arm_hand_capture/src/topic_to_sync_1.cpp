#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"


int main(int argc, char **argv)
{


  ros::init(argc, argv, "topic_to_syn_1_node");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("topic_to_sync_1", 1000);

  ros::Rate loop_rate(10); // sleep enough time(leftover) to hit 10Hz publish rate


  int count = 0;
  while (ros::ok())
  {
    // Set up the message content
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now(); // add timestamp
    pose.pose.position.x = (double)count; // specify position

    //ROS_INFO("%s", msg.data.c_str());

    // Publish the message
    pub.publish(pose);

    ros::spinOnce();

    loop_rate.sleep(); // sleep through the leftover time to hit 10Hz publish rate
    ++count;
  }


  return 0;
}
