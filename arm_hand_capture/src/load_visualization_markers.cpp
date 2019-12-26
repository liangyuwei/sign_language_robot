// For ROS
#include <ros/ros.h>
#include <ros/time.h>

// Others
#include <vector>

// Message types
#include <arm_hand_capture/DualArmDualHandState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


class GetPosAndVisualize
{

  public:
    GetPosAndVisualize();
    ~GetPosAndVisualize(){};
    void posCallback(const arm_hand_capture::DualArmDualHandStateConstPtr& msg);


  protected:
    ros::NodeHandle n_;
    ros::Subscriber pos_sub_;    
    ros::Publisher vis_pub_;

};


/* Get the locations of each frame, and set proper visualization markers */
void GetPosAndVisualize::posCallback(const arm_hand_capture::DualArmDualHandStateConstPtr& msg)
{

  // Prep
  visualization_msgs::MarkerArray marker_array;
  //std::vector<visualization_msgs::Marker> marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "sign_language_robot_ns"; // namespace?


  // Draw shoulders
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  Eigen::Vector3d r_up_pos = {msg->right_upperarm_pose.pose.position.x, msg->right_upperarm_pose.pose.position.y, msg->right_upperarm_pose.pose.position.z};
  Eigen::Vector3d l_up_pos = {msg->left_upperarm_pose.pose.position.x, msg->left_upperarm_pose.pose.position.y, msg->left_upperarm_pose.pose.position.z};
  Eigen::Vector3d base_z = {0.0, 0.0, 1.0};
  Eigen::Quaterniond quat_shoulders = Eigen::Quaterniond::FromTwoVectors(base_z, l_up_pos - r_up_pos);
  marker.pose.orientation.x = quat_shoulders.x();
  marker.pose.orientation.y = quat_shoulders.y();
  marker.pose.orientation.z = quat_shoulders.z();
  marker.pose.orientation.w = quat_shoulders.w();
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = (r_up_pos - l_up_pos).norm();
  marker.color.a = 1.0; // visable
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker_array.markers.push_back(marker);

  // Draw upperarm links
  //marker.id = 0;


  // Draw forearm links
  //marker.id = 0;


  // Draw hands
  //marker.id = 0;


  // Publish them
  vis_pub_.publish(marker_array);

}




GetPosAndVisualize::GetPosAndVisualize()
{
  // Initialize a subscriber
  ROS_INFO_STREAM("Waiting for /dual_arms_dual_hands_state to come up...");
  pos_sub_ = n_.subscribe<arm_hand_capture::DualArmDualHandState>("/dual_arms_dual_hands_state", 100, boost::bind(&GetPosAndVisualize::posCallback, this, _1));

  // Initialize a publisher
  ROS_INFO_STREAM("Bring up a publisher /visualization_marker_array...");  
  vis_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 0);
  ROS_INFO_STREAM("Ready to publish visualization markers.");


  // Spin, the  whole code ends here
  ros::spin();
}




int main(int argc, char** argv){

  // Initialize a ROS node
  ros::init(argc, argv, "load_visualization_markers_node");
  
  // Instantiate an object
  GetPosAndVisualize get_pos_and_visualize;

  ros::spin();
  return 0;
}


