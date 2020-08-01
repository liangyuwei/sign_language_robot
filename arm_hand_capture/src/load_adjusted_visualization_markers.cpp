// For ROS
#include <ros/ros.h>
#include <ros/time.h>

// Others
#include <vector>

// Message types
#include <arm_hand_capture/DualArmDualHandStateWithImage.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/* Set a marker connecting the two points given with a sphere */
// Note that nothing else but the pos is changed!!!
void set_segment(Eigen::Vector3d point_1, Eigen::Vector3d point_2, double scale_x, double scale_y, visualization_msgs::Marker& marker, bool specify_quat, Eigen::Quaterniond quat)
{
  // Set positions
  marker.pose.position.x = (point_1[0] + point_2[0]) / 2.0;
  marker.pose.position.y = (point_1[1] + point_2[1]) / 2.0;
  marker.pose.position.z = (point_1[2] + point_2[2]) / 2.0;

  // Set orientation
  if (specify_quat)
  {
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
  }
  else // use only two points to set marker orientation (x and y axes are probably arbitrary...)
  {
    Eigen::Vector3d base_z = {0.0, 0.0, 1.0};
    Eigen::Quaterniond quat_shoulders = Eigen::Quaterniond::FromTwoVectors(base_z, point_1 - point_2);
    marker.pose.orientation.x = quat_shoulders.x();
    marker.pose.orientation.y = quat_shoulders.y();
    marker.pose.orientation.z = quat_shoulders.z();
    marker.pose.orientation.w = quat_shoulders.w();
  }
  
  marker.scale.x = scale_x; //0.08; //0.04;
  marker.scale.y = scale_y; //0.04; //0.08;  // should be reverted...
  marker.scale.z = (point_1 - point_2).norm();

}

class GetPosAndVisualize
{

  public:
    GetPosAndVisualize();
    ~GetPosAndVisualize(){};
    void posCallback(const arm_hand_capture::DualArmDualHandStateWithImageConstPtr& msg);


  protected:
    ros::NodeHandle n_;
    ros::Subscriber pos_sub_;    
    ros::Publisher vis_pub_;

};


/* Get the locations of each frame, and set proper visualization markers */
void GetPosAndVisualize::posCallback(const arm_hand_capture::DualArmDualHandStateWithImageConstPtr& msg)
{

  // Prepare Marker message
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "adjusted_movement"; // namespace?
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.a = 1.0; // visible
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;


  // Get temporary parameters
  Eigen::Vector3d r_up_pos = {msg->right_upperarm_pose.pose.position.x, msg->right_upperarm_pose.pose.position.y, msg->right_upperarm_pose.pose.position.z};
  Eigen::Vector3d r_fr_pos = {msg->right_forearm_pose.pose.position.x, msg->right_forearm_pose.pose.position.y, msg->right_forearm_pose.pose.position.z};
  Eigen::Vector3d r_hd_pos = {msg->right_hand_pose.pose.position.x, msg->right_hand_pose.pose.position.y, msg->right_hand_pose.pose.position.z};

  Eigen::Vector3d l_up_pos = {msg->left_upperarm_pose.pose.position.x, msg->left_upperarm_pose.pose.position.y, msg->left_upperarm_pose.pose.position.z};
  Eigen::Vector3d l_fr_pos = {msg->left_forearm_pose.pose.position.x, msg->left_forearm_pose.pose.position.y, msg->left_forearm_pose.pose.position.z};
  Eigen::Vector3d l_hd_pos = {msg->left_hand_pose.pose.position.x, msg->left_hand_pose.pose.position.y, msg->left_hand_pose.pose.position.z};


  // orientation (Quaterniond constructor, (w,x,y,z))
  Eigen::Quaterniond r_up_quat(msg->right_upperarm_pose.pose.orientation.w, msg->right_upperarm_pose.pose.orientation.x, msg->right_upperarm_pose.pose.orientation.y, msg->right_upperarm_pose.pose.orientation.z);
  Eigen::Quaterniond r_fr_quat(msg->right_forearm_pose.pose.orientation.w, msg->right_forearm_pose.pose.orientation.x, msg->right_forearm_pose.pose.orientation.y, msg->right_forearm_pose.pose.orientation.z);
  Eigen::Quaterniond r_hd_quat(msg->right_hand_pose.pose.orientation.w, msg->right_hand_pose.pose.orientation.x, msg->right_hand_pose.pose.orientation.y, msg->right_hand_pose.pose.orientation.z);

  Eigen::Quaterniond l_up_quat(msg->left_upperarm_pose.pose.orientation.w, msg->left_upperarm_pose.pose.orientation.x, msg->left_upperarm_pose.pose.orientation.y, msg->left_upperarm_pose.pose.orientation.z);
  Eigen::Quaterniond l_fr_quat(msg->left_forearm_pose.pose.orientation.w, msg->left_forearm_pose.pose.orientation.x, msg->left_forearm_pose.pose.orientation.y, msg->left_forearm_pose.pose.orientation.z);
  Eigen::Quaterniond l_hd_quat(msg->left_hand_pose.pose.orientation.w, msg->left_hand_pose.pose.orientation.x, msg->left_hand_pose.pose.orientation.y, msg->left_hand_pose.pose.orientation.z);


  double link_radius = 0.03;

  // Draw shoulders
  marker.id = 0;
  set_segment(l_up_pos, r_up_pos, link_radius, link_radius, marker, false, Eigen::Quaterniond());
  marker_array.markers.push_back(marker);


  // Draw upperarm links
  marker.id = 1; // left upperarm link
  set_segment(l_fr_pos, l_up_pos, link_radius, link_radius, marker, false, Eigen::Quaterniond());
  marker_array.markers.push_back(marker);
  marker.id = 2; // right upperarm link
  set_segment(r_fr_pos, r_up_pos, link_radius, link_radius, marker, false, Eigen::Quaterniond());
  marker_array.markers.push_back(marker);


  // Draw forearm links
  marker.id = 3; // left forearm link
  set_segment(l_hd_pos, l_fr_pos, link_radius, link_radius, marker, false, Eigen::Quaterniond());
  marker_array.markers.push_back(marker);
  marker.id = 4; // right forearm link
  set_segment(r_hd_pos, r_fr_pos, link_radius, link_radius, marker, false, Eigen::Quaterniond());
  marker_array.markers.push_back(marker);


  // Draw hands
  double scale_x = 0.08;
  double scale_y = 0.03;//0.04;
  double robot_hand_length = 0.2;//0.278; // data for inspire-robot hand
  Eigen::Vector3d hand_axis(0.0, 0.0, 1.0); // along z-axis

  marker.id = 5;
  Eigen::Quaterniond l_wrist_quat(msg->left_hand_pose.pose.orientation.w,
                                  msg->left_hand_pose.pose.orientation.x,
                                  msg->left_hand_pose.pose.orientation.y,
                                  msg->left_hand_pose.pose.orientation.z);

  Eigen::Vector3d l_hand_tip = l_hd_pos + robot_hand_length * l_wrist_quat.toRotationMatrix() * hand_axis;
  set_segment(l_hd_pos, l_hand_tip, scale_x, scale_y, marker, true, l_wrist_quat);
  marker_array.markers.push_back(marker);  
  
  marker.id = 6;
  Eigen::Quaterniond r_wrist_quat(msg->right_hand_pose.pose.orientation.w,
                                  msg->right_hand_pose.pose.orientation.x,
                                  msg->right_hand_pose.pose.orientation.y,
                                  msg->right_hand_pose.pose.orientation.z);
  Eigen::Vector3d r_hand_tip = r_hd_pos + robot_hand_length * r_wrist_quat.toRotationMatrix() * hand_axis;
  set_segment(r_hd_pos, r_hand_tip, scale_x, scale_y, marker, true, r_wrist_quat);
  marker_array.markers.push_back(marker);  


  // Publish them
  vis_pub_.publish(marker_array);

}




GetPosAndVisualize::GetPosAndVisualize()
{
  // Initialize a subscriber
  ROS_INFO_STREAM("Waiting for /dual_arms_dual_hands_state to come up...");
  pos_sub_ = n_.subscribe<arm_hand_capture::DualArmDualHandStateWithImage>("/dual_arms_dual_hands_state_with_image_adjusted_movement", 100, boost::bind(&GetPosAndVisualize::posCallback, this, _1));

  // Initialize a publisher
  ROS_INFO_STREAM("Bring up a publisher /visualization_marker_array...");  
  vis_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100);
  ROS_INFO_STREAM("Ready to publish visualization markers.");


  // Spin, the  whole code ends here
  ros::spin();
}




int main(int argc, char** argv){

  // Initialize a ROS node
  ros::init(argc, argv, "load_visualization_markers_node");
  
  // Instantiate an object
  GetPosAndVisualize get_pos_and_visualize;

  return 0;
}


