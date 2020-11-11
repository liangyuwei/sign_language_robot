// ROS
#include <ros/ros.h>
// Messages
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "realtime_motion_retargeting/ControlMsg.h"
#include "arm_hand_capture/DualArmDualHandStateWithImage.h"
// For moveit
#include <moveit/move_group_interface/move_group_interface.h>
// For Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// My header files
#include "config.h"
#include "NullSpaceControl.h"
#include "util_functions.h"

using namespace cfg;

const static double scale = 1.369;
const static double l_offs[3] = {0.1436, 0.2072, 0.5275};
const static Eigen::Vector3d l_offset = 
    matrix_helper::create_translation_vector(l_offs[0],l_offs[1],l_offs[2]);
const static double r_offs[3] = {0.1436, -0.2072, 0.5275};
const static Eigen::Vector3d r_offset = 
    matrix_helper::create_translation_vector(r_offs[0],r_offs[1],r_offs[2]);

// static const std::string PLANNING_GROUP = "dual_arm_with_hands";
// moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
// const robot_state::JointModelGroup* joint_model_group = 
//     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

Eigen::Vector3d PointMsg2Pos(geometry_msgs::Point point_msg)
{
    Eigen::Vector3d pos;
    pos << point_msg.x, point_msg.y, point_msg.z;
    return pos;
}

Eigen::Quaterniond QuatMsg2Quat(geometry_msgs::Quaternion quat_msg)
{
    Eigen::Quaterniond quat(quat_msg.w,quat_msg.x,quat_msg.y,quat_msg.z);
    return quat;
}

class ControllerNode
{
    public:
        ControllerNode();
        void runControllerNode(int argc, char** argv);
        void dataCallback(const arm_hand_capture::DualArmDualHandStateWithImage::ConstPtr& msg);
    private:
        ros::Publisher pub;
        ros::Subscriber dataSub;
        ros::Subscriber jointStateSub;
        double ql_data[NUM_OF_JOINTS/2] = {
                -1.5,-1.5,1.5,0.0,0.0,0.0,0.0
        };
        double qr_data[NUM_OF_JOINTS/2] = {
            1.5,-1.5,-1.5,0.0,0.0,0.0,0.0
        };
        std::vector<double> ql_vec;
        std::vector<double> qr_vec;
        Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_last;
        Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_last;
        NullSpaceControl* nullspace_control_ptr = nullptr;
        int count = 0;
        
};

ControllerNode::ControllerNode(){}

void ControllerNode::runControllerNode(int argc, char** argv)
{
    // Init node & publisher
    ros::init(argc, argv, "ControllerNode");
    ros::NodeHandle nh;
    this->pub = nh.advertise<realtime_motion_retargeting::ControlMsg>("cmdPublisher",1000);
    this->dataSub = nh.subscribe("/dual_arms_dual_hands_state_with_image",1,&ControllerNode::dataCallback,this); 
    // this->jointStateSub = nh.subscribe("/dual_arms_dual_hands_state_with_image",1000,&ControllerNode::jointStatecallback,this);
    ros::Rate loop_rate(10);

    // Init joint angle
    ql_vec.resize(NUM_OF_JOINTS/2);
    qr_vec.resize(NUM_OF_JOINTS/2);
    for (int i = 0; i < NUM_OF_JOINTS/2; i++)
    {
        ql_vec[i] = ql_data[i];
        qr_vec[i] = qr_data[i];
    }
    ql_last = matrix_helper::stdVec2Matrix(ql_vec);
    qr_last = matrix_helper::stdVec2Matrix(qr_vec);
    
    // Init nullspace control ptr
    this->nullspace_control_ptr = new NullSpaceControl();
    ros::spin();
}

void ControllerNode::dataCallback(const arm_hand_capture::DualArmDualHandStateWithImage::ConstPtr& msg)
{
    if (this->count%2==0) return;
    this->count++;
    // Process msg
    geometry_msgs::Pose l_wrist_pose = msg->left_hand_pose.pose;
    geometry_msgs::Pose r_wrist_pose = msg->right_hand_pose.pose;
    geometry_msgs::Point l_wrist_point_msg = l_wrist_pose.position;
    geometry_msgs::Point r_wrist_point_msg = l_wrist_pose.position;
    geometry_msgs::Quaternion l_wrist_quat_msg = l_wrist_pose.orientation;
    geometry_msgs::Quaternion r_wrist_quat_msg = r_wrist_pose.orientation;

    geometry_msgs::Pose l_elbow_pose = msg->left_forearm_pose.pose;
    geometry_msgs::Pose r_elbow_pose = msg->right_forearm_pose.pose;
    geometry_msgs::Point l_elbow_point_msg = l_elbow_pose.position; 
    geometry_msgs::Point r_elbow_point_msg = r_elbow_pose.position; 

    geometry_msgs::Pose l_shoulder_pose = msg->left_upperarm_pose.pose;
    geometry_msgs::Pose r_shoulder_pose = msg->right_upperarm_pose.pose;
    geometry_msgs::Point l_shoulder_point_msg = l_shoulder_pose.position; 
    geometry_msgs::Point r_shoulder_point_msg = r_shoulder_pose.position; 

    Eigen::Vector3d l_wrist_pos = PointMsg2Pos(l_wrist_point_msg);
    Eigen::Vector3d r_wrist_pos = PointMsg2Pos(r_wrist_point_msg);
    Eigen::Quaterniond l_wrist_quat = QuatMsg2Quat(l_wrist_quat_msg);
    Eigen::Quaterniond r_wrist_quat = QuatMsg2Quat(r_wrist_quat_msg);
    Eigen::Vector3d l_elbow_pos = PointMsg2Pos(l_elbow_point_msg);
    Eigen::Vector3d r_elbow_pos = PointMsg2Pos(r_elbow_point_msg);
    Eigen::Vector3d l_shoulder_pos = PointMsg2Pos(l_shoulder_point_msg);
    Eigen::Vector3d r_shoulder_pos = PointMsg2Pos(r_shoulder_point_msg);

    l_wrist_pos = l_wrist_pos - l_shoulder_pos;
    r_wrist_pos = r_wrist_pos - r_shoulder_pos;
    l_elbow_pos = l_elbow_pos - l_shoulder_pos;
    r_elbow_pos = r_elbow_pos - r_shoulder_pos;
    
    // Map to yumi workspace
    l_wrist_pos = scale * l_wrist_pos + l_offset;
    r_wrist_pos = scale * r_wrist_pos + r_offset;
    l_elbow_pos = scale * l_elbow_pos + l_offset;
    r_elbow_pos = scale * r_elbow_pos + r_offset;

    // NullSpace IK
    Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle = 
        this->nullspace_control_ptr->solve_one_step(
            l_wrist_pos,r_wrist_pos,l_wrist_quat,r_wrist_quat,l_elbow_pos,r_elbow_pos,ql_last,qr_last
        );

    // Linear mapping for glove angle
    std::vector<double> l_glove_angle = msg->glove_state.left_glove_state;
    std::vector<double> r_glove_angle = msg->glove_state.right_glove_state;
    std::vector<double> l_hand_joint_angle = convert_glove_angle_dof15(l_glove_angle);
    std::vector<double> r_hand_joint_angle = convert_glove_angle_dof15(r_glove_angle);

    // Construct & Publish controlMsg
    realtime_motion_retargeting::ControlMsg control_message;
    std::vector<double> l_arm_joint_angle = matrix_helper::Matrix2stdVec(joint_angle.block(0,0,NUM_OF_JOINTS/2,1));
    std::vector<double> r_arm_joint_angle = matrix_helper::Matrix2stdVec(joint_angle.block(NUM_OF_JOINTS/2,0,NUM_OF_JOINTS/2,1));
    control_message.l_arm_joint_angle = l_arm_joint_angle;
    control_message.r_arm_joint_angle = r_arm_joint_angle;
    control_message.l_hand_joint_angle = l_hand_joint_angle;
    control_message.r_hand_joint_angle = r_hand_joint_angle;
    this->pub.publish(control_message);

    // Save the result at last time
    ql_last = joint_angle.block(0,0,NUM_OF_JOINTS/2,1);
    qr_last = joint_angle.block(NUM_OF_JOINTS/2,0,NUM_OF_JOINTS/2,1);

    std::cout<<"---------------------------------------"<<std::endl;
    std::cout<<"Left arm joint angle: " << joint_angle.block(0,0,NUM_OF_JOINTS/2,1).transpose() << std::endl;
    std::cout<<"Right arm joint angle: " << joint_angle.block(NUM_OF_JOINTS/2,0,NUM_OF_JOINTS/2,1).transpose() << std::endl;
    std::cout<<"Left hand joint angle: " ;
    for (int i = 0; i < l_hand_joint_angle.size(); i++)
    {
        std::cout << l_hand_joint_angle[i] << " ";
    }
    std::cout<<std::endl;
    
    std::cout<<"Right hand joint angle: " ;
    for (int i = 0; i < r_hand_joint_angle.size(); i++)
    {
        std::cout << r_hand_joint_angle[i] << " ";
    }
    std::cout<<std::endl;
    
}

int main(int argc, char** argv)
{
    ControllerNode node;
    node.runControllerNode(argc, argv);
    return 0;
}