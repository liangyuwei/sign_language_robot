// ROS
#include <ros/ros.h>
// Messages
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "realtime_motion_retargeting/ControlMsg.h"
// #include "arm_hand_capture/DualArmDualHandStateWithImage.h"
#include "arm_hand_capture/DualArmDualHandState.h"
#include "dynamic_mapping_params/MappingParams.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

// For moveit
#include <moveit/move_group_interface/move_group_interface.h>
// For Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// Basic
#include <chrono>
#include <map>
// My header files
#include "config.h"
#include "NullSpaceControl.h"
#include "util_functions.h"
#include "kinematics.h"
#include "yumi_trac_ik_solver.h"

using namespace cfg;

static double l_elbow_scale = 1.0;
static double l_elbow_offs[3] = {0.0, 0.2, 0.2};
static Eigen::Vector3d l_elbow_offset = 
    matrix_helper::create_translation_vector(l_elbow_offs[0],l_elbow_offs[1],l_elbow_offs[2]);

static double r_elbow_scale = 1.0;
static double r_elbow_offs[3] = {0.0, -0.2, 0.2};
static Eigen::Vector3d r_elbow_offset = 
    matrix_helper::create_translation_vector(r_elbow_offs[0],r_elbow_offs[1],r_elbow_offs[2]);

static double l_wrist_scale = 1.0;
static double l_wrist_offs[3] = {0.0, 0.2, 0.2};
static Eigen::Vector3d l_wrist_offset = 
    matrix_helper::create_translation_vector(l_wrist_offs[0],l_wrist_offs[1],l_wrist_offs[2]);

static double r_wrist_scale = 1.0;
static double r_wrist_offs[3] = {0.0, -0.2, 0.2};
static Eigen::Vector3d r_wrist_offset = 
    matrix_helper::create_translation_vector(r_wrist_offs[0],r_wrist_offs[1],r_wrist_offs[2]);

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
        // void dataCallback(const arm_hand_capture::DualArmDualHandStateWithImage::ConstPtr& msg);
        void dataCallback(const arm_hand_capture::DualArmDualHandState::ConstPtr& msg);
        void paramCallback(const dynamic_mapping_params::MappingParams::ConstPtr& msg);
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    private:
        ros::Publisher pub;
        ros::Publisher jointStatePub;
        ros::Subscriber dataSub;
        ros::Subscriber jointStateSub;
        ros::Subscriber mappingParamSub;
        double ql_data[NUM_OF_JOINTS/2] = {
                -1.5,-1.5,1.5,0.0,0.0,0.0,0.0
        };
        double qr_data[NUM_OF_JOINTS/2] = {
            1.5,-1.5,-1.5,0.0,0.0,0.0,0.0
        };
        std::vector<double> ql_vec;
        std::vector<double> qr_vec;
        Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_initial;
        Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_initial;
        Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_last;
        Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_last;
        Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_current;
        Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_current;
        NullSpaceControl* nullspace_control_ptr = nullptr;
        Eigen::Matrix3d rotm_left_arm;
        Eigen::Matrix3d rotm_right_arm;
        Eigen::Quaterniond TransformToRobotFrame(Eigen::Quaterniond quat, bool left_or_right);
        yumi_trac_ik_solver trac_ik_solver;
        bool offset_init_flag = false;

        int count = 0;
        
};

ControllerNode::ControllerNode(){
    rotm_left_arm << 0.0, -1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, 1.0; // from manual calculation...
    rotm_right_arm << 0.0, -1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, 1.0; // from manual calculation...
}

Eigen::Quaterniond ControllerNode::TransformToRobotFrame(Eigen::Quaterniond quat, bool left_or_right=true)
{
    Eigen::Quaterniond quatRobot;
    if(left_or_right) {
        quatRobot = quat * this->rotm_left_arm;
    }
    else {
        quatRobot = quat * this->rotm_right_arm;
    }
    return quatRobot;
}

void ControllerNode::runControllerNode(int argc, char** argv)
{
    // Init node & publisher
    ros::init(argc, argv, "ControllerNode");
    ros::NodeHandle nh;
    this->pub = nh.advertise<realtime_motion_retargeting::ControlMsg>("cmdPublisher",1000);
    this->jointStatePub = nh.advertise<std_msgs::Float64MultiArray>("current_joint_states",1000);
    this->dataSub = nh.subscribe("/dual_arms_dual_hands_state",1000,&ControllerNode::dataCallback,this); 
    this->mappingParamSub = nh.subscribe("current_mapping_params",1000,&ControllerNode::paramCallback,this);
    this->jointStateSub = nh.subscribe("/yumi/joint_states",1000,&ControllerNode::jointStateCallback,this);
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
    ql_initial = matrix_helper::stdVec2Matrix(ql_vec);
    qr_initial = matrix_helper::stdVec2Matrix(qr_vec);
    ql_current = matrix_helper::stdVec2Matrix(ql_vec);
    qr_current = matrix_helper::stdVec2Matrix(qr_vec);
    
    // Init nullspace control ptr
    this->nullspace_control_ptr = new NullSpaceControl();
    ros::spin();

    // Init yumi trac ik solver
    trac_ik_solver = yumi_trac_ik_solver();
}

void ControllerNode::paramCallback(const dynamic_mapping_params::MappingParams::ConstPtr& msg)
{
    std::cout<<"[ControllerNode] The mapping params has changed"<<std::endl;
    ROS_WARN("The mapping params has changed");
    l_elbow_scale = msg->l_elbow_scale;
    l_elbow_offset[0] = msg->l_elbow_offs_x;
    l_elbow_offset[1] = msg->l_elbow_offs_y;
    l_elbow_offset[2] = msg->l_elbow_offs_z;

    r_elbow_scale = msg->r_elbow_scale;
    r_elbow_offset[0] = msg->r_elbow_offs_x;
    r_elbow_offset[1] = msg->r_elbow_offs_y;
    r_elbow_offset[2] = msg->r_elbow_offs_z;
    
    l_wrist_scale = msg->l_wrist_scale;
    l_wrist_offset[0] = msg->l_wrist_offs_x;
    l_wrist_offset[1] = msg->l_wrist_offs_y;
    l_wrist_offset[2] = msg->l_wrist_offs_z;

    r_wrist_scale = msg->r_wrist_scale;
    r_wrist_offset[0] = msg->r_wrist_offs_x;
    r_wrist_offset[1] = msg->r_wrist_offs_y;
    r_wrist_offset[2] = msg->r_wrist_offs_z;
}

void ControllerNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // std::cout<<"[ControllerNode] The joint state is received"<<std::endl;
    std::vector<string> n = msg->name;
    std::vector<double> p = msg->position;
    std::map<string,double> m;
    for (int i = 0; i < n.size(); i++)
    {
        // std::cout<<n[i]<<" : " << p[i] <<std::endl;
        m[n[i]] = p[i];
    }
    double l_joint1_state = m.at("yumi_joint_1_l"); 
    double l_joint2_state = m.at("yumi_joint_2_l"); 
    double l_joint3_state = m.at("yumi_joint_7_l"); 
    double l_joint4_state = m.at("yumi_joint_3_l"); 
    double l_joint5_state = m.at("yumi_joint_4_l"); 
    double l_joint6_state = m.at("yumi_joint_5_l"); 
    double l_joint7_state = m.at("yumi_joint_6_l"); 

    double r_joint1_state = m.at("yumi_joint_1_r"); 
    double r_joint2_state = m.at("yumi_joint_2_r"); 
    double r_joint3_state = m.at("yumi_joint_7_r"); 
    double r_joint4_state = m.at("yumi_joint_3_r"); 
    double r_joint5_state = m.at("yumi_joint_4_r"); 
    double r_joint6_state = m.at("yumi_joint_5_r"); 
    double r_joint7_state = m.at("yumi_joint_6_r");
    
    std::vector<double> l_arm_p;
    std::vector<double> r_arm_p;

    l_arm_p.push_back(l_joint1_state);
    l_arm_p.push_back(l_joint2_state);
    l_arm_p.push_back(l_joint3_state);
    l_arm_p.push_back(l_joint4_state);
    l_arm_p.push_back(l_joint5_state);
    l_arm_p.push_back(l_joint6_state);
    l_arm_p.push_back(l_joint7_state);

    r_arm_p.push_back(r_joint1_state);
    r_arm_p.push_back(r_joint2_state);
    r_arm_p.push_back(r_joint3_state);
    r_arm_p.push_back(r_joint4_state);
    r_arm_p.push_back(r_joint5_state);
    r_arm_p.push_back(r_joint6_state);
    r_arm_p.push_back(r_joint7_state);
    // std::cout<<"[ControllerNode]  The length of l_arm_p is: " << l_arm_p.size() <<std::endl;
    // std::cout<<"[ControllerNode]  The length of r_arm_p is: " << r_arm_p.size() <<std::endl;

    for (int i = 0; i < NUM_OF_JOINTS/2; i++)
    {
        ql_vec[i] = l_arm_p[i];
        qr_vec[i] = r_arm_p[i];
    }
    ql_current = matrix_helper::stdVec2Matrix(ql_vec);
    qr_current = matrix_helper::stdVec2Matrix(qr_vec);

    // std_msgs::Float64MultiArray joint_state_message;
    // joint_state_message.data = ql_vec;
    // this->jointStatePub.publish(joint_state_message);

    
}

// void ControllerNode::dataCallback(const arm_hand_capture::DualArmDualHandStateWithImage::ConstPtr& msg)
void ControllerNode::dataCallback(const arm_hand_capture::DualArmDualHandState::ConstPtr& msg)
{
    std::cout<<"---------------------------------------"<<std::endl;
    // if (this->count%2==0) return;
    // this->count++;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    // Process msg
    geometry_msgs::Pose l_wrist_pose = msg->left_hand_pose.pose;
    geometry_msgs::Pose r_wrist_pose = msg->right_hand_pose.pose;
    geometry_msgs::Point l_wrist_point_msg = l_wrist_pose.position;
    geometry_msgs::Point r_wrist_point_msg = r_wrist_pose.position;
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
    Eigen::Quaterniond l_wrist_quat = TransformToRobotFrame(QuatMsg2Quat(l_wrist_quat_msg),true);
    Eigen::Quaterniond r_wrist_quat = TransformToRobotFrame(QuatMsg2Quat(r_wrist_quat_msg),true);
    Eigen::Vector3d l_elbow_pos = PointMsg2Pos(l_elbow_point_msg);
    Eigen::Vector3d r_elbow_pos = PointMsg2Pos(r_elbow_point_msg);
    Eigen::Vector3d l_shoulder_pos = PointMsg2Pos(l_shoulder_point_msg);
    Eigen::Vector3d r_shoulder_pos = PointMsg2Pos(r_shoulder_point_msg);

    Eigen::Vector3d l_wrist_pos_rel = l_wrist_pos - l_shoulder_pos;
    Eigen::Vector3d r_wrist_pos_rel = r_wrist_pos - r_shoulder_pos;
    Eigen::Vector3d l_elbow_pos_rel = l_elbow_pos - l_shoulder_pos;
    Eigen::Vector3d r_elbow_pos_rel = r_elbow_pos - r_shoulder_pos;
    
    // Init offsets
    if(!this->offset_init_flag) {
        Eigen::Matrix<double,14,1> q_intial = concat_joint_angles(ql_initial,qr_initial);
        kinematics::Result res = kinematics::yumi_forward_kinematics(q_intial);
        Eigen::Vector3d l_elbow_pos_robot = res.l_elbow_pos;
        Eigen::Vector3d r_elbow_pos_robot = res.r_elbow_pos;
        Eigen::Vector3d l_wrist_pos_robot = res.l_wrist_pos;
        Eigen::Vector3d r_wrist_pos_robot = res.r_wrist_pos;

        l_elbow_offset = l_elbow_pos_robot - l_elbow_scale * l_elbow_pos_rel;
        r_elbow_offset = r_elbow_pos_robot - r_elbow_scale * r_elbow_pos_rel;
        l_wrist_offset = l_wrist_pos_robot - l_wrist_scale * l_wrist_pos_rel;
        r_wrist_offset = r_wrist_pos_robot - r_wrist_scale * r_wrist_pos_rel;

        this->offset_init_flag = true;
    }

    std::cout<< "[ControllerNode] l elbow offset: " << l_elbow_offset.transpose()<<std::endl;
    std::cout<< "[ControllerNode] r elbow offset: " << r_elbow_offset.transpose()<<std::endl;
    std::cout<< "[ControllerNode] l wrist offset: " << l_wrist_offset.transpose()<<std::endl;
    std::cout<< "[ControllerNode] r wrist offset: " << r_wrist_offset.transpose()<<std::endl;
    std::cout<<std::endl;
    
    // Map to yumi workspace
    l_elbow_pos = l_elbow_scale * l_elbow_pos_rel + l_elbow_offset;
    r_elbow_pos = r_elbow_scale * r_elbow_pos_rel + r_elbow_offset;
    l_wrist_pos = l_wrist_scale * l_wrist_pos_rel + l_wrist_offset;
    r_wrist_pos = r_wrist_scale * r_wrist_pos_rel + r_wrist_offset;

    // // NullSpace IK
    // Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle = 
    //     this->nullspace_control_ptr->solve_one_step(
    //         l_wrist_pos,r_wrist_pos,l_wrist_quat,r_wrist_quat,l_elbow_pos,r_elbow_pos,ql_last,qr_last
    //     );

    // Try trac-ik
    Eigen::Matrix<double,NUM_OF_JOINTS/2,1> ql_result;
    Eigen::Matrix<double,NUM_OF_JOINTS/2,1> qr_result;
    double timeout = 0.002;
    Eigen::Matrix3d l_wrist_rot = l_wrist_quat.toRotationMatrix();
    Eigen::Matrix3d r_wrist_rot = r_wrist_quat.toRotationMatrix();
    trac_ik_solver.run_trac_ik_left(ql_last,ql_result,l_wrist_pos,l_wrist_rot,
                    cfg::q_l_arm_lb_mat,cfg::q_l_arm_ub_mat,timeout);
    trac_ik_solver.run_trac_ik_right(qr_last,qr_result,r_wrist_pos,r_wrist_rot,
                    cfg::q_r_arm_lb_mat,cfg::q_r_arm_ub_mat,timeout);
    Eigen::Matrix<double,NUM_OF_JOINTS,1> joint_angle = concat_joint_angles(ql_result,qr_result);

    // Linear mapping for glove angle
    std::vector<double> l_glove_angle = msg->glove_state.left_glove_state;
    std::vector<double> r_glove_angle = msg->glove_state.right_glove_state;
    std::vector<double> l_hand_joint_angle = convert_glove_angle_dof15(l_glove_angle);
    std::vector<double> r_hand_joint_angle = convert_glove_angle_dof15(r_glove_angle);

    Eigen::Matrix<double,NUM_OF_JOINTS/2,1> l_joint_angle = joint_angle.block(0,0,NUM_OF_JOINTS/2,1);
    Eigen::Matrix<double,NUM_OF_JOINTS/2,1> r_joint_angle = joint_angle.block(NUM_OF_JOINTS/2,0,NUM_OF_JOINTS/2,1);

    // Eigen::Matrix<double,NUM_OF_JOINTS/2,1> dql = l_joint_angle - ql_current;
    // Eigen::Matrix<double,NUM_OF_JOINTS/2,1> dqr = r_joint_angle - qr_current;
    // matrix_helper::clamp_to_joint_limits(dql,cfg::yumi_velocity_lb,cfg::yumi_velocity_ub);
    // matrix_helper::clamp_to_joint_limits(dqr,cfg::yumi_velocity_lb,cfg::yumi_velocity_ub);
    // std::cout<< "[ControllerNode] dql: "<<dql.transpose()<<std::endl;
    // std::cout<< "[ControllerNode] dqr: "<<dqr.transpose()<<std::endl;
    // l_joint_angle = ql_current + dql;
    // r_joint_angle = qr_current + dqr;

    // l_joint_angle = 0.9 * ql_last + 0.1 * l_joint_angle;
    // r_joint_angle = 0.9 * qr_last + 0.1 * r_joint_angle;

    std::vector<double> l_arm_joint_angle = matrix_helper::Matrix2stdVec(l_joint_angle);
    std::vector<double> r_arm_joint_angle = matrix_helper::Matrix2stdVec(r_joint_angle);

    // Check the correctness of result
    kinematics::Result res = kinematics::yumi_forward_kinematics(joint_angle);
    Eigen::Vector3d l_wrist_pos_fk = res.l_wrist_pos;
    Eigen::Vector3d r_wrist_pos_fk = res.r_wrist_pos;
    Eigen::Quaterniond l_wrist_quat_fk = res.l_wrist_quat;
    Eigen::Quaterniond r_wrist_quat_fk = res.r_wrist_quat;
    Eigen::Vector3d l_elbow_pos_fk = res.l_elbow_pos;
    Eigen::Vector3d r_elbow_pos_fk = res.r_elbow_pos;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    std::cout<<"[ControllerNode] l_wrist_pos_rel: " << l_wrist_pos_rel.transpose() << std::endl;
    std::cout<<"[ControllerNode] r_wrist_pos_rel: " << r_wrist_pos_rel.transpose() << std::endl;
    std::cout<<"[ControllerNode] l_elbow_pos_rel: " << l_elbow_pos_rel.transpose() << std::endl;
    std::cout<<"[ControllerNode] r_elbow_pos_rel: " << r_elbow_pos_rel.transpose() << std::endl;
    std::cout<<std::endl;

    std::cout<<"[ControllerNode] l_wrist_pos: " << l_wrist_pos.transpose() << std::endl;
    std::cout<<"[ControllerNode] r_wrist_pos: " << r_wrist_pos.transpose() << std::endl;
    std::cout<<"[ControllerNode] l_wrist_quat: " << 
    l_wrist_quat.w() << " " << l_wrist_quat.x() << " " << l_wrist_quat.y() << " " << l_wrist_quat.z() << std::endl;
    std::cout<<"[ControllerNode] r_wrist_quat: " << 
    r_wrist_quat.w() << " " << r_wrist_quat.x() << " " << r_wrist_quat.y() << " " << r_wrist_quat.z() << std::endl;
    std::cout<<"[ControllerNode] l_elbow_pos: " << l_elbow_pos.transpose() << std::endl;
    std::cout<<"[ControllerNode] r_elbow_pos: " << r_elbow_pos.transpose() << std::endl;
    std::cout<<std::endl;

    std::cout<<"[ControllerNode] l_wrist_pos_fk: " << l_wrist_pos_fk.transpose() << std::endl;
    std::cout<<"[ControllerNode] r_wrist_pos_fk: " << r_wrist_pos_fk.transpose() << std::endl;
    std::cout<<"[ControllerNode] l_wrist_quat_fk: " << 
    l_wrist_quat_fk.w() << " " << l_wrist_quat_fk.x() << " " << l_wrist_quat_fk.y() << " " << l_wrist_quat_fk.z() << std::endl;
    std::cout<<"[ControllerNode] r_wrist_quat_fk: " << 
    r_wrist_quat_fk.w() << " " << r_wrist_quat_fk.x() << " " << r_wrist_quat_fk.y() << " " << r_wrist_quat_fk.z() << std::endl;
    std::cout<<"[ControllerNode] l_elbow_pos_fk: " << l_elbow_pos_fk.transpose() << std::endl;
    std::cout<<"[ControllerNode] r_elbow_pos_fk: " << r_elbow_pos_fk.transpose() << std::endl;
    std::cout<<std::endl;

    std::cout<<"[ControllerNode] Left arm joint angle: " << l_joint_angle.transpose() << std::endl;
    std::cout<<"[ControllerNode] Right arm joint angle: " << r_joint_angle.transpose() << std::endl;
    std::cout<<"[ControllerNode] Current left joint angle: " << ql_current.transpose() << std::endl;
    std::cout<<"[ControllerNode] Current right joint angle: " << qr_current.transpose() << std::endl;
    // std::cout<<"[ControllerNode] Left hand joint angle: " ;
    // for (int i = 0; i < l_hand_joint_angle.size(); i++)
    // {
    //     std::cout << l_hand_joint_angle[i] << " ";
    // }
    // std::cout<<std::endl;
    
    // std::cout<<"[ControllerNode] Right hand joint angle: " ;
    // for (int i = 0; i < r_hand_joint_angle.size(); i++)
    // {
    //     std::cout << r_hand_joint_angle[i] << " ";
    // }
    // std::cout<<std::endl;
    std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    std::cout<<"[ControllerNode] Inverse Kinematics cost "<< dt.count() << "s" << std::endl;
    
    // Construct & Publish controlMsg
    realtime_motion_retargeting::ControlMsg control_message;
    control_message.l_arm_joint_angle = l_arm_joint_angle;
    control_message.r_arm_joint_angle = r_arm_joint_angle;
    control_message.l_hand_joint_angle = l_hand_joint_angle;
    control_message.r_hand_joint_angle = r_hand_joint_angle;
    this->pub.publish(control_message);

    // Save the result at last time
    ql_last = l_joint_angle;
    qr_last = r_joint_angle;

}

int main(int argc, char** argv)
{
    // Set K_NULLSPACE_ELBOW
    double k;
    ros::param::get("k_nullspace_elbow",k);
    cfg::K_NULLSPACE_ELBOW = k;

    ControllerNode node;
    node.runControllerNode(argc, argv);
    return 0;
}