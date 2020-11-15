#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_mapping_params/MappingParamsConfig.h>
#include <dynamic_mapping_params/MappingParams.h>

class DynamicReconfigureNode
{
    public:
        void runDynamicReconfigureNode(ros::NodeHandle nh);
        void callback(dynamic_mapping_params::MappingParamsConfig &config, uint32_t level) ;
    private:
        ros::Publisher params_pub;
};

void DynamicReconfigureNode::runDynamicReconfigureNode(ros::NodeHandle nh)
{
    this->params_pub = nh.advertise<dynamic_mapping_params::MappingParams>("current_mapping_params",1000);
}

void DynamicReconfigureNode::callback(dynamic_mapping_params::MappingParamsConfig &config, uint32_t level) 
{
    // ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f", 
    //        config.l_scale, 
    //        config.l_offs_x, 
    //        config.l_offs_y, 
    //        config.l_offs_z,
    //        config.r_scale,
    //        config.r_offs_x,
    //        config.r_offs_y,
    //        config.r_offs_z);
    dynamic_mapping_params::MappingParams msg;
    msg.l_elbow_scale = config.l_elbow_scale;
    msg.l_elbow_offs_x = config.l_elbow_offs_x;
    msg.l_elbow_offs_y = config.l_elbow_offs_y;
    msg.l_elbow_offs_z = config.l_elbow_offs_z;

    msg.r_elbow_scale = config.r_elbow_scale;
    msg.r_elbow_offs_x = config.r_elbow_offs_x;
    msg.r_elbow_offs_y = config.r_elbow_offs_y;
    msg.r_elbow_offs_z = config.r_elbow_offs_z;

    msg.l_wrist_scale = config.l_wrist_scale;
    msg.l_wrist_offs_x = config.l_wrist_offs_x;
    msg.l_wrist_offs_y = config.l_wrist_offs_y;
    msg.l_wrist_offs_z = config.l_wrist_offs_z;

    msg.r_wrist_scale = config.r_wrist_scale;
    msg.r_wrist_offs_x = config.r_wrist_offs_x;
    msg.r_wrist_offs_y = config.r_wrist_offs_y;
    msg.r_wrist_offs_z = config.r_wrist_offs_z;

    this->params_pub.publish(msg);
}

int main(int argc, char **argv) 
{
    DynamicReconfigureNode reconfigure_node;

    ros::init(argc, argv, "dynamic_reconfigure_node");
    ros::NodeHandle nh;
    reconfigure_node.runDynamicReconfigureNode(nh);

    dynamic_reconfigure::Server<dynamic_mapping_params::MappingParamsConfig> server;
    dynamic_reconfigure::Server<dynamic_mapping_params::MappingParamsConfig>::CallbackType f;
    boost::function<void (dynamic_mapping_params::MappingParamsConfig &,int) > f2( boost::bind( &DynamicReconfigureNode::callback,&reconfigure_node, _1, _2 ) );
    f = f2;
    server.setCallback(f);

    ROS_INFO("[ReconfigureNode] Spinning node");
    ros::spin();
   
   return 0;
}