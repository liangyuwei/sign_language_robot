// ROS
#include <ros/ros.h>
// Messages
#include "realtime_motion_retargeting/MotionMsg.h"
// Basic
#include <string>
// For opt parse
#include <getopt.h>
// For h5_io
#include "h5_io.h"
// My header files
#include "transform_matrix_helper.h"

using namespace std;

class DataSamplerNode
{
    public:
       DataSamplerNode(); 
       void runDataSamplerNode(int argc, char** argv, string in_file_name, string in_group_name, double interval);
    private:
        ros::Publisher pub;
        double publish_inteval;
};

DataSamplerNode::DataSamplerNode() {}

void DataSamplerNode::runDataSamplerNode(
    int argc, char** argv, string in_file_name, string in_group_name, double interval) 
{
    // Init node
    ros::init(argc, argv, "DataSamplerNode");
    ros::NodeHandle nh;

    // Init publisher
    this->pub = nh.advertise<realtime_motion_retargeting::MotionMsg>("dataPublisher",1000);
    this->publish_inteval = interval;

    // Temporalily read data from h5 file
    std::vector<std::vector<double>> l_shoulder_pos_traj = read_h5(in_file_name, in_group_name, "l_shoulder_pos");
    std::vector<std::vector<double>> r_shoulder_pos_traj = read_h5(in_file_name, in_group_name, "r_shoulder_pos");
    std::vector<std::vector<double>> l_wrist_pos_traj = read_h5(in_file_name, in_group_name, "l_wrist_pos");
    std::vector<std::vector<double>> r_wrist_pos_traj = read_h5(in_file_name, in_group_name, "r_wrist_pos");
    std::vector<std::vector<double>> l_wrist_quat_traj = read_h5(in_file_name, in_group_name, "l_wrist_quat_resampled");
    std::vector<std::vector<double>> r_wrist_quat_traj = read_h5(in_file_name, in_group_name, "r_wrist_quat_resampled");
    std::vector<std::vector<double>> l_elbow_pos_traj = read_h5(in_file_name, in_group_name, "l_elbow_pos");
    std::vector<std::vector<double>> r_elbow_pos_traj = read_h5(in_file_name, in_group_name, "r_elbow_pos");
    std::vector<std::vector<double>>  l_glove_angle_traj = read_h5(in_file_name,in_group_name,"l_glove_angle_resampled");
    std::vector<std::vector<double>>  r_glove_angle_traj = read_h5(in_file_name,in_group_name,"r_glove_angle_resampled");

    // Publish Motion Message in a For-Loop
    int nbInteval = 100;
    for (int i = 0; i < cfg::NUM_DATAPOINTS-1; i++)
    {
        std::vector<double> l_shoulder_pos_1 = l_shoulder_pos_traj[i];
        std::vector<double> r_shoulder_pos_1 = r_shoulder_pos_traj[i];
        // std::vector<double> center(3);
        // center[0] = (l_shoulder_pos[0] + r_shoulder_pos_traj[0])/2.0;
        // center[1] = (l_shoulder_pos[1] + r_shoulder_pos_traj[1])/2.0;
        // center[2] = (l_shoulder_pos[2] + r_shoulder_pos_traj[2])/2.0;
        std::vector<double> l_wrist_pos_1 = l_wrist_pos_traj[i];
        std::vector<double> r_wrist_pos_1 = r_wrist_pos_traj[i];
        l_wrist_pos_1 = matrix_helper::vectorMinus(l_wrist_pos_1,l_shoulder_pos_1);
        r_wrist_pos_1 = matrix_helper::vectorMinus(r_wrist_pos_1,r_shoulder_pos_1);
        std::vector<double> l_elbow_pos_1 = l_elbow_pos_traj[i];
        std::vector<double> r_elbow_pos_1 = r_elbow_pos_traj[i];
        l_elbow_pos_1 = matrix_helper::vectorMinus(l_elbow_pos_1,l_shoulder_pos_1);
        r_elbow_pos_1 = matrix_helper::vectorMinus(r_elbow_pos_1,r_shoulder_pos_1);
        std::vector<double> l_wrist_quat_1 = l_wrist_quat_traj[i];
        std::vector<double> r_wrist_quat_1 = r_wrist_quat_traj[i];
        std::vector<double> l_glove_angle_1 = l_glove_angle_traj[i];
        std::vector<double> r_glove_angle_1 = r_glove_angle_traj[i];

        std::vector<double> l_shoulder_pos_2 = l_shoulder_pos_traj[i+1];
        std::vector<double> r_shoulder_pos_2 = r_shoulder_pos_traj[i+1];
        std::vector<double> l_wrist_pos_2 = l_wrist_pos_traj[i+1];
        std::vector<double> r_wrist_pos_2 = r_wrist_pos_traj[i+1];
        l_wrist_pos_2 = matrix_helper::vectorMinus(l_wrist_pos_2,l_shoulder_pos_2);
        r_wrist_pos_2 = matrix_helper::vectorMinus(r_wrist_pos_2,r_shoulder_pos_2);
        std::vector<double> l_elbow_pos_2 = l_elbow_pos_traj[i+1];
        std::vector<double> r_elbow_pos_2 = r_elbow_pos_traj[i+1];
        l_elbow_pos_2 = matrix_helper::vectorMinus(l_elbow_pos_2,l_shoulder_pos_2);
        r_elbow_pos_2 = matrix_helper::vectorMinus(r_elbow_pos_2,r_shoulder_pos_2);
        std::vector<double> l_wrist_quat_2 = l_wrist_quat_traj[i+1];
        std::vector<double> r_wrist_quat_2 = r_wrist_quat_traj[i+1];
        std::vector<double> l_glove_angle_2 = l_glove_angle_traj[i+1];
        std::vector<double> r_glove_angle_2 = r_glove_angle_traj[i+1];

        for (int j = 0; j < nbInterval; j++)
        {
            double t = double(j) / double(nbInterval);
            std::vector<double> l_wrist_pos =  matrix_helper::interpolate_between_stdvec(l_wrist_pos_1,l_wrist_pos_2,t);
            std::vector<double> r_wrist_pos =  matrix_helper::interpolate_between_stdvec(r_wrist_pos_1,r_wrist_pos_2,t);
            std::vector<double> l_wrist_quat =  matrix_helper::interpolate_between_stdvec(l_wrist_quat_1,l_wrist_quat_2,t);
            std::vector<double> r_wrist_quat =  matrix_helper::interpolate_between_stdvec(r_wrist_quat_1,r_wrist_quat_2,t);
            std::vector<double> l_elbow_pos =  matrix_helper::interpolate_between_stdvec(l_elbow_pos_1,l_elbow_pos_2,t);
            std::vector<double> r_elbow_pos =  matrix_helper::interpolate_between_stdvec(r_elbow_pos_1,r_elbow_pos_2,t);
            std::vector<double> l_glove_angle =  matrix_helper::interpolate_between_stdvec(l_glove_angle_1,l_glove_angle_2,t);
            std::vector<double> r_glove_angle =  matrix_helper::interpolate_between_stdvec(r_glove_angle_1,r_glove_angle_2,t);
            realtime_motion_retargeting::MotionMsg motion_message;
            motion_message.l_wrist_pos = l_wrist_pos;
            motion_message.r_wrist_pos = r_wrist_pos;
            motion_message.l_wrist_quat = l_wrist_quat;
            motion_message.r_wrist_quat = r_wrist_quat;
            motion_message.l_elbow_pos = l_elbow_pos;
            motion_message.r_elbow_pos = r_elbow_pos;
            motion_message.l_glove_angle = l_glove_angle;
            motion_message.r_glove_angle = r_glove_angle;

            this->pub.publish(motion_message);

            ros::Duration duration(this->publish_inteval);
            duration.sleep();           
        }
        

    }
    
}

int main(int argc, char** argv)
{
    // Process the terminal arguments & Read the path settings
    string in_file_name = "/home/liweijie/datasets/motion-retargeting-datasets/test_imi_data_YuMi.h5";
    string in_group_name = "fengren_1";
    double publish_interval = 0.01;
    static struct option long_options[] = 
    {
        {"in-h5-filename",             optional_argument, NULL, 'i'},
        {"in-group-name",              optional_argument, NULL, 'g'},
        {"publish-interval",           optional_argument, NULL, 't'},
        {0,                            0,    0,   0}
    };
    int c;
    while (1)
    {
        int opt_index = 0;
        // Get arguments
        c = getopt_long(argc, argv, "i:g:t:h", long_options, &opt_index);
        if (c == -1)
        break;

        // Process
        switch(c)
        {
            case 'h':
                std::cout << "Help: \n" << std::endl;
                std::cout << "Arguments:\n" << std::endl;
                std::cout << "    -i, --in-h5-filename, specify the name of the input h5 file, otherwise a default name specified inside the program will be used. Suffix is required.\n" << std::endl;
                std::cout << "    -g, --in-group-name, specify the group name in the h5 file, which is actually the motion's name.\n" << std::endl;
                std::cout << "    -t, --publish-interval, specify the interval between two topics\n" << std::endl;
                return 0;
                break;

            case 'i':
                in_file_name = optarg;
                break;

            case 'g':
                in_group_name = optarg;
                break;

            case 't':
                publish_interval = std::atof(optarg);
                break;

            default:
                break;
        }
    }

    // Start the Data Sampler Node
    DataSamplerNode node;
    node.runDataSamplerNode(argc,argv,in_file_name,in_group_name,publish_interval);
    return 0;
}