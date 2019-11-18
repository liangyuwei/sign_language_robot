#include <ros/ros.h>

#include <hand_control.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>


#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

using namespace std;


int hand_id_;
std::string port_name_;
int baudrate_;
int test_flags;
serial::Serial *com_port_;
uint8_t hand_state_;
void setANGLE1(serial::Serial *port, int angle0, int angle1, int angle2, int angle3, int angle4, int angle5) 		
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x0F);
    //Command get state
    output.push_back(0x12);
    output.push_back(0xCE);
    output.push_back(0x05);

    int temp_int1,temp_int2,temp_int3,temp_int4,temp_int5,temp_int6;
    temp_int1 = angle0;
    temp_int2 = angle1;
    temp_int3 = angle2;
    temp_int4 = angle3;
    temp_int5 = angle4;
    temp_int6 = angle5;

    output.push_back(temp_int1 & 0xff);
    output.push_back((temp_int1 >> 8) & 0xff);
    output.push_back(temp_int2 & 0xff);
    output.push_back((temp_int2 >> 8) & 0xff);
    output.push_back(temp_int3 & 0xff);
    output.push_back((temp_int3 >> 8) & 0xff);
    output.push_back(temp_int4 & 0xff);
    output.push_back((temp_int4 >> 8) & 0xff);
    output.push_back(temp_int5 & 0xff);
    output.push_back((temp_int5 >> 8) & 0xff);
    output.push_back(temp_int6 & 0xff);
    output.push_back((temp_int6 >> 8) & 0xff);
    //Checksum calculation

    unsigned int check_num = 0;
    int len = output[3] + 5;

    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);

    //Send message to the module
    port->write(output);

    ros::Duration(0.015).sleep();

    std::string s1;

    for (int i = 0; i<output.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", output[i]);
        s1 = s1 + str + " ";
    }
    if (test_flags == 1)
        ROS_INFO_STREAM("Write: " << s1);

    std::vector<uint8_t> input;

    while (input.empty())
    {
        port->read(input, (size_t)64);
    }

    std::string s2;
    for (int i = 0; i<input.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", input[i]);
        s2 = s2 + str + " ";
    }
    if (test_flags == 1)
        ROS_INFO_STREAM("Read: " << s2);
}
int Arr[12];

void arrayCallback1(const std_msgs::Int32MultiArray::ConstPtr& array)
{

    int i = 0;
    // print all the remaining numbers
    for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {

        Arr[i] = *it;
        printf("%d, ", Arr[i]);
        i++;
    }
    setANGLE1(com_port_,Arr[0],Arr[1],Arr[2],Arr[3],Arr[4],Arr[5]);

    return;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "handcontroltopicpublisher1");
    ros::NodeHandle nh;

    //topic
    ros::Subscriber sub = nh.subscribe("chatter1", 1000, arrayCallback1);

    ros::Rate loop_rate(10);

    //Read launch file params
    nh.getParam("inspire_hand/hand_id", hand_id_);
    nh.getParam("inspire_hand/portname", port_name_);
    nh.getParam("inspire_hand/baudrate", baudrate_);
    nh.getParam("inspire_hand/test_flags", test_flags);
    //Initialize and open serial port
    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));

    ros::spin();

    return(EXIT_SUCCESS);
}
