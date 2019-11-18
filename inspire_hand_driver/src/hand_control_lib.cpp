#ifndef HAND_CONTROL_LIB_CPP
#define HAND_CONTROL_LIB_CPP

#include <hand_control.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>

//#include <std_msgs/String.h>
/*
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

int Arr[12];

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{

        int i = 0;
        // print all the remaining numbers
        for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
        {
                if (i==0)
                        printf("act_ang:");
                if (i==6)
                        printf("\nact_force:");

                Arr[i] = *it;
                printf("%d, ", Arr[i]);
                if (i==11)
                        printf("\n");
                i++;
        }

        return;
}

*/
namespace inspire_hand
{

hand_serial::hand_serial(ros::NodeHandle *nh):
    act_position_(-1),
    hand_state_(0xff)
{
    //Read launch file params
    nh->getParam("inspire_hand/hand_id", hand_id_);
    nh->getParam("inspire_hand/portname", port_name_);
    nh->getParam("inspire_hand/baudrate", baudrate_);
    nh->getParam("inspire_hand/test_flags", test_flags);

    //Initialize and open serial port
    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));
    if (com_port_->isOpen())
    {
        ROS_INFO_STREAM("Hand: Serial port " << port_name_ << " openned");
        int id_state = 0;
        while (1)
        {
            id_state = start(com_port_);
            if (id_state == 1)
                break;
            hand_id_++;
            if (hand_id_ >= 256)
            {
                ROS_INFO("Id error!!!");
                hand_id_ = 1;
            }
        }

        //Get initial state and discard input buffer
        while (hand_state_ == 0xff)
        {
            //hand_state_ = 0x01;
            hand_state_ = getERROR(com_port_);
            ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
        }

        //Start periodic hand state reading
        //getPeriodicPositionUpdate(com_port_, TF_UPDATE_PERIOD);

        /*ros::Publisher chatter_pub = nh->advertise<std_msgs::Int32MultiArray>("chatter", 1000);
                        ros::Subscriber sub = nh->subscribe("chatter", 1000, arrayCallback);
                        ros::Rate loop_rate(10000);
                        while (ros::ok())
                        {

                                std_msgs::Int32MultiArray array;
                                //Clear array
                                array.data.clear();
                                getANGLE_ACT(com_port_);
                                getFORCE_ACT(com_port_);


                                for (int i = 0; i <6; i++)
                                {
                                        //assign array a random number between 0 and 255.
                                        array.data.push_back(curangle_[i]);

                                }

                                for (int i = 0; i <6; i++)
                                {
                                        //assign array a random number between 0 and 255.
                                        array.data.push_back(curforce_[i]);

                                }
                                //Publish array
                                chatter_pub.publish(array);
                                //Let the world know
                                ROS_INFO("I published something!");
                                //Do this.
                                ros::spinOnce();

                                loop_rate.sleep();
                        }*/
    }
    else
        ROS_ERROR_STREAM("Hand: Serial port " << port_name_ << " not opened");
}

hand_serial::~hand_serial()
{
    com_port_->close();      //Close port
    delete com_port_;        //delete object
}

int
hand_serial::start(serial::Serial *port)
{

    std::vector<uint8_t> output;
    output.push_back(0xEB);
    output.push_back(0x90);
    output.push_back(hand_id_);
    output.push_back(0x04);
    output.push_back(0x11);
    output.push_back(0xFE);
    output.push_back(0x05);
    output.push_back(0x0C);

    unsigned int check_num = 0;

    int len = output[3] + 5;
    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];

    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);

    //Send message to the module and wait for response
    port->write(output);

    ros::Duration(0.015).sleep();

    //Read response
    std::vector<uint8_t> input;
    port->read(input, (size_t)64);
    //ROS_INFO("ok");
    if (input.empty())
        return 0;
    else
        return 1;
}
bool
hand_serial::setID(serial::Serial *port,int id)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x12);
    output.push_back(0xE8);
    output.push_back(0x03);

    unsigned int temp_int2;
    temp_int2 = (unsigned int)id;
    hand_id_ = id;
    output.push_back(temp_int2);
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
    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}
bool
hand_serial::setREDU_RATIO(serial::Serial *port,int redu_ratio)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module redu_ratio
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x12);
    output.push_back(0xE9);
    output.push_back(0x03);

    unsigned int temp_int2;
    temp_int2 = (unsigned int)redu_ratio;

    if (redu_ratio == 0)
        baudrate_ = 115200;
    else if (redu_ratio == 1)
        baudrate_ = 57600;
    else
        baudrate_ = 19200;
    output.push_back(temp_int2);
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
    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

bool
hand_serial::setCLEAR_ERROR(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //mclear_errorsage from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x012);
    output.push_back(0xEC);
    output.push_back(0x03);
    output.push_back(0x01);

    //Checksum calculation
    unsigned int check_num = 0;
    int len = output[3] + 5;
    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);
    //Send mclear_errorsage to the module
    port->write(output);
    ros::Duration(0.5 * 2).sleep();
    std::string s1;
    for (int i = 0; i<output.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", output[i]);
        s1 = s1 + str + " ";
    }
    if (test_flags)
        ROS_INFO_STREAM("Write: " << s1);
    std::vector<uint8_t> input;
    while (input.empty()) {
        port->read(input, (size_t)64);
    }
    std::string s2;
    for (int i = 0; i<input.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", input[i]);
        s2 = s2 + str + " ";
    }
    if (test_flags)
        ROS_INFO_STREAM("Read: " << s2);
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

bool
hand_serial::setSAVE_FLASH(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //msave_flashsage from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x012);
    output.push_back(0xED);
    output.push_back(0x03);
    output.push_back(0x01);

    //Checksum calculation
    unsigned int check_num = 0;
    int len = output[3] + 5;
    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);
    //Send msave_flashsage to the module
    port->write(output);
    ros::Duration(0.5 * 2).sleep();
    std::string s1;
    for (int i = 0; i<output.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", output[i]);
        s1 = s1 + str + " ";
    }
    if (test_flags)
        ROS_INFO_STREAM("Write: " << s1);
    std::vector<uint8_t> input;
    while (input.empty()) {
        port->read(input, (size_t)64);
    }
    std::string s2;
    for (int i = 0; i<input.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", input[i]);
        s2 = s2 + str + " ";
    }
    if (test_flags)
        ROS_INFO_STREAM("Read: " << s2);
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

bool
hand_serial::setRESET_PARA(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //mreset_parasage from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x012);
    output.push_back(0xEE);
    output.push_back(0x03);
    output.push_back(0x01);

    //Checksum calculation
    unsigned int check_num = 0;
    int len = output[3] + 5;
    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);
    //Send mreset_parasage to the module
    port->write(output);
    ros::Duration(0.5 * 2).sleep();
    std::string s1;
    for (int i = 0; i<output.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", output[i]);
        s1 = s1 + str + " ";
    }
    if (test_flags)
        ROS_INFO_STREAM("Write: " << s1);
    std::vector<uint8_t> input;
    while (input.empty()) {
        port->read(input, (size_t)64);
    }
    std::string s2;
    for (int i = 0; i<input.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", input[i]);
        s2 = s2 + str + " ";
    }
    if (test_flags)
        ROS_INFO_STREAM("Read: " << s2);
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

bool
hand_serial::setFORCE_CLB(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //mforce_clbsage from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x012);
    output.push_back(0xF1);
    output.push_back(0x03);
    output.push_back(0x01);

    //Checksum calculation
    unsigned int check_num = 0;
    int len = output[3] + 5;
    for (int i = 2; i < len - 1; i++)
        check_num = check_num + output[i];
    //Add checksum to the output buffer
    output.push_back(check_num & 0xff);
    //Send mforce_clbsage to the module
    port->write(output);
    ros::Duration(0.5 * 2).sleep();
    std::string s1;
    for (int i = 0; i<output.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", output[i]);
        s1 = s1 + str + " ";
    }
    if (test_flags)
        ROS_INFO_STREAM("Write: " << s1);
    std::vector<uint8_t> input;
    while (input.empty()) {
        port->read(input, (size_t)64);
    }
    std::string s2;
    for (int i = 0; i<input.size(); ++i)
    {
        char str[16];
        sprintf(str, "%02X", input[i]);
        s2 = s2 + str + " ";
    }
    if (test_flags)
        ROS_INFO_STREAM("Read: " << s2);
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

bool
hand_serial::setGESTURE_NO(serial::Serial *port, int gesture_no)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module gesture_no
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x12);
    output.push_back(0xF0);
    output.push_back(0x03);

    unsigned int temp_int2;
    temp_int2 = (unsigned int)gesture_no;

    output.push_back(temp_int2);
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
    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}
bool
hand_serial::setCURRENT_LIMIT(serial::Serial *port,int current0,int current1,int current2,int current3,int current4,int current5)
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
    output.push_back(0xFC);
    output.push_back(0x03);

    unsigned int temp_int1,temp_int2,temp_int3,temp_int4,temp_int5,temp_int6;
    temp_int1 = (unsigned int)current0;
    temp_int2 = (unsigned int)current1;
    temp_int3 = (unsigned int)current2;
    temp_int4 = (unsigned int)current3;
    temp_int5 = (unsigned int)current4;
    temp_int6 = (unsigned int)current5;

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
    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}
bool
hand_serial::setDEFAULT_SPEED(serial::Serial *port,int speed0,int speed1,int speed2,int speed3,int speed4,int speed5)
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
    output.push_back(0x08);
    output.push_back(0x04);

    unsigned int temp_int1,temp_int2,temp_int3,temp_int4,temp_int5,temp_int6;
    temp_int1 = (unsigned int)speed0;
    temp_int2 = (unsigned int)speed1;
    temp_int3 = (unsigned int)speed2;
    temp_int4 = (unsigned int)speed3;
    temp_int5 = (unsigned int)speed4;
    temp_int6 = (unsigned int)speed5;

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

    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}
bool
hand_serial::setDEFAULT_FORCE(serial::Serial *port, int force0, int force1, int force2, int force3, int force4, int force5)
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
    output.push_back(0x14);
    output.push_back(0x04);

    unsigned int temp_int1, temp_int2, temp_int3, temp_int4, temp_int5, temp_int6;

    temp_int1 = (unsigned int)force0;
    temp_int2 = (unsigned int)force1;
    temp_int3 = (unsigned int)force2;
    temp_int4 = (unsigned int)force3;
    temp_int5 = (unsigned int)force4;
    temp_int6 = (unsigned int)force5;

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

    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}
bool
hand_serial::setUSER_DEF_ANGLE(serial::Serial *port,int angle0,int angle1,int angle2,int angle3,int angle4,int angle5,int k)

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
    int temp1;
    temp1 = 1066 + (k - 14) * 12;
    unsigned int temp_int;
    temp_int = (unsigned int)temp1;
    output.push_back(temp_int & 0xff);
    output.push_back((temp_int >> 8) & 0xff);

    unsigned int temp_int1, temp_int2, temp_int3, temp_int4, temp_int5, temp_int6;

    temp_int1 = (unsigned int)angle0;
    temp_int2 = (unsigned int)angle1;
    temp_int3 = (unsigned int)angle2;
    temp_int4 = (unsigned int)angle3;
    temp_int5 = (unsigned int)angle4;
    temp_int6 = (unsigned int)angle5;

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

    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

bool
hand_serial::setPOS(serial::Serial *port, int pos0, int pos1, int pos2, int pos3, int pos4, int pos5)
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
    output.push_back(0xC2);
    output.push_back(0x05);

    unsigned int temp_int1, temp_int2, temp_int3, temp_int4, temp_int5, temp_int6;
    temp_int1 = (unsigned int)pos0;
    temp_int2 = (unsigned int)pos1;
    temp_int3 = (unsigned int)pos2;
    temp_int4 = (unsigned int)pos3;
    temp_int5 = (unsigned int)pos4;
    temp_int6 = (unsigned int)pos5;

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

    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}
bool
hand_serial::setANGLE(serial::Serial *port, int angle0, int angle1, int angle2, int angle3, int angle4, int angle5)
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

bool
hand_serial::setFORCE(serial::Serial *port,int force0,int force1,int force2,int force3,int force4,int force5)
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
    output.push_back(0xDA);
    output.push_back(0x05);

    unsigned int temp_int1, temp_int2, temp_int3, temp_int4, temp_int5, temp_int6;
    temp_int1 = (unsigned int)force0;
    temp_int2 = (unsigned int)force1;
    temp_int3 = (unsigned int)force2;
    temp_int4 = (unsigned int)force3;
    temp_int5 = (unsigned int)force4;
    temp_int6 = (unsigned int)force5;

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

    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

bool
hand_serial::setSPEED(serial::Serial *port, int speed0, int speed1, int speed2, int speed3, int speed4, int speed5)
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
    output.push_back(0xF2);
    output.push_back(0x05);

    unsigned int temp_int1, temp_int2, temp_int3, temp_int4, temp_int5, temp_int6;
    temp_int1 = (unsigned int)speed0;
    temp_int2 = (unsigned int)speed1;
    temp_int3 = (unsigned int)speed2;
    temp_int4 = (unsigned int)speed3;
    temp_int5 = (unsigned int)speed4;
    temp_int6 = (unsigned int)speed5;

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

    while (input.empty()) {
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
    int temp[10] = { 0 };
    for (int j = 0; j<1; j++)
        temp[j] = input[7];
    if (temp[0] == 1)
        return true;
    else
        return false;
}

void
hand_serial::getPOS_ACT(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0xFE);
    output.push_back(0x05);
    output.push_back(0x0C);
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

    //Read response
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

    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = ((input[8 + j * 2] << 8) & 0xff00) + input[7 + j * 2];
    //ROS_INFO("Gripper: No error detected");
    ROS_INFO_STREAM("hand: current pos: "
                    << temp[0]
            << " "
            << temp[1]
            << " "
            << temp[2]
            << " "
            << temp[3]
            << " "
            << temp[4]
            << " "
            << temp[5]
               );

    curpos_[0] = float(temp[0]);
    curpos_[1] = float(temp[1]);
    curpos_[2] = float(temp[2]);
    curpos_[3] = float(temp[3]);
    curpos_[4] = float(temp[4]);
    curpos_[5] = float(temp[5]);
}

void
hand_serial::getANGLE_ACT(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0x0A);
    output.push_back(0x06);
    output.push_back(0x0C);
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
    //if (test_flags == 1)
    //ROS_INFO_STREAM("Write: " << s1);


    //Read response
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
    //if (test_flags == 1)
    //ROS_INFO_STREAM("Read: " << s2);


    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = ((input[8 + j * 2] << 8) & 0xff00) + input[7 + j * 2];
    //ROS_INFO("Gripper: No error detected");
    /*ROS_INFO_STREAM("hand: current angle: "
                        << temp[0]
                        << " "
                        << temp[1]
                        << " "
                        << temp[2]
                        << " "
                        << temp[3]
                        << " "
                        << temp[4]
                        << " "
                        << temp[5]
                );*/

    curangle_[0] = float(temp[0]);
    curangle_[1] = float(temp[1]);
    curangle_[2] = float(temp[2]);
    curangle_[3] = float(temp[3]);
    curangle_[4] = float(temp[4]);
    curangle_[5] = float(temp[5]);
}


void
hand_serial::getFORCE_ACT(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0x2E);
    output.push_back(0x06);
    output.push_back(0x0C);
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
    //if (test_flags == 1)
    //ROS_INFO_STREAM("Write: " << s1);


    //Read response
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
    //if (test_flags == 1)
    //ROS_INFO_STREAM("Read: " << s2);

    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
    {
        temp[j] = ((input[8 + j * 2] << 8) & 0xff00) + input[7 + j * 2];
        if(temp[j]>32768)
            temp[j] = temp[j] - 65536;
    }

    //ROS_INFO("Gripper: No error detected");
    /*ROS_INFO_STREAM("hand: current force: "
                        << temp[0]
                        << " "
                        << temp[1]
                        << " "
                        << temp[2]
                        << " "
                        << temp[3]
                        << " "
                        << temp[4]
                        << " "
                        << temp[5]
                );*/

    curforce_[0] = float(temp[0]);
    curforce_[1] = float(temp[1]);
    curforce_[2] = float(temp[2]);
    curforce_[3] = float(temp[3]);
    curforce_[4] = float(temp[4]);
    curforce_[5] = float(temp[5]);
}

void
hand_serial::getCURRENT(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0x3A);
    output.push_back(0x06);
    output.push_back(0x0C);
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

    //Read response
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
    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = ((input[8 + j * 2] << 8) & 0xff00) + input[7 + j * 2];
    //ROS_INFO("Gripper: No error detected");
    ROS_INFO_STREAM("hand: current: "
                    << temp[0]
            << " "
            << temp[1]
            << " "
            << temp[2]
            << " "
            << temp[3]
            << " "
            << temp[4]
            << " "
            << temp[5]
               );
    current_[0] = float(temp[0]);
    current_[1] = float(temp[1]);
    current_[2] = float(temp[2]);
    current_[3] = float(temp[3]);
    current_[4] = float(temp[4]);
    current_[5] = float(temp[5]);
}

uint8_t
hand_serial::getERROR(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0x46);
    output.push_back(0x06);
    output.push_back(0x06);
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
    //Read response
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
    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = input[7 + j];
    //ROS_INFO("Gripper: No error detected");
    ROS_INFO_STREAM("hand: error: "
                    << temp[0]
            << " "
            << temp[1]
            << " "
            << temp[2]
            << " "
            << temp[3]
            << " "
            << temp[4]
            << " "
            << temp[5]
               );
    errorvalue_[0] = float(temp[0]);
    errorvalue_[1] = float(temp[1]);
    errorvalue_[2] = float(temp[2]);
    errorvalue_[3] = float(temp[3]);
    errorvalue_[4] = float(temp[4]);
    errorvalue_[5] = float(temp[5]);
    if (errorvalue_[0] == 0 && errorvalue_[1] == 0 &&errorvalue_[2] == 0 &&errorvalue_[3] == 0 &&errorvalue_[4] == 0 &&errorvalue_[5] == 0)
        return((uint8_t)0x00);
    else
        return((uint8_t)0xff);
}

void
hand_serial::getSTATUS(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0x4C);
    output.push_back(0x06);
    output.push_back(0x06);
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

    //Read response
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

    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = input[7 + j];
    //ROS_INFO("Gripper: No status detected");
    ROS_INFO_STREAM("hand: status: "
                    << temp[0]
            << " "
            << temp[1]
            << " "
            << temp[2]
            << " "
            << temp[3]
            << " "
            << temp[4]
            << " "
            << temp[5]
               );
    statusvalue_[0] = float(temp[0]);
    statusvalue_[1] = float(temp[1]);
    statusvalue_[2] = float(temp[2]);
    statusvalue_[3] = float(temp[3]);
    statusvalue_[4] = float(temp[4]);
    statusvalue_[5] = float(temp[5]);
}

void
hand_serial::getTEMP(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0x52);
    output.push_back(0x06);
    output.push_back(0x06);
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

    //Read response
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

    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = input[7 + j];
    //ROS_INFO("Gripper: No temp detected");
    ROS_INFO_STREAM("hand: temp: "
                    << temp[0]
            << " "
            << temp[1]
            << " "
            << temp[2]
            << " "
            << temp[3]
            << " "
            << temp[4]
            << " "
            << temp[5]
               );
    tempvalue_[0] = float(temp[0]);
    tempvalue_[1] = float(temp[1]);
    tempvalue_[2] = float(temp[2]);
    tempvalue_[3] = float(temp[3]);
    tempvalue_[4] = float(temp[4]);
    tempvalue_[5] = float(temp[5]);
}

void
hand_serial::getPOS_SET(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0xC2);
    output.push_back(0x05);
    output.push_back(0x0C);
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

    //Read response
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

    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = ((input[8 + j * 2] << 8) & 0xff00) + input[7 + j * 2];
    //ROS_INFO("Gripper: No error detected");
    ROS_INFO_STREAM("hand: set pos: "
                    << temp[0]
            << " "
            << temp[1]
            << " "
            << temp[2]
            << " "
            << temp[3]
            << " "
            << temp[4]
            << " "
            << temp[5]
               );
    setpos_[0] = float(temp[0]);
    setpos_[1] = float(temp[1]);
    setpos_[2] = float(temp[2]);
    setpos_[3] = float(temp[3]);
    setpos_[4] = float(temp[4]);
    setpos_[5] = float(temp[5]);
}

void
hand_serial::getANGLE_SET(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0xCE);
    output.push_back(0x05);
    output.push_back(0x0C);
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
    //Read response
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
    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = ((input[8 + j * 2] << 8) & 0xff00) + input[7 + j * 2];
    //ROS_INFO("Gripper: No error detected");
    ROS_INFO_STREAM("hand: set angle: "
                    << temp[0]
            << " "
            << temp[1]
            << " "
            << temp[2]
            << " "
            << temp[3]
            << " "
            << temp[4]
            << " "
            << temp[5]
               );
    setangle_[0] = float(temp[0]);
    setangle_[1] = float(temp[1]);
    setangle_[2] = float(temp[2]);
    setangle_[3] = float(temp[3]);
    setangle_[4] = float(temp[4]);
    setangle_[5] = float(temp[5]);
}

void
hand_serial::getFORCE_SET(serial::Serial *port)
{
    std::vector<uint8_t> output;
    //message from master to module
    output.push_back(0xEB);
    output.push_back(0x90);
    //module id
    output.push_back(hand_id_);
    //Data Length
    output.push_back(0x04);
    //Command get state
    output.push_back(0x11);
    output.push_back(0xDA);
    output.push_back(0x05);
    output.push_back(0x0C);
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
    //Read response
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
    int temp[10] = { 0 };
    for (int j = 0; j<6; j++)
        temp[j] = ((input[8 + j * 2] << 8) & 0xff00) + input[7 + j * 2];
    //ROS_INFO("Gripper: No error detected");
    ROS_INFO_STREAM("hand: set force: "
                    << temp[0]
            << " "
            << temp[1]
            << " "
            << temp[2]
            << " "
            << temp[3]
            << " "
            << temp[4]
            << " "
            << temp[5]
               );
    setforce_[0] = float(temp[0]);
    setforce_[1] = float(temp[1]);
    setforce_[2] = float(temp[2]);
    setforce_[3] = float(temp[3]);
    setforce_[4] = float(temp[4]);
    setforce_[5] = float(temp[5]);
}

//bool
//	hand_serial::setParam(serial::Serial *port)
//{
//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(2);           //Data Length
//	output.push_back(0x01);           //Command reference
//	output.push_back(0x01);

//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module
//	port->write(output);



//	ros::Duration(0.5 * 2).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);
//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 1; j++)
//		temp[j] = input[5];

//	if (temp[0] == 1)
//		return true;
//	else
//		return false;
//}

//uint8_t
//	hand_serial::getState(serial::Serial *port)
//{
//	ROS_INFO("Reading current module state...");
//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(1);           //Data Length
//	output.push_back(0xDC);           //Command get state


//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module and wait for response
//	port->write(output);
//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);
//	//Read response
//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 2; j++)
//		temp[j] = ((input[6 + j * 2] << 8) & 0xff00) + input[5 + j * 2];

//	int error[6] = { 0 };
//	error[0] = input[9] & 0x01;
//	error[1] = (input[9] >> 1) & 0x01;
//	error[2] = (input[9] >> 2) & 0x01;
//	error[3] = (input[9] >> 3) & 0x01;
//	error[4] = (input[9] >> 4) & 0x01;
//	error[5] = (input[9] >> 5) & 0x01;
//	for (int i = 0; i < 6; i++)
//	{
//		if (error[i] == 0)
//			ROS_INFO_STREAM("Driver " << i << ":No error detected");
//		else
//			ROS_INFO_STREAM("Driver " << i << ":Error!!!");
//	}

//	int state[6] = { 0 };
//	for (int k = 0; k < 3; k++)
//	{
//		state[2 * k] = input[10 + k] & 0x0f;
//		state[2 * k + 1] = (input[10 + k] & 0xf0) >> 4;
//	}

//	for (int m = 0; m < 6; m++)
//	{
//		//ROS_INFO_STREAM("Hand "<< int(input[10]));
//		if (state[m] == 1)
//			ROS_INFO_STREAM("Driver " << m << ":in motion");
//		if (state[m] == 2)
//			ROS_INFO_STREAM("Driver " << m << ":stop in place");
//		if (state[m] == 3)
//			ROS_INFO_STREAM("Driver " << m << ":force control in place to stop");
//	}

//	if ((unsigned int)input[9] == 0)
//	{

//		ROS_INFO_STREAM("Hand: Voltage: " << temp[0] / 100.0 << "[V] Temperature: " << temp[1] / 10.0 << "[C] ");
//		return((uint8_t)0x00);
//	}
//	else
//	{
//		//ROS_INFO("Hand: Error!!!");
//		return((uint8_t)0xff);
//	}

//	//Re-start periodic position reading
//	//getPeriodicPositionUpdate(com_port_, TF_UPDATE_PERIOD);


//}

//void
//	hand_serial::getSetangle(serial::Serial *port)
//{
//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(1);           //Data Length
//	output.push_back(0xD7);           //Command get state


//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module and wait for response
//	port->write(output);
//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);

//	//Read response
//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 6; j++)
//		temp[j] = ((input[6 + j * 2] << 8) & 0xff00) + input[5 + j * 2];

//	//ROS_INFO("hand: No error detected");
//	ROS_INFO_STREAM("Hand: Angle: " << temp[0] << " " << temp[1] << " " << temp[2] << " " << temp[3] << " " << temp[4] << " " << temp[5]);

//	setangle_[0] = float(temp[0]);
//	setangle_[1] = float(temp[1]);
//	setangle_[2] = float(temp[2]);
//	setangle_[3] = float(temp[3]);
//	setangle_[4] = float(temp[4]);
//	setangle_[5] = float(temp[5]);

//	//return(act_position_);
//}

//void
//	hand_serial::getCurangle(serial::Serial *port)
//{
//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(1);           //Data Length
//	output.push_back(0xD9);           //Command get state


//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module and wait for response
//	port->write(output);
//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);

//	//Read response
//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 6; j++)
//		temp[j] = ((input[6 + j * 2] << 8) & 0xff00) + input[5 + j * 2];

//	//ROS_INFO("hand: No error detected");
//	ROS_INFO_STREAM("Hand: Angle: " << temp[0] << " " << temp[1] << " " << temp[2] << " " << temp[3] << " " << temp[4] << " " << temp[5]);

//	curangle_[0] = float(temp[0]);
//	curangle_[1] = float(temp[1]);
//	curangle_[2] = float(temp[2]);
//	curangle_[3] = float(temp[3]);
//	curangle_[4] = float(temp[4]);
//	curangle_[5] = float(temp[5]);

//	//return(act_position_);
//}

//bool
//	hand_serial::setPower(serial::Serial *port, int power1, int power2, int power3, int power4, int power5)
//{

//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(11);           //Data Length
//	output.push_back(0x52);           //Command get state

//	//Position <0-70>mm
//	unsigned int temp_int1, temp_int2, temp_int3, temp_int4, temp_int5;
//	temp_int1 = (unsigned int)power1;
//	temp_int2 = (unsigned int)power2;
//	temp_int3 = (unsigned int)power3;
//	temp_int4 = (unsigned int)power4;
//	temp_int5 = (unsigned int)power5;

//	output.push_back(temp_int1 & 0xff);
//	output.push_back((temp_int1 >> 8) & 0xff);
//	output.push_back(temp_int2 & 0xff);
//	output.push_back((temp_int2 >> 8) & 0xff);
//	output.push_back(temp_int3 & 0xff);
//	output.push_back((temp_int3 >> 8) & 0xff);
//	output.push_back(temp_int4 & 0xff);
//	output.push_back((temp_int4 >> 8) & 0xff);
//	output.push_back(temp_int5 & 0xff);
//	output.push_back((temp_int5 >> 8) & 0xff);


//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module
//	port->write(output);

//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);

//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 1; j++)
//		temp[j] = input[5];

//	if (temp[0] == 1)
//		return true;
//	else
//		return false;
//}



//void
//	hand_serial::getPower(serial::Serial *port)
//{
//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(1);           //Data Length
//	output.push_back(0xD3);           //Command get state


//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module and wait for response
//	port->write(output);
//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);

//	//Read response
//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 5; j++)
//		temp[j] = ((input[6 + j * 2] << 8) & 0xff00) + input[5 + j * 2];

//	//ROS_INFO("hand: No error detected");
//	ROS_INFO_STREAM("Hand: Power: " << temp[0] << " " << temp[1] << " " << temp[2] << " " << temp[3] << " " << temp[4]);

//	power_[0] = float(temp[0]);
//	power_[1] = float(temp[1]);
//	power_[2] = float(temp[2]);
//	power_[3] = float(temp[3]);
//	power_[4] = float(temp[4]);

//}

//bool
//	hand_serial::setSpeed(serial::Serial *port, int speed1, int speed2, int speed3, int speed4, int speed5, int speed6)
//{

//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(13);           //Data Length
//	output.push_back(0x51);           //Command get state

//	//Position <0-70>mm
//	unsigned int temp_int1, temp_int2, temp_int3, temp_int4, temp_int5, temp_int6;
//	temp_int1 = (unsigned int)speed1;
//	temp_int2 = (unsigned int)speed2;
//	temp_int3 = (unsigned int)speed3;
//	temp_int4 = (unsigned int)speed4;
//	temp_int5 = (unsigned int)speed5;
//	temp_int6 = (unsigned int)speed6;

//	output.push_back(temp_int1 & 0xff);
//	output.push_back((temp_int1 >> 8) & 0xff);
//	output.push_back(temp_int2 & 0xff);
//	output.push_back((temp_int2 >> 8) & 0xff);
//	output.push_back(temp_int3 & 0xff);
//	output.push_back((temp_int3 >> 8) & 0xff);
//	output.push_back(temp_int4 & 0xff);
//	output.push_back((temp_int4 >> 8) & 0xff);
//	output.push_back(temp_int5 & 0xff);
//	output.push_back((temp_int5 >> 8) & 0xff);
//	output.push_back(temp_int6 & 0xff);
//	output.push_back((temp_int6 >> 8) & 0xff);


//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module
//	port->write(output);

//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);

//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 1; j++)
//		temp[j] = input[5];

//	if (temp[0] == 1)
//		return true;
//	else
//		return false;

//}

//void
//	hand_serial::getSpeed(serial::Serial *port)
//{
//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(1);           //Data Length
//	output.push_back(0xD2);           //Command get state


//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module and wait for response
//	port->write(output);
//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);

//	//Read response
//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 6; j++)
//		temp[j] = ((input[6 + j * 2] << 8) & 0xff00) + input[5 + j * 2];

//	//ROS_INFO("hand: No error detected");
//	ROS_INFO_STREAM("Hand: Speed: " << temp[0] << " " << temp[1] << " " << temp[2] << " " << temp[3] << " " << temp[4] << " " << temp[5]);

//	speed_[0] = float(temp[0]);
//	speed_[1] = float(temp[1]);
//	speed_[2] = float(temp[2]);
//	speed_[3] = float(temp[3]);
//	speed_[4] = float(temp[4]);
//	speed_[5] = float(temp[5]);

//	//return(act_position_);
//}

//bool
//	hand_serial::setAngle(serial::Serial *port, int angle1, int angle2, int angle3, int angle4, int angle5, int angle6)
//{

//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(13);           //Data Length
//	output.push_back(0x54);           //Command get state

//	//Position <0-70>mm
//	unsigned int temp_int1, temp_int2, temp_int3, temp_int4, temp_int5, temp_int6;
//	temp_int1 = (unsigned int)angle1;
//	temp_int2 = (unsigned int)angle2;
//	temp_int3 = (unsigned int)angle3;
//	temp_int4 = (unsigned int)angle4;
//	temp_int5 = (unsigned int)angle5;
//	temp_int6 = (unsigned int)angle6;

//	output.push_back(temp_int1 & 0xff);
//	output.push_back((temp_int1 >> 8) & 0xff);
//	output.push_back(temp_int2 & 0xff);
//	output.push_back((temp_int2 >> 8) & 0xff);
//	output.push_back(temp_int3 & 0xff);
//	output.push_back((temp_int3 >> 8) & 0xff);
//	output.push_back(temp_int4 & 0xff);
//	output.push_back((temp_int4 >> 8) & 0xff);
//	output.push_back(temp_int5 & 0xff);
//	output.push_back((temp_int5 >> 8) & 0xff);
//	output.push_back(temp_int6 & 0xff);
//	output.push_back((temp_int6 >> 8) & 0xff);


//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module
//	port->write(output);

//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);

//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 1; j++)
//		temp[j] = input[5];

//	if (temp[0] == 1)
//		return true;
//	else
//		return false;

//}

//bool
//	hand_serial::setID(serial::Serial *port, int id)
//{
//	std::vector<uint8_t> output;
//	output.push_back(0xEB);           //message from master to module
//	output.push_back(0x90);
//	output.push_back(hand_id_);    //module id
//	output.push_back(2);           //Data Length
//	output.push_back(0x04);           //Command get state

//	unsigned int temp_int2;
//	temp_int2 = (unsigned int)id;
//	hand_id_ = id;
//	//temp_int1 = (unsigned int)power;

//	output.push_back(temp_int2);
//	//output.push_back(temp_int1&0xFF);
//	//output.push_back((temp_int1>>8)&0xFF);
//	//Checksum calculation
//	unsigned int check_num = 0;

//	int len = output[3] + 5;
//	for (int i = 2; i < len - 1; i++)
//		check_num = check_num + output[i];

//	//Add checksum to the output buffer
//	output.push_back(check_num & 0xff);

//	//Send message to the module and wait for response
//	port->write(output);
//	//ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();

//	ros::Duration(0.015).sleep();

//	std::string s1;
//	for (int i = 0; i < output.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", output[i]);
//		s1 = s1 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Write: " << s1);

//	std::vector<uint8_t> input;
//	while (input.empty()) {
//		port->read(input, (size_t)64);
//	}

//	std::string s2;
//	for (int i = 0; i < input.size(); ++i)
//	{
//		char str[16];
//		sprintf(str, "%02X", input[i]);
//		s2 = s2 + str + " ";
//	}
//	if (test_flags == 1)
//		ROS_INFO_STREAM("Read: " << s2);

//	int temp[10] = { 0 };
//	for (int j = 0; j < 1; j++)
//		temp[j] = input[5];

//	if (temp[0] == 1)
//		return true;
//	else
//		return false;

//}


/////////////////////////////////////////////////////////////
//CALLBACKS
/////////////////////////////////////////////////////////////
bool
hand_serial::setIDCallback(inspire_hand::set_id::Request &req,
                           inspire_hand::set_id::Response &res)
{
    ROS_INFO("hand: reset id");
    if (req.id>0 && req.id<255)
    {
        res.idgrab = setID(com_port_, req.id);
    }
    else
    {
        ROS_INFO("Hand: error id([1 254])!");
        res.idgrab = false;
    }
}

bool
hand_serial::setREDU_RATIOCallback(inspire_hand::set_redu_ratio::Request &req,
                                   inspire_hand::set_redu_ratio::Response &res)
{
    ROS_INFO("hand: reset redu_ratio");
    if (req.redu_ratio>-1 && req.redu_ratio<3)
    {
        res.redu_ratiograb = setREDU_RATIO(com_port_, req.redu_ratio);
    }
    else
    {
        ROS_INFO("Hand: error redu_ratio([0 2])!");
        res.redu_ratiograb = false;
    }
}

bool
hand_serial::setCLEAR_ERRORCallback(inspire_hand::set_clear_error::Request &req,
                                    inspire_hand::set_clear_error::Response &res)
{
    ROS_INFO("Hand: clear error Cmd recieved ");
    res.setclear_error_accepted = setCLEAR_ERROR(com_port_);
}

bool
hand_serial::setSAVE_FLASHCallback(inspire_hand::set_save_flash::Request &req,
                                   inspire_hand::set_save_flash::Response &res)
{
    ROS_INFO("Hand: save para to flash Cmd recieved ");
    res.setsave_flash_accepted = setSAVE_FLASH(com_port_);
}

bool
hand_serial::setRESET_PARACallback(inspire_hand::set_reset_para::Request &req,
                                   inspire_hand::set_reset_para::Response &res)
{
    ROS_INFO("Hand: reset para Cmd recieved ");
    res.setreset_para_accepted = setRESET_PARA(com_port_);
}

bool
hand_serial::setFORCE_CLBCallback(inspire_hand::set_force_clb::Request &req,
                                  inspire_hand::set_force_clb::Response &res)
{
    ROS_INFO("Hand:gesture force clb Cmd recieved ");
    res.setforce_clb_accepted = setFORCE_CLB(com_port_);
}

bool
hand_serial::setGESTURE_NOCallback(inspire_hand::set_gesture_no::Request &req,
                                   inspire_hand::set_gesture_no::Response &res)
{
    ROS_INFO("hand: reset gesture_no");
    if (req.gesture_no>-1 && req.gesture_no<46)
    {
        res.gesture_nograb = setGESTURE_NO(com_port_, req.gesture_no);
    }
    else
    {
        ROS_INFO("Hand: error gesture_no([0 45])!");
        res.gesture_nograb = false;
    }
}

bool
hand_serial::setCURRENT_LIMITCallback(inspire_hand::set_current_limit::Request &req,
                                      inspire_hand::set_current_limit::Response &res)
{
    ROS_INFO("hand: set current limit");
    if (req.current0 >= 0&& req.current1 >= 0&& req.current2 >= 0&& req.current3 >= 0&& req.current4 >= 0&& req.current5 >= 0)
    {
        if (req.current0 <= 1500&& req.current1 <= 1500&& req.current2 <= 1500&& req.current3 <= 1500&& req.current4 <= 1500&& req.current5 <= 1500)
        {
            res.current_limit_accepted = setCURRENT_LIMIT(com_port_,req.current0,req.current1,req.current2,req.current3,req.current4,req.current5);
        }
        else
        {
            ROS_WARN("Hand: current error!");
            res.current_limit_accepted = false;
        }
    }
    else
    {
        ROS_WARN("Hand: current error!");
        res.current_limit_accepted = false;
    }
}

bool
hand_serial::setDEFAULT_SPEEDCallback(inspire_hand::set_default_speed::Request &req,
                                      inspire_hand::set_default_speed::Response &res)
{
    ROS_INFO("hand: set speed limit");
    if (req.speed0 >= 0&& req.speed1 >= 0&& req.speed2 >= 0&& req.speed3 >= 0&& req.speed4 >= 0&& req.speed5 >= 0)
    {
        if (req.speed0 <= 1000&& req.speed1 <= 1000&& req.speed2 <= 1000&& req.speed3 <= 1000&& req.speed4 <= 1000&& req.speed5 <= 1000)
        {
            res.default_speed_accepted = setDEFAULT_SPEED(com_port_,req.speed0,req.speed1,req.speed2,req.speed3,req.speed4,req.speed5);
        }
        else
        {
            ROS_WARN("Hand: speed error!");
            res.default_speed_accepted = false;
        }
    }
    else
    {
        ROS_WARN("Hand: speed error!");
        res.default_speed_accepted = false;
    }
}

bool
hand_serial::setDEFAULT_FORCECallback(inspire_hand::set_default_force::Request &req,
                                      inspire_hand::set_default_force::Response &res)
{
    ROS_INFO("hand: set force limit");
    if (req.force0 >= 0&& req.force1 >= 0&& req.force2 >= 0&& req.force3 >= 0&& req.force4 >= 0&& req.force5 >= 0)
    {
        if (req.force0 <= 1000 && req.force1 <= 1000 && req.force2 <= 1000 && req.force3 <= 1000 && req.force4 <= 1000 && req.force5 <= 1000)
        {
            res.default_force_accepted = setDEFAULT_FORCE(com_port_, req.force0, req.force1, req.force2, req.force3, req.force4, req.force5);
        }
        else
        {
            ROS_WARN("Hand: force error!");
            res.default_force_accepted = false;
        }
    }
    else
    {
        ROS_WARN("Hand: force error!");
        res.default_force_accepted = false;
    }
}

bool
hand_serial::setUSER_DEF_ANGLECallback(inspire_hand::set_user_def_angle::Request &req,
                                       inspire_hand::set_user_def_angle::Response &res)
{
    ROS_INFO("hand: set user_def_angle");
    if (req.k >= 14 && req.k <= 45)
    {
        ;
    }
    else
    {
        ROS_WARN("Hand: k error!");
        res.angle_accepted = false;
        return false;
    }
    if (req.angle0 >= 0&& req.angle1 >= 0&& req.angle2 >= 0&& req.angle3 >= 0&& req.angle4 >= 0&& req.angle5 >= 0)
    {
        if (req.angle0 <= 1000&& req.angle1 <= 1000&& req.angle2 <= 1000&& req.angle3 <= 1000&& req.angle4 <= 1000&& req.angle5 <= 1000)
        {
            res.angle_accepted = setANGLE(com_port_,req.angle0,req.angle1,req.angle2,req.angle3,req.angle4,req.angle5);
        }

        else
        {
            ROS_WARN("Hand: angle error!");
            res.angle_accepted = false;
        }
    }
    else
    {
        ROS_WARN("Hand: angle error!");
        res.angle_accepted = false;
    }
}

bool
hand_serial::setPOSCallback(inspire_hand::set_pos::Request &req,
                            inspire_hand::set_pos::Response &res)
{
    ROS_INFO("hand: set pos");
    if (req.pos0 >= 0&& req.pos1 >= 0&& req.pos2 >= 0&& req.pos3 >= 0&& req.pos4 >= 0&& req.pos5 >= 0)
    {
        if (req.pos0 <= 2000&& req.pos1 <= 2000&& req.pos2 <= 2000&& req.pos3 <= 2000&& req.pos4 <= 2000&& req.pos5 <= 2000)
        {
            res.pos_accepted = setPOS(com_port_,req.pos0,req.pos1,req.pos2,req.pos3,req.pos4,req.pos5);
        }
        else
        {
            ROS_WARN("Hand: pos error!");
            res.pos_accepted = false;
        }
    }
    else
    {
        ROS_WARN("Hand: pos error!");
        res.pos_accepted = false;
    }
}

bool
hand_serial::setANGLECallback(inspire_hand::set_angle::Request &req,
                              inspire_hand::set_angle::Response &res)
{
    ROS_INFO("hand: set angle");
    if (req.angle0 >= -1&& req.angle1 >= -1&& req.angle2 >= -1&& req.angle3 >= -1&& req.angle4 >= -1&& req.angle5 >= -1)
    {
        if (req.angle0 <= 1000&& req.angle1 <= 1000&& req.angle2 <= 1000&& req.angle3 <= 1000&& req.angle4 <= 1000&& req.angle5 <= 1000)
        {
            res.angle_accepted = setANGLE(com_port_,req.angle0,req.angle1,req.angle2,req.angle3,req.angle4,req.angle5);
        }
        else
        {
            ROS_WARN("Hand: angle error!");
            res.angle_accepted = false;
        }
    }
    else
    {
        ROS_WARN("Hand: angle error!");
        res.angle_accepted = false;
    }
}

bool
hand_serial::setFORCECallback(inspire_hand::set_force::Request &req,
                              inspire_hand::set_force::Response &res)
{
    ROS_INFO("hand: set force");
    if (req.force0 >= 0&& req.force1 >= 0&& req.force2 >= 0&& req.force3 >= 0&& req.force4 >= 0&& req.force5 >= 0)
    {
        if (req.force0 <= 1000&& req.force1 <= 1000&& req.force2 <= 1000&& req.force3 <= 1000&& req.force4 <= 1000&& req.force5 <= 1000)
        {
            res.force_accepted = setFORCE(com_port_,req.force0,req.force1,req.force2,req.force3,req.force4,req.force5);
        }
        else
        {
            ROS_WARN("Hand: force error!");
            res.force_accepted = false;
        }
    }
    else
    {
        ROS_WARN("Hand: force error!");
        res.force_accepted = false;
    }
}

bool
hand_serial::setSPEEDCallback(inspire_hand::set_speed::Request &req,
                              inspire_hand::set_speed::Response &res)
{
    ROS_INFO("hand: set speed");
    if (req.speed0 >= 0&& req.speed1 >= 0&& req.speed2 >= 0&& req.speed3 >= 0&& req.speed4 >= 0&& req.speed5 >= 0)
    {
        if (req.speed0 <= 1000&& req.speed1 <= 1000&& req.speed2 <= 1000&& req.speed3 <= 1000&& req.speed4 <= 1000&& req.speed5 <= 1000)
        {
            res.speed_accepted = setSPEED(com_port_,req.speed0,req.speed1,req.speed2,req.speed3,req.speed4,req.speed5);

        }

        else
        {
            ROS_WARN("Hand: speed error!");
            res.speed_accepted = false;
        }
    }
    else
    {
        ROS_WARN("Hand: speed error!");
        res.speed_accepted = false;
    }
}

bool
hand_serial::getPOS_ACTCallback(inspire_hand::get_pos_act::Request &req,
                                inspire_hand::get_pos_act::Response &res)
{
    ROS_INFO("Hand: Get act pos request recieved");
    getPOS_ACT(com_port_);
    res.curpos[0] = curpos_[0];
    res.curpos[1] = curpos_[1];
    res.curpos[2] = curpos_[2];
    res.curpos[3] = curpos_[3];
    res.curpos[4] = curpos_[4];
    res.curpos[5] = curpos_[5];
}

bool
hand_serial::getANGLE_ACTCallback(inspire_hand::get_angle_act::Request &req,
                                  inspire_hand::get_angle_act::Response &res)
{
    ROS_INFO("Hand: Get act angle request recieved");
    getANGLE_ACT(com_port_);
    res.curangle[0] = curangle_[0];
    res.curangle[1] = curangle_[1];
    res.curangle[2] = curangle_[2];
    res.curangle[3] = curangle_[3];
    res.curangle[4] = curangle_[4];
    res.curangle[5] = curangle_[5];
}

bool
hand_serial::getFORCE_ACTCallback(inspire_hand::get_force_act::Request &req,
                                  inspire_hand::get_force_act::Response &res)
{
    ROS_INFO("Hand: Get act force request recieved");

    getFORCE_ACT(com_port_);
    res.curforce[0] = curforce_[0];
    res.curforce[1] = curforce_[1];
    res.curforce[2] = curforce_[2];
    res.curforce[3] = curforce_[3];
    res.curforce[4] = curforce_[4];
    res.curforce[5] = curforce_[5];
}

bool
hand_serial::getCURRENTCallback(inspire_hand::get_current::Request &req,
                                inspire_hand::get_current::Response &res)
{
    ROS_INFO("Hand: Get current request recieved");
    getCURRENT(com_port_);
    res.current[0] = current_[0];
    res.current[1] = current_[1];
    res.current[2] = current_[2];
    res.current[3] = current_[3];
    res.current[4] = current_[4];
    res.current[5] = current_[5];
}

bool
hand_serial::getERRORCallback(inspire_hand::get_error::Request &req,
                              inspire_hand::get_error::Response &res)
{
    ROS_INFO("Hand: Get error request recieved");
    getERROR(com_port_);
    res.errorvalue[0] = errorvalue_[0];
    res.errorvalue[1] = errorvalue_[1];
    res.errorvalue[2] = errorvalue_[2];
    res.errorvalue[3] = errorvalue_[3];
    res.errorvalue[4] = errorvalue_[4];
    res.errorvalue[5] = errorvalue_[5];
}

bool
hand_serial::getSTATUSCallback(inspire_hand::get_status::Request &req,
                               inspire_hand::get_status::Response &res)
{
    ROS_INFO("Hand: Get status request recieved");
    getSTATUS(com_port_);
    res.statusvalue[0] = statusvalue_[0];
    res.statusvalue[1] = statusvalue_[1];
    res.statusvalue[2] = statusvalue_[2];
    res.statusvalue[3] = statusvalue_[3];
    res.statusvalue[4] = statusvalue_[4];
    res.statusvalue[5] = statusvalue_[5];
}

bool
hand_serial::getTEMPCallback(inspire_hand::get_temp::Request &req,
                             inspire_hand::get_temp::Response &res)
{
    ROS_INFO("Hand: Get temp request recieved");
    getTEMP(com_port_);
    res.tempvalue[0] = tempvalue_[0];
    res.tempvalue[1] = tempvalue_[1];
    res.tempvalue[2] = tempvalue_[2];
    res.tempvalue[3] = tempvalue_[3];
    res.tempvalue[4] = tempvalue_[4];
    res.tempvalue[5] = tempvalue_[5];
}

bool
hand_serial::getPOS_SETCallback(inspire_hand::get_pos_set::Request &req,
                                inspire_hand::get_pos_set::Response &res)
{
    ROS_INFO("Hand: Get act pos request recieved");
    getPOS_SET(com_port_);
    res.setpos[0] = setpos_[0];
    res.setpos[1] = setpos_[1];
    res.setpos[2] = setpos_[2];
    res.setpos[3] = setpos_[3];
    res.setpos[4] = setpos_[4];
    res.setpos[5] = setpos_[5];
}

bool
hand_serial::getANGLE_SETCallback(inspire_hand::get_angle_set::Request &req,
                                  inspire_hand::get_angle_set::Response &res)
{
    ROS_INFO("Hand: Get set angle request recieved");
    getANGLE_SET(com_port_);
    res.setangle[0] = setangle_[0];
    res.setangle[1] = setangle_[1];
    res.setangle[2] = setangle_[2];
    res.setangle[3] = setangle_[3];
    res.setangle[4] = setangle_[4];
    res.setangle[5] = setangle_[5];
}

bool
hand_serial::getFORCE_SETCallback(inspire_hand::get_force_set::Request &req,
                                  inspire_hand::get_force_set::Response &res)
{
    ROS_INFO("Hand: Get set force request recieved");
    getFORCE_SET(com_port_);
    res.setforce[0] = setforce_[0];
    res.setforce[1] = setforce_[1];
    res.setforce[2] = setforce_[2];
    res.setforce[3] = setforce_[3];
    res.setforce[4] = setforce_[4];
    res.setforce[5] = setforce_[5];
}


//bool
//	hand_serial::setParamCallback(inspire_hand::set_param::Request &req,
//		inspire_hand::set_param::Response &res)
//{
//	ROS_INFO("Hand: SetParam Cmd recieved ");
//	res.setparam_accepted = setParam(com_port_);
//	//res.setparam_accepted = true;
//}

//bool
//	hand_serial::getSetangleCallback(inspire_hand::get_setangle::Request &req,
//		inspire_hand::get_setangle::Response &res)
//{
//	ROS_INFO("hand: Get Set Angle request recieved");
//	getSetangle(com_port_);
//	res.gsangle[0] = setangle_[0];
//	res.gsangle[1] = setangle_[1];
//	res.gsangle[2] = setangle_[2];
//	res.gsangle[3] = setangle_[3];
//	res.gsangle[4] = setangle_[4];
//	res.gsangle[5] = setangle_[5];
//}

//bool
//	hand_serial::getCurangleCallback(inspire_hand::get_curangle::Request &req,
//		inspire_hand::get_curangle::Response &res)
//{
//	ROS_INFO("hand: Get Current Angle request recieved");
//	getCurangle(com_port_);
//	res.cangle[0] = curangle_[0];
//	res.cangle[1] = curangle_[1];
//	res.cangle[2] = curangle_[2];
//	res.cangle[3] = curangle_[3];
//	res.cangle[4] = curangle_[4];
//	res.cangle[5] = curangle_[5];
//}

//bool
//	hand_serial::getPowerCallback(inspire_hand::get_power::Request &req,
//		inspire_hand::get_power::Response &res)
//{
//	ROS_INFO("hand: Get Power request recieved");
//	getPower(com_port_);
//	res.power[0] = power_[0];
//	res.power[1] = power_[1];
//	res.power[2] = power_[2];
//	res.power[3] = power_[3];
//	res.power[4] = power_[4];
//	//res.power[5] = power_[5];
//}

//bool
//	hand_serial::setPowerCallback(inspire_hand::set_power::Request &req,
//		inspire_hand::set_power::Response &res)
//{


//	if (req.power1 >= 0 && req.power2 >= 0 && req.power3 >= 0 && req.power4 >= 0 && req.power5 >= 0)
//	{

//		if (req.power1 <= 1000 && req.power2 <= 1000 && req.power3 <= 1000 && req.power4 <= 1000 && req.power5 <= 1000)
//		{
//			setPower(com_port_, req.power1, req.power2, req.power3, req.power4, req.power5);
//			res.pgoal_accepted = true;
//		}
//		else
//		{
//			ROS_WARN("Hand: Power error!");
//			res.pgoal_accepted = false;
//		}
//	}
//	else
//	{
//		ROS_WARN("Hand: Power error!");
//		res.pgoal_accepted = false;
//	}
//}

//bool
//	hand_serial::setSpeedCallback(inspire_hand::set_speed::Request &req,
//		inspire_hand::set_speed::Response &res)
//{


//	if (req.speed1 >= 0 && req.speed2 >= 0 && req.speed3 >= 0 && req.speed4 >= 0 && req.speed5 >= 0 && req.speed6 >= 0)
//	{

//		if (req.speed1 <= 1000 && req.speed2 <= 1000 && req.speed3 <= 1000 && req.speed4 <= 1000 && req.speed5 <= 1000 && req.speed6 <= 1000)
//		{
//			for (int i = 0; i < 10; i++)
//			{
//				res.sgoal_accepted = setSpeed(com_port_, req.speed1, req.speed2, req.speed3, req.speed4, req.speed5, req.speed6);
//				if (res.sgoal_accepted == true)
//					break;
//				else
//					ROS_INFO("Error! Retrying...");
//			}

//			//res.sgoal_accepted = true;
//		}
//		else
//		{
//			ROS_WARN("Hand: Speed error!");
//			res.sgoal_accepted = false;
//		}
//	}
//	else
//	{
//		ROS_WARN("Hand: Speed error!");
//		res.sgoal_accepted = false;
//	}
//}

//bool
//	hand_serial::getSpeedCallback(inspire_hand::get_speed::Request &req,
//		inspire_hand::get_speed::Response &res)
//{
//	ROS_INFO("hand: Get speed request recieved");
//	getSpeed(com_port_);
//	res.speed[0] = speed_[0];
//	res.speed[1] = speed_[1];
//	res.speed[2] = speed_[2];
//	res.speed[3] = speed_[3];
//	res.speed[4] = speed_[4];
//	res.speed[5] = speed_[5];
//}

//bool
//	hand_serial::setAngleCallback(inspire_hand::set_angle::Request &req,
//		inspire_hand::set_angle::Response &res)
//{


//	if (req.angle1 >= 0 && req.angle2 >= 0 && req.angle3 >= 0 && req.angle4 >= 0 && req.angle5 >= 0 && req.angle6 >= 0)
//	{

//		if (req.angle1 <= 1000 && req.angle2 <= 1000 && req.angle3 <= 1000 && req.angle4 <= 1000 && req.angle5 <= 1000 && req.angle6 <= 1000)
//		{
//			res.agoal_accepted = setAngle(com_port_, req.angle1, req.angle2, req.angle3, req.angle4, req.angle5, req.angle6);
//			//res.agoal_accepted = true;
//		}
//		else
//		{
//			ROS_WARN("Hand: Angle error!");
//			res.agoal_accepted = false;
//		}
//	}
//	else
//	{
//		ROS_WARN("Hand: Angle error!");
//		res.agoal_accepted = false;
//	}
//}

//bool
//	hand_serial::setIDCallback(inspire_hand::set_id::Request &req,
//		inspire_hand::set_id::Response &res)
//{
//	ROS_INFO("Hand: reset id");
//	if (req.id > 0 && req.id < 255)
//	{
//		res.idgrab = setID(com_port_, req.id);
//		//ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();

//		//res.idgrab = true;
//	}
//	else {
//		ROS_INFO("Hand: error id!");
//		res.idgrab = false;
//	}


//}

//bool
//	hand_serial::getStateCallback(inspire_hand::get_state::Request &req,
//		inspire_hand::get_state::Response &res)
//{
//	ROS_INFO("hand: Get state request recieved");
//	res.error_code = getState(com_port_);
//}

/*void
                hand_serial::timerCallback(const ros::TimerEvent &event)
        {
                std::vector<uint8_t> output;
                output.push_back(0xEB);           //message from master to module
                output.push_back(0x90);
                output.push_back(hand_id_);    //module id
                output.push_back(1);              //Data Length
                output.push_back(0xD9);           //Command get state


                //Checksum calculation
                unsigned int check_num = 0;

                int len = output[3] + 5;
                for (int i = 2; i < len - 1; i++)
                        check_num = check_num + output[i];

                //Add checksum to the output buffer
                output.push_back(check_num & 0xff);

                //Send message to the module and wait for response
                com_port_->write(output);
                ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();

                //Read response
                std::vector<uint8_t> input;
                com_port_->read(input, (size_t)64);

                int temp[10] = { 0 };
                for (int j = 0; j < 6; j++)
                        temp[j] = ((input[6 + j * 2] << 8) & 0xff00) + input[5 + j * 2];

                //ROS_INFO("hand: No error detected");
                //ROS_INFO_STREAM("Hand: Angle: " << temp[0] << " " << temp[1]<< " " << temp[2]<< " " << temp[3]<< " " << temp[4]<< " " << temp[5]);

                curangle_1[0] = float(temp[0]);
                curangle_1[1] = float(temp[1]);
                curangle_1[2] = float(temp[2]);
                curangle_1[3] = float(temp[3]);
                curangle_1[4] = float(temp[4]);
                curangle_1[5] = float(temp[5]);


                //Publish TF
                hand_joint_state_.header.stamp = ros::Time::now();
                hand_joint_state_.name.clear();
                hand_joint_state_.position.clear();

                hand_joint_state_.name.push_back("angle1");
                hand_joint_state_.position.push_back(curangle_1[0]);

                hand_joint_state_.name.push_back("angle2");
                hand_joint_state_.position.push_back(curangle_1[1]);

                hand_joint_state_.name.push_back("angle3");
                hand_joint_state_.position.push_back(curangle_1[2]);

                hand_joint_state_.name.push_back("angle4");
                hand_joint_state_.position.push_back(curangle_1[3]);

                hand_joint_state_.name.push_back("angle5");
                hand_joint_state_.position.push_back(curangle_1[4]);

                hand_joint_state_.name.push_back("angle6");
                hand_joint_state_.position.push_back(curangle_1[5]);


                joint_pub.publish(hand_joint_state_);

        }
        */
////////////////////////////////////////////////////
//ADDITIONAL FUNCTIONS
////////////////////////////////////////////////////

float
hand_serial::IEEE_754_to_float(uint8_t *raw)
{
    int sign = (raw[0] >> 7) ? -1 : 1;
    int8_t exponent = (raw[0] << 1) + (raw[1] >> 7) - 126;

    uint32_t fraction_bits = ((raw[1] & 0x7F) << 16) + (raw[2] << 8) + raw[3];

    float fraction = 0.5f;
    for (uint8_t ii = 0; ii < 24; ++ii)
        fraction += ldexpf((fraction_bits >> (23 - ii)) & 1, -(ii + 1));

    float significand = sign * fraction;

    return ldexpf(significand, exponent);
}

void
hand_serial::float_to_IEEE_754(float position, unsigned int *output_array)
{
    unsigned char *p_byte = (unsigned char*)(&position);

    for (size_t i = 0; i < sizeof(float); i++)
        output_array[i] = (static_cast<unsigned int>(p_byte[i]));
}

uint16_t
hand_serial::CRC16(uint16_t crc, uint16_t data)
{
    const uint16_t tbl[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    return(((crc & 0xFF00) >> 8) ^ tbl[(crc & 0x00FF) ^ (data & 0x00FF)]);
}
}

#endif
