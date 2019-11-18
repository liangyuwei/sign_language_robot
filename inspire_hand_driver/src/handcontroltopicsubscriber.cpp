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
float setangle_[6];
float setforce_[6];

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
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "handcontroltopicsubscriber");
    ros::NodeHandle nh;


    //Read launch file params
    nh.getParam("inspire_hand/hand_id", hand_id_);
    nh.getParam("inspire_hand/portname", port_name_);
    nh.getParam("inspire_hand/baudrate", baudrate_);
    nh.getParam("inspire_hand/test_flags", test_flags);
    //Initialize and open serial port
    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));

    //topic
    ros::Subscriber sub = nh.subscribe("chatter", 1000, arrayCallback);
    ros::spin();

    return(EXIT_SUCCESS);
}
