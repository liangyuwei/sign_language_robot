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


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "handcontroltopicsubscriber1");
    ros::NodeHandle nh;


    //Read launch file params
    nh.getParam("inspire_hand/hand_id", hand_id_);
    nh.getParam("inspire_hand/portname", port_name_);
    nh.getParam("inspire_hand/baudrate", baudrate_);
    nh.getParam("inspire_hand/test_flags", test_flags);
    //Initialize and open serial port
    com_port_ = new serial::Serial(port_name_, (uint32_t)baudrate_, serial::Timeout::simpleTimeout(100));

    //topic

    ros::Publisher  pub = nh.advertise<std_msgs::Int32MultiArray>("chatter1", 1000);

    ros::Rate loop_rate(10);
    setangle_[0] = 1000;
    setangle_[1] = 1000;
    setangle_[2] = 1000;
    setangle_[3] = 1000;
    setangle_[4] = 1000;
    setangle_[5] = 1000;

    setforce_[0] = 200;
    setforce_[1] = 200;
    setforce_[2] = 200;
    setforce_[3] = 200;
    setforce_[4] = 200;
    setforce_[5] = 200;

    while (ros::ok())
    {
        std_msgs::Int32MultiArray array;
        //Clear array
        array.data.clear();


        for (int i = 0; i <6; i++)
        {
            //assign array a random number between 0 and 255.
            array.data.push_back(setangle_[i]);

        }
        for (int i = 0; i <6; i++)
        {
            //assign array a random number between 0 and 255.
            array.data.push_back(setforce_[i]);

        }
        //Publish array
        pub.publish(array);


        loop_rate.sleep();
    }

    return(EXIT_SUCCESS);
}
