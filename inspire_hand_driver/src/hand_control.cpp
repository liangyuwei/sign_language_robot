#include <hand_control.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>


//#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

/*int Arr[2];

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{

        int i = 0;
        // print all the remaining numbers
        for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
        {
                Arr[i] = *it;
                printf("%d, ", Arr[i]);
                i++;
        }

        return;
}*/

int
main(int argc, char *argv[])
{
    ros::init(argc, argv, "hand_control");
    ros::NodeHandle nh;

    //Create hand object instance
    inspire_hand::hand_serial hand(&nh);


    //Initialize user interface
    ros::ServiceServer set_id_service = nh.advertiseService("inspire_hand/set_id", &inspire_hand::hand_serial::setIDCallback, &hand);
    ros::ServiceServer set_redu_ratio_service = nh.advertiseService("inspire_hand/set_redu_ratio", &inspire_hand::hand_serial::setREDU_RATIOCallback, &hand);
    ros::ServiceServer set_clear_error_service = nh.advertiseService("inspire_hand/set_clear_error", &inspire_hand::hand_serial::setCLEAR_ERRORCallback, &hand);
    ros::ServiceServer set_save_flash_service = nh.advertiseService("inspire_hand/set_save_flash", &inspire_hand::hand_serial::setSAVE_FLASHCallback, &hand);
    ros::ServiceServer set_reset_para_service = nh.advertiseService("inspire_hand/set_reset_para", &inspire_hand::hand_serial::setRESET_PARACallback, &hand);
    ros::ServiceServer set_force_clbr_service = nh.advertiseService("inspire_hand/set_force_clb", &inspire_hand::hand_serial::setFORCE_CLBCallback, &hand);
    ros::ServiceServer set_gesture_no_service = nh.advertiseService("inspire_hand/set_gesture_no", &inspire_hand::hand_serial::setGESTURE_NOCallback, &hand);
    ros::ServiceServer set_current_limit_service = nh.advertiseService("inspire_hand/set_current_limit", &inspire_hand::hand_serial::setCURRENT_LIMITCallback, &hand);
    ros::ServiceServer set_default_speed_service = nh.advertiseService("inspire_hand/set_default_speed", &inspire_hand::hand_serial::setDEFAULT_SPEEDCallback, &hand);
    ros::ServiceServer set_default_force_service = nh.advertiseService("inspire_hand/set_default_force", &inspire_hand::hand_serial::setDEFAULT_FORCECallback, &hand);
    ros::ServiceServer set_user_def_angle_service = nh.advertiseService("inspire_hand/set_user_def_angle", &inspire_hand::hand_serial::setUSER_DEF_ANGLECallback, &hand);
    ros::ServiceServer set_pos_service = nh.advertiseService("inspire_hand/set_pos", &inspire_hand::hand_serial::setPOSCallback, &hand);
    ros::ServiceServer set_angle_service = nh.advertiseService("inspire_hand/set_angle", &inspire_hand::hand_serial::setANGLECallback, &hand);
    ros::ServiceServer set_force_service = nh.advertiseService("inspire_hand/set_force", &inspire_hand::hand_serial::setFORCECallback, &hand);
    ros::ServiceServer set_speed_service = nh.advertiseService("inspire_hand/set_speed", &inspire_hand::hand_serial::setSPEEDCallback, &hand);
    ros::ServiceServer get_pos_act_service = nh.advertiseService("inspire_hand/get_pos_act", &inspire_hand::hand_serial::getPOS_ACTCallback, &hand);
    ros::ServiceServer get_angle_act_service = nh.advertiseService("inspire_hand/get_angle_act", &inspire_hand::hand_serial::getANGLE_ACTCallback, &hand);
    ros::ServiceServer get_force_act_service = nh.advertiseService("inspire_hand/get_force_act", &inspire_hand::hand_serial::getFORCE_ACTCallback, &hand);
    ros::ServiceServer get_current_service = nh.advertiseService("inspire_hand/get_current", &inspire_hand::hand_serial::getCURRENTCallback, &hand);
    ros::ServiceServer get_error_service = nh.advertiseService("inspire_hand/get_error", &inspire_hand::hand_serial::getERRORCallback, &hand);
    ros::ServiceServer get_status_service = nh.advertiseService("inspire_hand/get_status", &inspire_hand::hand_serial::getSTATUSCallback, &hand);
    ros::ServiceServer get_temp_service = nh.advertiseService("inspire_hand/get_temp", &inspire_hand::hand_serial::getTEMPCallback, &hand);
    ros::ServiceServer get_pos_set_service = nh.advertiseService("inspire_hand/get_pos_set", &inspire_hand::hand_serial::getPOS_SETCallback, &hand);
    ros::ServiceServer get_angle_set_service = nh.advertiseService("inspire_hand/get_angle_set", &inspire_hand::hand_serial::getANGLE_SETCallback, &hand);
    ros::ServiceServer get_force_set_service = nh.advertiseService("inspire_hand/get_force_set", &inspire_hand::hand_serial::getFORCE_SETCallback, &hand);

    //ros::ServiceServer set_param_service = nh.advertiseService("inspire_hand/set_param", &inspire_hand::hand_serial::setParamCallback, &hand);

    //ros::ServiceServer get_state_service = nh.advertiseService("inspire_hand/get_state", &inspire_hand::hand_serial::getStateCallback, &hand);

    //ros::ServiceServer get_setangle_service = nh.advertiseService("inspire_hand/get_setangle", &inspire_hand::hand_serial::getSetangleCallback, &hand);

    //ros::ServiceServer get_curangle_service = nh.advertiseService("inspire_hand/get_curangle", &inspire_hand::hand_serial::getCurangleCallback, &hand);


    //ros::ServiceServer set_power_service = nh.advertiseService("inspire_hand/set_power", &inspire_hand::hand_serial::setPowerCallback, &hand);

    //ros::ServiceServer get_power_service = nh.advertiseService("inspire_hand/get_power", &inspire_hand::hand_serial::getPowerCallback, &hand);

    //ros::ServiceServer set_speed_service = nh.advertiseService("inspire_hand/set_speed", &inspire_hand::hand_serial::setSpeedCallback, &hand);

    //ros::ServiceServer get_speed_service = nh.advertiseService("inspire_hand/get_speed", &inspire_hand::hand_serial::getSpeedCallback, &hand);

    //ros::ServiceServer set_angle_service = nh.advertiseService("inspire_hand/set_angle", &inspire_hand::hand_serial::setAngleCallback, &hand);

    //ros::ServiceServer set_id_service = nh.advertiseService("inspire_hand/set_id", &inspire_hand::hand_serial::setIDCallback, &hand);

    //ros::Timer timer = nh.createTimer(ros::Duration(hand.TF_UPDATE_PERIOD), &inspire_hand::hand_serial::timerCallback, &hand);

    //hand.joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);



    //topic
    //topic
    //ros::Publisher chatter_pub = nh.advertise<std_msgs::Int32MultiArray>("chatter", 1000);
    //ros::Subscriber sub = nh.subscribe("chatter", 1000, arrayCallback);
    //ros::Rate loop_rate(1000);
    /*
        while (ros::ok())
        {
                std_msgs::Int32MultiArray array;
                //Clear array
                array.data.clear();


                for (int i = 0; i <2; i++)
                {
                        //assign array a random number between 0 and 255.
                        array.data.push_back(rand() % 255);

                }
                //Publish array
                chatter_pub.publish(array);
                //Let the world know
                ROS_INFO("I published something!");
                //Do this.
                ros::spinOnce();

                loop_rate.sleep();
        }*/

    ros::spin();

    return(EXIT_SUCCESS);
}
