/*********************************************************************************************//**
* hand_control.h
*

* September 2015
* Author:Hanson Du

* *********************************************************************************************/



#ifndef HAND_CONTROL_H
#define HAND_CONTROL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//Service headers
#include <inspire_hand/set_id.h>
#include <inspire_hand/set_redu_ratio.h>
#include <inspire_hand/set_clear_error.h>
#include <inspire_hand/set_save_flash.h>
#include <inspire_hand/set_reset_para.h>
#include <inspire_hand/set_force_clb.h>
#include <inspire_hand/set_gesture_no.h>
#include <inspire_hand/set_current_limit.h>
#include <inspire_hand/set_default_speed.h>
#include <inspire_hand/set_default_force.h>
#include <inspire_hand/set_user_def_angle.h>
#include <inspire_hand/set_pos.h>
#include <inspire_hand/set_angle.h>
#include <inspire_hand/set_force.h>
#include <inspire_hand/set_speed.h>
#include <inspire_hand/get_pos_act.h>
#include <inspire_hand/get_angle_act.h>
#include <inspire_hand/get_force_act.h>
#include <inspire_hand/get_current.h>
#include <inspire_hand/get_error.h>
#include <inspire_hand/get_status.h>
#include <inspire_hand/get_temp.h>
#include <inspire_hand/get_pos_set.h>
#include <inspire_hand/get_angle_set.h>
#include <inspire_hand/get_force_set.h>


namespace inspire_hand
{

class hand_serial
{
public:

    hand_serial(ros::NodeHandle *nh);

    ~hand_serial();
    //设置函数的callback
    bool setIDCallback(inspire_hand::set_id::Request &req,
                       inspire_hand::set_id::Response &res);

    bool setREDU_RATIOCallback(inspire_hand::set_redu_ratio::Request &req,
                               inspire_hand::set_redu_ratio::Response &res);

    bool setCLEAR_ERRORCallback(inspire_hand::set_clear_error::Request &req,
                                inspire_hand::set_clear_error::Response &res);

    bool setSAVE_FLASHCallback(inspire_hand::set_save_flash::Request &req,
                               inspire_hand::set_save_flash::Response &res);

    bool setRESET_PARACallback(inspire_hand::set_reset_para::Request &req,
                               inspire_hand::set_reset_para::Response &res);

    bool setFORCE_CLBCallback(inspire_hand::set_force_clb::Request &req,
                              inspire_hand::set_force_clb::Response &res);

    bool setGESTURE_NOCallback(inspire_hand::set_gesture_no::Request &req,
                               inspire_hand::set_gesture_no::Response &res);

    bool setCURRENT_LIMITCallback(inspire_hand::set_current_limit::Request &req,
                                  inspire_hand::set_current_limit::Response &res);

    bool setDEFAULT_SPEEDCallback(inspire_hand::set_default_speed::Request &req,
                                  inspire_hand::set_default_speed::Response &res);

    bool setDEFAULT_FORCECallback(inspire_hand::set_default_force::Request &req,
                                  inspire_hand::set_default_force::Response &res);

    bool setUSER_DEF_ANGLECallback(inspire_hand::set_user_def_angle::Request &req,
                                   inspire_hand::set_user_def_angle::Response &res);

    bool setPOSCallback(inspire_hand::set_pos::Request &req,
                        inspire_hand::set_pos::Response &res);

    bool setANGLECallback(inspire_hand::set_angle::Request &req,
                          inspire_hand::set_angle::Response &res);

    bool setFORCECallback(inspire_hand::set_force::Request &req,
                          inspire_hand::set_force::Response &res);

    bool setSPEEDCallback(inspire_hand::set_speed::Request &req,
                          inspire_hand::set_speed::Response &res);

    bool getPOS_ACTCallback(inspire_hand::get_pos_act::Request &req,
                            inspire_hand::get_pos_act::Response &res);

    bool getANGLE_ACTCallback(inspire_hand::get_angle_act::Request &req,
                              inspire_hand::get_angle_act::Response &res);

    bool getFORCE_ACTCallback(inspire_hand::get_force_act::Request &req,
                              inspire_hand::get_force_act::Response &res);

    bool getCURRENTCallback(inspire_hand::get_current::Request &req,
                            inspire_hand::get_current::Response &res);

    bool getERRORCallback(inspire_hand::get_error::Request &req,
                          inspire_hand::get_error::Response &res);

    bool getSTATUSCallback(inspire_hand::get_status::Request &req,
                           inspire_hand::get_status::Response &res);

    bool getTEMPCallback(inspire_hand::get_temp::Request &req,
                         inspire_hand::get_temp::Response &res);

    bool getPOS_SETCallback(inspire_hand::get_pos_set::Request &req,
                            inspire_hand::get_pos_set::Response &res);

    bool getANGLE_SETCallback(inspire_hand::get_angle_set::Request &req,
                              inspire_hand::get_angle_set::Response &res);

    bool getFORCE_SETCallback(inspire_hand::get_force_set::Request &req,
                              inspire_hand::get_force_set::Response &res);

    //void timerCallback(const ros::TimerEvent &event);

    //关节参数发布
    //ros::Publisher joint_pub;

    //TF更新周期
    //static const float TF_UPDATE_PERIOD = 0.5;


private:

    //读取灵巧手六个自由度驱动器实际位置
    int start(serial::Serial *port);

    //设置灵巧手ID号
    bool setID(serial::Serial *port, int id);

    //设置灵巧手波特率
    bool setREDU_RATIO(serial::Serial *port, int redu_ratio);

    //灵巧手清除错误
    bool setCLEAR_ERROR(serial::Serial *port);

    //保存参数到FLASH
    bool setSAVE_FLASH(serial::Serial *port);

    //恢复出厂设置
    bool setRESET_PARA(serial::Serial *port);

    //力传感器校准
    bool setFORCE_CLB(serial::Serial *port);

    //设置灵巧手目标手势序列号
    bool setGESTURE_NO(serial::Serial *port, int gesture_no);

    //设置灵巧手六个自由度驱动器电流保护值
    bool setCURRENT_LIMIT(serial::Serial *port, int current0, int current1, int current2, int current3, int current4, int current5);

    //设置灵巧手六个自由度上电速度值
    bool setDEFAULT_SPEED(serial::Serial *port, int speed0, int speed1, int speed2, int speed3, int speed4, int speed5);

    //设置灵巧手六个自由度上电力控阈值
    bool setDEFAULT_FORCE(serial::Serial *port, int force0, int force1, int force2, int force3, int force4, int force5);

    //设置用户自定义手势角度值
    bool setUSER_DEF_ANGLE(serial::Serial *port, int angle0, int angle1, int angle2, int angle3, int angle4, int angle5, int k);

    //设置灵巧手六个自由度驱动器位置
    bool setPOS(serial::Serial *port, int pos0, int pos1, int pos2, int pos3, int pos4, int pos5);

    //设置灵巧手六个自由度角度
    bool setANGLE(serial::Serial *port, int angle0, int angle1, int angle2, int angle3, int angle4, int angle5);

    //设置六个自由度力控阈值
    bool setFORCE(serial::Serial *port, int force0, int force1, int force2, int force3, int force4, int force5);

    //设置灵巧手六个自由度速度
    bool setSPEED(serial::Serial *port, int speed0, int speed1, int speed2, int speed3, int speed4, int speed5);

    //读取灵巧手六个自由度驱动器实际位置
    void getPOS_ACT(serial::Serial *port);

    //读取灵巧手六个自由度实际角度
    void getANGLE_ACT(serial::Serial *port);

    //读取灵巧手六个自由度实际受力
    void getFORCE_ACT(serial::Serial *port);

    //读取灵巧手六个自由度驱动器实际电流值
    void getCURRENT(serial::Serial *port);

    //读取灵巧手六个自由度驱动器故障信息
    uint8_t getERROR(serial::Serial *port);

    //读取灵巧手六个自由度状态信息
    void getSTATUS(serial::Serial *port);

    //读取灵巧手六个自由度温度
    void getTEMP(serial::Serial *port);

    //读取灵巧手六个自由度驱动器设置位置
    void getPOS_SET(serial::Serial *port);

    //读取灵巧手六个自由度设置角度
    void getANGLE_SET(serial::Serial *port);

    //读取灵巧手六个自由度设置力控阈值
    void getFORCE_SET(serial::Serial *port);

    /** \brief Set periodic position reading by GET_STATE(0x95) command */
    //void getPeriodicPositionUpdate(serial::Serial *port, float update_frequency);

    /** \brief Function to determine checksum*/
    uint16_t CRC16(uint16_t crc, uint16_t data);

    /** \brief Conversion from 4 bytes to float*/
    float IEEE_754_to_float(uint8_t *raw);

    /** \brief Conversion from float to 4 bytes*/
    void float_to_IEEE_754(float position, unsigned int *output_array);

    //Launch params
    int hand_id_;
    std::string port_name_;
    int baudrate_;
    int test_flags;

    //hand state variables
    float act_position_;
    uint8_t hand_state_;
    float curpos_[6];
    float curangle_[6];
    float curforce_[6];
    float current_[6];
    float errorvalue_[6];
    float statusvalue_[6];
    float tempvalue_[6];
    float setpos_[6];
    float setangle_[6];
    float setforce_[6];
    //sensor_msgs::JointState hand_joint_state_;

    //Serial variables
    serial::Serial *com_port_;

    //Consts


    //static const double MIN_GRIPPER_VEL_LIMIT = 0;
    //static const double MAX_GRIPPER_VEL_LIMIT = 83;
    //static const double MIN_GRIPPER_ACC_LIMIT = 0;
    //static const double MAX_GRIPPER_ACC_LIMIT = 320;
    static const double WAIT_FOR_RESPONSE_INTERVAL = 0.5;
    static const double INPUT_BUFFER_SIZE = 64;
    //static const int    URDF_SCALE_FACTOR = 2000;

};
}

#endif
