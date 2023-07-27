#pragma once 
#ifndef ROBOT_INFO_H
#define ROBOT_INFO_H

#include <ros/ros.h>
#include <string>
//#include "robotinfo_msgs/RobotInfo10Fields.msg"
#include "robotinfo_msgs/RobotInfo10Fields.h"

class RobotInfo
{
    public:
        RobotInfo();
        virtual void publish_data();
    protected:
        ros::Publisher robotInfo_Publisher;
        ros::NodeHandle nh;
        robotinfo_msgs::RobotInfo10Fields info_msg;
        std::string robot_description;
        std::string serial_number;
        std::string ip_address;
        std::string firmware_version; 
        std::string info_topic;
        ros::Rate loop_rate;
              
};

#endif // ROBOT_INFO_H
