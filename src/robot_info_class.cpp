#include "robot_info/robot_info.h"
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"

RobotInfo::RobotInfo()
    : loop_rate(1)
{ 
  info_topic = "/robot_info";
  nh = ros::NodeHandle("~");
  robotInfo_Publisher = nh.advertise<robotinfo_msgs::RobotInfo10Fields>(info_topic, 10);
  robot_description = "Mir100";
  serial_number = "567A359";
  ip_address = "169.254.5.180";
  firmware_version = "3.5.8";
}

void RobotInfo::publish_data() {
  info_msg.data_field_01 = robot_description;
  info_msg.data_field_02 = serial_number;
  info_msg.data_field_03 = ip_address;
  info_msg.data_field_04 = firmware_version;
  info_msg.data_field_05 = "maximum_payload: 100 Kg";

  while (ros::ok()) {
    robotInfo_Publisher.publish(info_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

