#include "robot_info/agv_robot_info_class.h"

AGVRobotInfo::AGVRobotInfo() : RobotInfo() {
  maximum_payload = "100 Kg";
}

void AGVRobotInfo::publish_data() {
  robotinfo_msgs::RobotInfo10Fields info;  
  info.data_field_01 = "robot_description: Mir100";
  info.data_field_02 = "serial_number: 567A359";
  info.data_field_03 = "ip_address: 169.254.5.180";
  info.data_field_04 = "firmware_version: 3.5.8";
  info.data_field_05 = "maximum_payload: " + maximum_payload;

  while (ros::ok()) {
    robotInfo_Publisher.publish(info);  // Publish 'info' instead of 'info_msg'
    ros::spinOnce();
    loop_rate.sleep();
  }
}
