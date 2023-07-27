#include "robot_info/agv_robot_info_class.h"

AGVRobotInfo::AGVRobotInfo() : RobotInfo() {
  maximum_payload = "100 Kg";
  hydraulic_system.hydraulic_oil_temperature = "45C";
  hydraulic_system.hydraulic_oil_tank_fill_level = "100%";
  hydraulic_system.hydraulic_oil_pressure = "250 bar";
}

void AGVRobotInfo::publish_data() {
  robotinfo_msgs::RobotInfo10Fields info;  
  info.data_field_01 = "robot_description: Mir100";
  info.data_field_02 = "serial_number: 567A359";
  info.data_field_03 = "ip_address: 169.254.5.180";
  info.data_field_04 = "firmware_version: 3.5.8";
  info.data_field_05 = "maximum_payload: " + maximum_payload;
  info.data_field_06 = "hydraulic_oil_temperature: " + hydraulic_system.getTemperature();
  info.data_field_07 = "hydraulic_oil_tank_fill_level: " + hydraulic_system.getFillLevel();
  info.data_field_08 = "hydraulic_oil_pressure: " + hydraulic_system.getPressure();

  while (ros::ok()) {
    robotInfo_Publisher.publish(info);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
