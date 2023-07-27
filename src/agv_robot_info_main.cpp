#include "robot_info/agv_robot_info_class.h"
#include <ros/ros.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "agv_robot_info_node");
  AGVRobotInfo agv_robot;
  agv_robot.publish_data();
  return 0;
}
