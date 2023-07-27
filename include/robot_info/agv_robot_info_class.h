#pragma once 
#ifndef AGV_ROBOT_INFO_H
#define AGV_ROBOT_INFO_H

#include "robot_info/robot_info.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "robot_info/hydraulic_system_monitor.h"

class AGVRobotInfo : public RobotInfo
{
    public:
        AGVRobotInfo();
        virtual void publish_data();
private:
  std::string maximum_payload;
  HydraulicSystemMonitor hydraulic_system;
};

#endif // AGV_ROBOT_INFO_H
