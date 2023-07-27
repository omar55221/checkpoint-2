#pragma once 
#ifndef AGV_ROBOT_INFO_H
#define AGV_ROBOT_INFO_H

#include "robot_info/robot_info.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"

class AGVRobotInfo : public RobotInfo
{
    public:
        AGVRobotInfo();
        void publish_data() override;
    private:
        std::string maximum_payload;
};

#endif // AGV_ROBOT_INFO_H
