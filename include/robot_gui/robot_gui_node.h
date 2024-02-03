#ifndef ROBOT_GUI_ROBOT_GUI_NODE_H
#define ROBOT_GUI_ROBOT_GUI_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"

class RobotGuiNode {
public:
    RobotGuiNode();
    void run();

private:
    // Callbacks for ROS topics
    void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // Method to publish velocity commands
    void publishCmdVel(double linear, double angular);

    // Method to call the distance service
    void getDistance();

    // ROS communication members
    ros::NodeHandle nh_;
    ros::Subscriber robot_info_sub_; // Subscriber to robot_info topic
    ros::Subscriber odom_sub_; // Subscriber to odom topic
    ros::Publisher cmd_vel_pub_; // Publisher to cmd_vel topic
    ros::ServiceClient get_distance_client_; // Client for get_distance service

    // Data storage for received messages
    robotinfo_msgs::RobotInfo10Fields robot_info_; // Stores latest robot info
    nav_msgs::Odometry odom_; // Stores latest odometry information
};

#endif  // ROBOT_GUI_ROBOT_GUI_NODE_H
