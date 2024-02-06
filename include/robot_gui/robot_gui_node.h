#ifndef ROBOT_GUI_ROBOT_GUI_NODE_H
#define ROBOT_GUI_ROBOT_GUI_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"

class RobotGuiNode {
public:
    
    RobotGuiNode(); // Constructor initializes ROS communication and GUI setup.
    void run(); // Main loop for processing GUI events and ROS callbacks.

private:
    // Callbacks for handling ROS topic updates
    void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // Method to publish velocity commands to control the robot
    void publishCmdVel(double linear, double angular);
    void getDistance();
    // ROS communication components
    ros::NodeHandle nh_; // Node handle for managing ROS communication
    ros::Subscriber robot_info_sub_; // Subscriber to robot_info topic
    ros::Subscriber odom_sub_; // Subscriber to odom topic
    ros::Publisher cmd_vel_pub_; // Publisher for robot velocity commands
    ros::ServiceClient get_distance_client_; // Client for the get_distance service

    // Storage for the latest received messages
    robotinfo_msgs::RobotInfo10Fields robot_info_; // Latest robot info message
    nav_msgs::Odometry odom_; // Latest odometry message

    // Members for managing continuous movement and GUI updates
    bool keepMoving; // Flag to control continuous movement
    geometry_msgs::Twist lastTwist; // Last velocity command for continuous movement
};

#endif  // ROBOT_GUI_ROBOT_GUI_NODE_H
