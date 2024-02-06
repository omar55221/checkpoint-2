#include "robot_gui/robot_gui_node.h"
#include <ros/ros.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#define CVUI_IMPLEMENTATION
#include <opencv2/opencv.hpp>
#include "cvui/cvui.h"

const std::string WINDOW_NAME = "Robot GUI";
const int WINDOW_WIDTH = 400;
const int WINDOW_HEIGHT = 600;

RobotGuiNode::RobotGuiNode() {
    robot_info_sub_ = nh_.subscribe("robot_info", 10, &RobotGuiNode::robotInfoCallback, this);
    odom_sub_ = nh_.subscribe("odom", 10, &RobotGuiNode::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    get_distance_client_ = nh_.serviceClient<std_srvs::Trigger>("get_distance");

    cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT);
    cvui::init(WINDOW_NAME);
}

void RobotGuiNode::run() {
    cv::Mat frame = cv::Mat(cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT), CV_8UC3);
    geometry_msgs::Twist continuousTwist; // Initialize with zero velocities
    continuousTwist.linear.x = 0.0;
    continuousTwist.angular.z = 0.0;

    while (ros::ok()) {
        frame = cv::Scalar(255, 0, 0); // Change frame background to blue

        // Display robot information
        cvui::printf(frame, 10, 20, 0.4, 0xffffff, "Description: %s", robot_info_.data_field_01.c_str());
        cvui::printf(frame, 10, 35, 0.4, 0xffffff, "Serial Number: %s", robot_info_.data_field_02.c_str());
        cvui::printf(frame, 10, 50, 0.4, 0xffffff, "IP Address: %s", robot_info_.data_field_03.c_str());
        cvui::printf(frame, 10, 65, 0.4, 0xffffff, "Firmware Version: %s", robot_info_.data_field_04.c_str());

        // Display velocities all the time, updating as the robot moves
        cvui::printf(frame, 10, 80, 0.4, 0xffffff, "Linear Vel: %.2f m/s", continuousTwist.linear.x);
        cvui::printf(frame, 10, 95, 0.4, 0xffffff, "Angular Vel: %.2f rad/s", continuousTwist.angular.z);

        // Display odometry information
        cvui::printf(frame, 10, 110, 0.4, 0xffffff, "Position: x=%.2f, y=%.2f", odom_.pose.pose.position.x, odom_.pose.pose.position.y);

        // Implement teleoperation buttons for all directions and stop
        if (cvui::button(frame, 10, 220, 100, 30, "Forward")) {
            continuousTwist.linear.x = 0.5; continuousTwist.angular.z = 0;
        }
        if (cvui::button(frame, 10, 260, 100, 30, "Backward")) {
            continuousTwist.linear.x = -0.5; continuousTwist.angular.z = 0;
        }
        if (cvui::button(frame, 10, 300, 100, 30, "Left")) {
            continuousTwist.angular.z = 0.5; continuousTwist.linear.x = 0;
        }
        if (cvui::button(frame, 120, 300, 100, 30, "Right")) {
            continuousTwist.angular.z = -0.5; continuousTwist.linear.x = 0;
        }
        if (cvui::button(frame, 60, 340, 100, 30, "Stop")) {
            continuousTwist.linear.x = 0; continuousTwist.angular.z = 0;
        }

        cmd_vel_pub_.publish(continuousTwist); // Publish the twist message continuously

        // Distance traveled service button, placed below the velocities
        static std::string distanceMessage = "";
        if (cvui::button(frame, 10, 135, 160, 30, "Get Distance")) {
            std_srvs::Trigger srv;
            if (get_distance_client_.call(srv)) {
                distanceMessage = "Distance: " + srv.response.message;
            } else {
                distanceMessage = "Failed to call service get_distance";
            }
        }
        cvui::printf(frame, 10, 170, 0.4, 0xffffff, "%s", distanceMessage.c_str());

        cvui::imshow(WINDOW_NAME, frame);
        ros::spinOnce();
        cv::waitKey(20); // Keep the GUI responsive
    }
}

void RobotGuiNode::robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg) {
    robot_info_ = *msg;
}

void RobotGuiNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_ = *msg;
}

void RobotGuiNode::publishCmdVel(double linear, double angular) {
    geometry_msgs::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    cmd_vel_pub_.publish(twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_gui_node");
    RobotGuiNode robotGuiNode;
    robotGuiNode.run();
    return 0;
}
