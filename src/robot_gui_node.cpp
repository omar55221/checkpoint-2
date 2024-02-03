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
    
    while (ros::ok()) {
        frame = cv::Scalar(49, 52, 49); // Dark background

        // Display robot information
        cvui::printf(frame, 10, 20, 0.4, 0xffffff, "Robot Description: %s", robot_info_.data_field_01.c_str());
        cvui::printf(frame, 10, 40, 0.4, 0xffffff, "Serial Number: %s", robot_info_.data_field_02.c_str());
        // Add more fields as needed...

        // Display odometry information
        cvui::printf(frame, 10, 160, 0.4, 0xffffff, "Position: x=%.2f, y=%.2f", odom_.pose.pose.position.x, odom_.pose.pose.position.y);
        cvui::printf(frame, 10, 180, 0.4, 0xffffff, "Orientation: z=%.2f, w=%.2f", odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);

        // Teleoperation buttons
        if (cvui::button(frame, 10, 220, 100, 30, "Forward")) {
            publishCmdVel(0.5, 0.0);
        }
        if (cvui::button(frame, 10, 260, 100, 30, "Backward")) {
            publishCmdVel(-0.5, 0.0);
        }
        if (cvui::button(frame, 10, 300, 100, 30, "Left")) {
            publishCmdVel(0.0, 0.5);
        }
        if (cvui::button(frame, 120, 300, 100, 30, "Right")) {
            publishCmdVel(0.0, -0.5);
        }
        if (cvui::button(frame, 60, 340, 100, 30, "Stop")) {
            publishCmdVel(0.0, 0.0);
        }

        // Distance traveled service button
        if (cvui::button(frame, 10, 380, 160, 30, "Get Distance")) {
            std_srvs::Trigger srv;
            if (get_distance_client_.call(srv)) {
                cvui::printf(frame, 10, 420, 0.4, 0xffffff, "Distance: %s", srv.response.message.c_str());
            } else {
                cvui::printf(frame, 10, 420, 0.4, 0xff0000, "Failed to call service get_distance");
            }
        }

        cvui::imshow(WINDOW_NAME, frame);

        ros::spinOnce();
        if (cv::waitKey(20) == 27) {
            break;
        }
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

void RobotGuiNode::getDistance() {
    // This method is now used for the button callback, so it's no longer necessary to log the distance here.
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_gui_node");
    RobotGuiNode robotGuiNode;
    robotGuiNode.run();
    return 0;
}
