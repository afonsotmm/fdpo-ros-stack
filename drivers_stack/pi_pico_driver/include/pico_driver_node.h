#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>



struct Pose {

    double x, y, theta;

};

class PiPicoDriver {
    public:
        PiPicoDriver(ros::NodeHandle& nh_);
        ~PiPicoDriver();

    private: 
        ros::NodeHandle& nh;
    
        ros::Subscriber velSub;
        ros::Subscriber pickBoxSub;
        ros::Publisher posePub;
        ros::Publisher tofPub;
        void PickBoxCallBack(const std_msgs::Bool::ConstPtr& msg);
        void velCallBack(const geometry_msgs::Twist::ConstPtr& msg);
        void PubPose();
        void PubTof();
        void readSerialLoop();
        void startSerial(const std::string& port, int baud);
        void DecodeMsg(const std::string& msg);
        std::string sendCommandAndWait(const std::string& cmd, int timeout_ms);

        Pose position;
        bool Tof;
        bool iman;
        double v_d, w_d;
        int serial_fd_;

};