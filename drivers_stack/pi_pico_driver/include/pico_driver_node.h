#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>

struct Pose {

    double x, y, theta;

};

class PiPicoDriver {
    public:
        PiPicoDriver(ros::NodeHandle& nh_);

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

        Pose position;
        float Tof;
        bool iman;
        double v_d, w_d;

};