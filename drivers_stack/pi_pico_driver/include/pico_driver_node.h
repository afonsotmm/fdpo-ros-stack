#pragma once

#include <ros/ros.h>

class PiPicoDriver {
    public:
        PiPicoDriver(ros::NodeHandle& nh_);

    private: 
        ros::NodeHandle& nh;
    
        ros::Subscriber velSub;
        void velCallBack(const geometry_msgs::Twist& msg);
};