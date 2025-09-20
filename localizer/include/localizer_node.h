#pragma once

#include <ros/ros.h>
#include "beacon_detector/Pose.h"
#include "beacon_detector/Cluster.h"
#include "beacon_detector/BeaconMatch.h" 

class LocalizerNode {

    public:
        explicit LocalizerNode(ros::NodeHandle& nh);

    private:
        ros::Subscriber sub_;
        void echoCallback(const beacon_detector::BeaconMatch::ConstPtr& msg);

};


