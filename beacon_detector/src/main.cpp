#include "beacon_detector_node.h"

#include <ros/ros.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "beacon_detector_node");
    ros::NodeHandle nh;

    BeaconDetector detector(nh);
    ROS_INFO("Beacon detector initialization...");
    
    ros::spin();
    
    return 0;
}

