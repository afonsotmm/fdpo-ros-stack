#include "beacon_detector/beacon_detector_node.h"

#include <ros/ros.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "beacon_detector_node");
    ros::NodeHandle nh;       // NodeHandle público para tópicos globais
    ros::NodeHandle nh_priv("~");  // NodeHandle privado para parâmetros

    BeaconDetector detector(nh, nh_priv);
    ROS_INFO("Beacon detector initialization...");
    
    ros::spin();
    
    return 0;
}

