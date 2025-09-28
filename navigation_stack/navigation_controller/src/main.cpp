#include "navigation_controller_node.h"

#include <ros/ros.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "navigation_controller_node");
    ros::NodeHandle nh("~");

    NavigationController navigation(nh);
    ROS_INFO("Navigation Controller initialization...");
    
    ros::spin();
    
    return 0;
}