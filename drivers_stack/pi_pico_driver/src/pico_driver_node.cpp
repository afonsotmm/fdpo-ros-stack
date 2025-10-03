#include "pico_driver_node.h"

PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) {
    velSub = nh.subscribe("/cmd_vel", 10, &PiPicoDriver::velCallBack, this);
}