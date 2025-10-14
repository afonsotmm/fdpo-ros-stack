#include "pico_driver_node.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "pico_driver_node");
  ros::NodeHandle nh;

  PiPicoDriver driver(nh);  

  ros::spin(); 

  return 0;
}