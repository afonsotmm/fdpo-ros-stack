#include "pico_driver_node.h"


PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) : nh(nh_) {
    velSub = nh.subscribe("/cmd_vel", 10, &PiPicoDriver::velCallBack, this);
    pickBoxSub = nh.subscribe("/pick_box", 10, &PiPicoDriver::PickBoxCallBack, this);
    posePub = nh.advertise<geometry_msgs::Pose2D>("/odom", 10);
    tofPub = nh.advertise<std_msgs::Float32>("/tof_measure", 10);
    
}


void PiPicoDriver::PickBoxCallBack(const std_msgs::Bool::ConstPtr& msg) {
    ROS_INFO_STREAM("Received pick_box msg: " << msg->data);

    iman = msg->data;
}

void PiPicoDriver::velCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO_STREAM("Received cmd_vel: linear=" << msg->linear.x << ", angular=" << msg->angular.z);

    v_d = msg->linear.x;
    w_d = msg->angular.z;

}

void PiPicoDriver::PubPose(){
    
    geometry_msgs::Pose2D XYTheta;
    XYTheta.x = position.x;
    XYTheta.y = position.y;
    XYTheta.theta = position.theta;
    posePub.publish(XYTheta);

}

void PiPicoDriver::PubTof(){
    
    std_msgs::Float32 tofValue;
    tofValue.data = Tof;
    tofPub.publish(tofValue);
    
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pico_driver_node");
  ros::NodeHandle nh;

  PiPicoDriver driver(nh);  // cria seu objeto controlador

  ros::spin();  // espera callbacks

  return 0;
}