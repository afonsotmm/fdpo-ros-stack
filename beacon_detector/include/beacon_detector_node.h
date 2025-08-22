#ifndef BEACON_DETECTOR_NODE_H
#define BEACON_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>  
#include <string>


class BeaconDetector {
    public:
        BeaconDetector(ros::NodeHandle& nh);
        virtual ~BeaconDetector();

    private:
        ros::NodeHandle& nh;

		tf2_ros::Buffer *tf_buffer;
		tf2_ros::TransformListener *tf_listener;
        
        sensor_msgs::PointCloud2 pointCloud;
        std::string target_frame;

        ros::Subscriber sensorDataSub;
        void processSensorData(sensor_msgs::LaserScanConstPtr scan);
};



#endif // BEACON_DETECTOR_NODE_H



