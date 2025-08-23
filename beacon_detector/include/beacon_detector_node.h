#pragma once

#include "dbscan.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>  
#include <visualization_msgs/MarkerArray.h> 
#include <string>
#define RVIZ_VISUALIZATION

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

        ros::Publisher markers_pub_;
        void publishClusters(const std::vector<Cluster>& clusters);

        ros::Subscriber sensorDataSub;
        void processSensorData(const sensor_msgs::LaserScan::ConstPtr& scan);
        void pointCloud2XY(const sensor_msgs::PointCloud2& cloud, std::vector<Point>& out);
        void dataClustering(std::vector<Point>& dataPoints);
};






