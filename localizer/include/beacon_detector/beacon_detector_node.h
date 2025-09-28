//
//  Created by afonso on 17/08/2025
//

#pragma once

#include "general/pose.h"
#include "general/point.h"
#include "general/cluster.h"
#include "general/beacon.h"
#include "beacon_detector/dbscan.h"

#include "localizer/Pose.h"
#include "localizer/Cluster.h"
#include "localizer/BeaconMatch.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h> 
#include <xmlrpcpp/XmlRpcValue.h>
#include <string>
#include <unordered_map>
#include <tuple>
#include <cmath>

#define RVIZ_VISUALIZATION

class BeaconDetector {

    public:
        BeaconDetector(ros::NodeHandle& nh);
        virtual ~BeaconDetector();

    private:
        ros::NodeHandle& nh;

        std::vector<Cluster> clusters;
        std::vector<Beacon> beacons_globalFrame;
        std::vector<Beacon> beacons_robotFrame;
        std::vector<Pose> clustersCentroids_robotFrame;
        std::unordered_map<std::string, int> beaconToCluster; // beacon_name -> cluster_index 

        double maxMatchDist;
        double centroidOffset;
        void loadBeaconsFromParams();
        void updateRobotFrameBeacons(const ros::Time& stamp);
        void matchBeaconsToClusters();

        std::string target_frame;
		tf2_ros::Buffer *tf_buffer;
		tf2_ros::TransformListener *tf_listener;
        sensor_msgs::PointCloud2 pointCloud;

        ros::Subscriber sensorDataSub;
        void processSensorData(const sensor_msgs::LaserScan::ConstPtr& scan);
        void pointCloud2XY(const sensor_msgs::PointCloud2& cloud, std::vector<Point>& out);
        void dataClustering(std::vector<Point>& dataPoints);

        ros::Publisher beaconEstimation_pub;
        void publishBeaconsEstimation(const std_msgs::Header& header);

        ros::Publisher markers_pub_;
        void publishClusters(const std::vector<Cluster>& clusters);
        void publishBeaconAssociations();
        ros::Publisher beacons_map_pub_; 
        void publishBeaconsInMap();    
};






