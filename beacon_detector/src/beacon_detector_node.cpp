#include "beacon_detector_node.h"

BeaconDetector::BeaconDetector(ros::NodeHandle& nh) : nh(nh) {

	tf_buffer = new tf2_ros::Buffer();
	tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    target_frame = "map";

    sensorDataSub = nh.subscribe("/base_scan", 10, &BeaconDetector::processSensorData, this);

    ROS_INFO("BeaconDetector instance created.");
}

BeaconDetector::~BeaconDetector() {

    delete tf_listener;
	delete tf_buffer;

}

void BeaconDetector::pointCloud2XY(const sensor_msgs::PointCloud2& cloud, std::vector<Point>& out)
{
    out.clear();

    sensor_msgs::PointCloud2ConstIterator<float> iterX(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterY(cloud, "y");

    while ((iterX != iterX.end()) && (iterY != iterY.end())) {

        Point point(*iterX, *iterY);
        out.push_back(point);

        ++iterX;
        ++iterY;

    }
}

void BeaconDetector::dataClustering(std::vector<Point>& dataPoints) {

    double eps;
    nh.param<double>("eps", eps, 0.9);
    
    int minPoints;
    nh.param<int>("minPoints", minPoints, 3);

    DBSCAN dbscan(dataPoints, eps, minPoints);
    dbscan.clusteringAlgorithm();

}

void BeaconDetector::processSensorData(const sensor_msgs::LaserScan::ConstPtr& scan) {

    laser_geometry::LaserProjection projector;

    projector.projectLaser(*scan, pointCloud, -1.0);

    try {

        sensor_msgs::PointCloud2 pointCloud_targetFrame;
        tf_buffer->transform(pointCloud, pointCloud_targetFrame, target_frame, ros::Duration(1.0));
        pointCloud = std::move(pointCloud_targetFrame);

    } catch (const tf2::TransformException& ex) {

        ROS_WARN_THROTTLE(1.0, "Failed to project laser data: %s", ex.what());
        return;
    }

    std::vector<Point> dataPoints;
    pointCloud2XY(pointCloud, dataPoints);
    dataClustering(dataPoints);
}



