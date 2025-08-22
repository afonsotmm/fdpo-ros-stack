#include "beacon_detector_node.h"

BeaconDetector::BeaconDetector(ros::NodeHandle& nh) : nh(nh) {

	tf_buffer = new tf2_ros::Buffer();
	tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    target_frame = "map";

    sensorDataSub = nh.subscribe("/base_scan", 10, &BeaconDetector::processSensorData, this);

    ROS_INFO("BeaconDetector instance created.");
}

BeaconDetector::~BeaconDetector() {

	delete tf_buffer;
	delete tf_listener;

}

void BeaconDetector::processSensorData(const sensor_msgs::LaserScan::ConstPtr scan) {

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

}