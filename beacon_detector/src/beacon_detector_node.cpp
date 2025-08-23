#include "beacon_detector_node.h"

BeaconDetector::BeaconDetector(ros::NodeHandle& nh) : nh(nh) {

	tf_buffer = new tf2_ros::Buffer();
	tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    target_frame = "map";

    sensorDataSub = nh.subscribe("/base_scan", 10, &BeaconDetector::processSensorData, this);
    markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("dbscan_markers", 1);

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
    nh.param<double>("eps", eps, 0.07);
    
    int minPoints;
    nh.param<int>("minPoints", minPoints, 3);

    DBSCAN dbscan(dataPoints, eps, minPoints);
    dbscan.clusteringAlgorithm();

    #ifdef RVIZ_VISUALIZATION
    publishClusters(dbscan.clusters);
    #endif

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


// ---------------------------------------------------------------------------------------
//                               RViz Visualization 
// ---------------------------------------------------------------------------------------
namespace {
// paleta simples e estável por id
    inline void colorFromId(std_msgs::ColorRGBA& c, std::size_t id) {
        static const float palette[][3] = {
            {0.90f,0.10f,0.10f}, {0.10f,0.60f,0.95f}, {0.10f,0.75f,0.20f},
            {0.95f,0.65f,0.10f}, {0.60f,0.10f,0.80f}, {0.10f,0.85f,0.80f},
            {0.85f,0.30f,0.10f}, {0.55f,0.55f,0.10f}
    };
        const auto& rgb = palette[id % (sizeof(palette)/sizeof(palette[0]))];
        c.r = rgb[0]; c.g = rgb[1]; c.b = rgb[2]; c.a = 1.0f;
    }
} // anon ns

void BeaconDetector::publishClusters(const std::vector<Cluster>& clusters) {
  visualization_msgs::MarkerArray arr;

  {
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(clear);
  }

  const ros::Time now = ros::Time::now();

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const auto& cl = clusters[i];

    // 1) Pontos do cluster
    visualization_msgs::Marker pts;
    pts.header.frame_id = target_frame;
    pts.header.stamp = now;
    pts.ns = "dbscan_points";
    pts.id = static_cast<int>(i);
    pts.type = visualization_msgs::Marker::POINTS;
    pts.action = visualization_msgs::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = 0.05; // diâmetro dos pontos (m)
    pts.scale.y = 0.05;

    colorFromId(pts.color, i);

    pts.lifetime = ros::Duration(0.2);

    pts.points.reserve(cl.points.size());
    for (auto* p : cl.points) {
      geometry_msgs::Point gp;
      gp.x = p->x; gp.y = p->y; gp.z = 0.0;
      pts.points.push_back(gp);
    }
    arr.markers.push_back(pts);

    // 2) Centroide (esfera branca)
    visualization_msgs::Marker centroid;
    centroid.header = pts.header;
    centroid.ns = "dbscan_centroid";
    centroid.id = static_cast<int>(i);
    centroid.type = visualization_msgs::Marker::SPHERE;
    centroid.action = visualization_msgs::Marker::ADD;
    centroid.pose.position.x = cl.centroid_x;
    centroid.pose.position.y = cl.centroid_y;
    centroid.pose.position.z = 0.0;
    centroid.pose.orientation.w = 1.0;
    centroid.scale.x = centroid.scale.y = centroid.scale.z = 0.12;
    centroid.color.r = 1.0; centroid.color.g = 1.0; centroid.color.b = 1.0; centroid.color.a = 1.0;
    centroid.lifetime = ros::Duration(0.2);
    arr.markers.push_back(centroid);

    // 3) Etiqueta do cluster (opcional)
    visualization_msgs::Marker label;
    label.header = pts.header;
    label.ns = "dbscan_label";
    label.id = static_cast<int>(i);
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.position.x = cl.centroid_x;
    label.pose.position.y = cl.centroid_y;
    label.pose.position.z = 0.25;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.18;  // altura do texto (m)
    colorFromId(label.color, i);
    label.lifetime = ros::Duration(0.2);
    label.text = std::to_string(i);
    arr.markers.push_back(label);
  }

  markers_pub_.publish(arr);
}


