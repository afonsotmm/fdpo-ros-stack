#include "localizer_node.h"

LocalizerNode::LocalizerNode(ros::NodeHandle& nh) {

    sub_ = nh.subscribe("/beacon_stimation", 10, &LocalizerNode::echoCallback, this);

}

void LocalizerNode::echoCallback(const beacon_detector::BeaconMatch::ConstPtr& msg) {

    const auto& clusters = msg->clusters;
    ROS_INFO("BeaconMatch: %zu clusters recebidos", clusters.size());

    for (size_t i = 0; i < clusters.size(); ++i) {

        const auto& c = clusters[i];

        ROS_INFO("  [%zu] name='%s'  num_points=%u  centroid=(%.3f, %.3f)",
                i,
                c.beacon_match_name.c_str(),
                c.num_points,
                c.centroid.x,
                c.centroid.y);

        for (size_t j = 0; j < c.points.size(); ++j) {

            const auto& p = c.points[j];
            ROS_INFO("       pt[%zu] = (%.3f, %.3f)", j, p.x, p.y);
        
        }
    }
}


