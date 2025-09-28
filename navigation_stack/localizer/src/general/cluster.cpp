#include "general/cluster.h"

// ---------------------------------------------------------------------------------------
//                               Cluster Class 
// ---------------------------------------------------------------------------------------

Cluster::Cluster(): centroid{-999.0, -999.0} {}


void Cluster::computeCentroid() {

    if(points.empty()) return;

    double sum_x = 0.0, sum_y = 0.0;
    std::size_t numPoints = points.size();

    for(auto& point: points) {

        sum_x += point.x;
        sum_y += point.y; 

    }

    centroid.x = sum_x / static_cast<double>(numPoints);
    centroid.y = sum_y / static_cast<double>(numPoints);

}