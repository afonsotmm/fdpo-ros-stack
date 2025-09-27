#pragma once

#include "pose.h"
#include "point.h"

#include <vector>

class Cluster {

    public:
        std::vector<Pose> points;
        Pose centroid;

        Cluster();
        void computeCentroid();
        
};