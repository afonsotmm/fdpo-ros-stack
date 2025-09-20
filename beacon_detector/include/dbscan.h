//
//  Created by afonso on 23/08/2025
//

#pragma once

#include "point.h"
#include "cluster.h"

#include <vector>
#include <deque>

class DBSCAN {

    public:
        // Parameters
        double eps, eps2;
        int minPoints;
        
        std::vector<Point> points;
        std::vector<Cluster> clusters;

        DBSCAN(std::vector<Point>& points, double eps, int minPoints);
        void pointNeighbours(Point& point_to_check);
        void clusteringAlgorithm();
        void restart();

};