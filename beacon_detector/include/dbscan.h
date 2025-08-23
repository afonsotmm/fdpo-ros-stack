//
//  Created by afonso on 23/08/2025
//

#pragma once

#include <vector>
#include <deque>

class Point{

    public:
        double x, y;

        bool visited;
        int class_; // Noise, Unassigned, Core, Border
        std::vector<Point*> neighbours;
        
        Point(double x, double y, int class_);

};

struct Cluster {

    std::vector<Point*> points;
    double centroid_x, centroid_y;

};

class DBSCAN {

    public:
        // Parameters
        double eps, eps2;
        int minPoints;

        // Point's Possible Classes
        enum{

            Unassigned = 0,
            Noise,
            Border,
            Core

        };
        
        std::vector<Point> points;
        std::vector<Cluster> clusters;

        DBSCAN(std::vector<Point>& points, double eps, int minPoints);
        void pointNeighbours(Point& point_to_check);
        void clusteringAlgorithm();

};