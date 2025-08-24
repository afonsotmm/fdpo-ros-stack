//
//  Created by afonso on 23/08/2025
//

#pragma once

#include <vector>
#include <deque>

struct Pose {

    double x, y;

};

class Point{

    public:
        Pose pose;
        
        bool visited;
        int class_; // Noise, Unassigned, Core, Border
        std::vector<Point*> neighbours;

        Point(double x, double y);

};

namespace point_ns{

    // Point's Possible Classes
    enum{

        Unassigned = 0,
        Noise,
        Border,
        Core

    }; 

}

class Cluster {

    public:
        std::vector<Point*> points;
        Pose centroid;

        Cluster();
        void computeCentroid();
        
};

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