#include "dbscan.h"

// ---------------------------------------------------------------------------------------
//                               Point Class 
// ---------------------------------------------------------------------------------------

Point::Point(double x, double y, int class_): x(x), y(y), class_(class_), visited(false) {}

// ---------------------------------------------------------------------------------------
//                               DBSCAN Class 
// ---------------------------------------------------------------------------------------
DBSCAN::DBSCAN(std::vector<Point>& points, double eps, int minPoints): points(points), eps(eps), eps2(eps*eps), minPoints(minPoints) {}

void DBSCAN::pointNeighbours(Point& point_to_check) {

    for(auto& point: points) {

        double dx = point_to_check.x - point.x;
        double dy = point_to_check.y - point.y;

        double eucl_dist = dx*dx + dy*dy;
        
        if(eucl_dist <= eps2) 
            point_to_check.neighbours.push_back(&point);

    }

}

void DBSCAN::clusteringAlgorithm() { 

    for(auto& point: points) {

        if(point.visited) continue;
        point.visited = true;
        
        pointNeighbours(point);

        if(point.neighbours.size() >= minPoints) {
            // it's a Core point
            point.class_ = Core;

            Cluster cluster_aux;
            cluster_aux.points.push_back(&point);

            std::deque<Point*> members(point.neighbours.begin(), point.neighbours.end());
            
            while(!members.empty()) {

                Point* curr_point = members.front();
                members.pop_front();

                if(!curr_point->visited) {

                    curr_point->visited = true;
                    pointNeighbours(*curr_point);

                    if(curr_point->neighbours.size() >= minPoints) {

                        curr_point->class_ = Core;
                        members.insert(members.end(), curr_point->neighbours.begin(), curr_point->neighbours.end());

                    }

                    else curr_point->class_ = Border;

                }

                if(curr_point->class_ == Unassigned || curr_point->class_ == Noise) {

                    cluster_aux.points.push_back(curr_point);

                }

            }

            clusters.push_back(cluster_aux);

        }

        else point.class_ = Noise;

    }

}