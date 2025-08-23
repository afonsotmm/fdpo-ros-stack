#include "dbscan.h"

// ---------------------------------------------------------------------------------------
//                               Point Class 
// ---------------------------------------------------------------------------------------

Point::Point(double x, double y): x(x), y(y), class_(point_ns::Unassigned), visited(false) {}



// ---------------------------------------------------------------------------------------
//                               Cluster Class 
// ---------------------------------------------------------------------------------------

Cluster::Cluster(): centroid_x(-999.0), centroid_y(-999.0) {}

void Cluster::computeCentroid() {

    if(points.empty()) return;

    double sum_x = 0.0, sum_y = 0.0;
    int numPoints = points.size();

    for(auto* point: points) {

        sum_x += point->x;
        sum_y += point->y; 

    }

    centroid_x = sum_x / numPoints;
    centroid_y = sum_y / numPoints;

}



// ---------------------------------------------------------------------------------------
//                               DBSCAN Class 
// ---------------------------------------------------------------------------------------

DBSCAN::DBSCAN(std::vector<Point>& points, double eps, int minPoints): points(points), eps(eps), eps2(eps*eps), minPoints(minPoints) {}

void DBSCAN::pointNeighbours(Point& point_to_check) {

    point_to_check.neighbours.clear();  

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
            point.class_ = point_ns::Core;

            Cluster cluster_aux;
            cluster_aux.points.push_back(&point);

            std::deque<Point*> members(point.neighbours.begin(), point.neighbours.end());
            
            while(!members.empty()) {

                Point* curr_point = members.front();
                members.pop_front();

                int initial_class = curr_point->class_;

                if(!curr_point->visited) {

                    curr_point->visited = true;
                    pointNeighbours(*curr_point);

                    if(curr_point->neighbours.size() >= minPoints) {

                        curr_point->class_ = point_ns::Core;
                        members.insert(members.end(), curr_point->neighbours.begin(), curr_point->neighbours.end());

                    }

                    else curr_point->class_ = point_ns::Border;

                }

                if(initial_class == point_ns::Unassigned || initial_class == point_ns::Noise) {

                    cluster_aux.points.push_back(curr_point);

                }

            }

            cluster_aux.computeCentroid();
            clusters.push_back(cluster_aux);

        }

        else point.class_ = point_ns::Noise;

    }

}

void DBSCAN::restart() {

    points.clear();
    clusters.clear();

}