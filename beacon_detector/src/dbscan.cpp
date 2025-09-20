#include "dbscan.h"

// ---------------------------------------------------------------------------------------
//                               DBSCAN Class 
// ---------------------------------------------------------------------------------------

DBSCAN::DBSCAN(std::vector<Point>& points, double eps, int minPoints): eps(eps), eps2(eps*eps), minPoints(minPoints), points(points) {}

void DBSCAN::pointNeighbours(Point& point_to_check) {

    point_to_check.neighbours.clear();  

    for(auto& point: points) {

        double dx = point_to_check.pose.x - point.pose.x;
        double dy = point_to_check.pose.y - point.pose.y;

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

        if(static_cast<int>(point.neighbours.size()) >= minPoints) {
            
            point.class_ = point::classes::Core;

            Cluster cluster_aux;
            cluster_aux.points.push_back(point.pose);

            std::deque<Point*> members(point.neighbours.begin(), point.neighbours.end());
            
            while(!members.empty()) {

                Point* curr_point = members.front();
                members.pop_front();

                int initial_class = curr_point->class_;

                if(!curr_point->visited) {

                    curr_point->visited = true;
                    pointNeighbours(*curr_point);

                    if(static_cast<int>(curr_point->neighbours.size()) >= minPoints) {

                        curr_point->class_ = point::classes::Core;
                        members.insert(members.end(), curr_point->neighbours.begin(), curr_point->neighbours.end());

                    }

                    else curr_point->class_ = point::classes::Border;

                }

                if(initial_class == point::classes::Unassigned || initial_class == point::classes::Noise) {

                    cluster_aux.points.push_back(curr_point->pose);

                }

            }

            cluster_aux.computeCentroid();
            clusters.push_back(cluster_aux);

        }

        else point.class_ = point::classes::Noise;

    }

}

void DBSCAN::restart() {

    points.clear();
    clusters.clear();

}