#pragma once

#include "pose.h"

#include <vector>

namespace point {

    // Point's Possible Classes
    namespace classes {
        
        enum{

            Unassigned = 0,
            Noise,
            Border,
            Core

        }; 
    }

}

class Point{

    public:
        Pose pose;
        
        bool visited;
        int class_; // Noise, Unassigned, Core, Border
        std::vector<Point*> neighbours;

        Point(double x, double y);

};