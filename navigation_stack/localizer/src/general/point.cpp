#include "general/point.h"

// ---------------------------------------------------------------------------------------
//                               Point Class 
// ---------------------------------------------------------------------------------------

Point::Point(double x, double y): pose{x, y}, class_(point::classes::Unassigned), visited(false) {}

