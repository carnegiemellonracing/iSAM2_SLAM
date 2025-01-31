#ifndef CONE_CACHE_TYPE_H
#define CONE_CACHE_TYPE_H

#include <tuple>
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Point2.h"

struct ConeCacheType {
    //Point2 is car-relative position of the cone double is the bearing from the car, and Point2 is the global position
    // Point2 cone; 
    tuple<Point2, double, Point2> cone;
    int ageTimeSteps; //how many time steps sitting in cone cache
    // int t; //number of times seen of cone
    ConeCacheType(tuple<Point2, double, Point2> cone, int ageTimeSteps): cone(cone), ageTimeSteps(ageTimeSteps) {};
    ConeCacheType(tuple<Point2, double, Point2> cone): cone(cone), ageTimeSteps(0) {};
};

#endif
