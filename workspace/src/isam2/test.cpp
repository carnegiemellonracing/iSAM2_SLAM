// #include "isam2.hpp"
#include <type_traits>
// Camera observations of landmarks will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <Eigen/Dense>
// Each variable in the system (sposes and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem
// incrementally, so include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

#include <vector>

#include <iostream>
#include <fstream>
// using namespace std;    
using namespace gtsam;

static const float M_DIST_TH = 0.000151169; // used to be 45 lmao
// static const float M_DIST_TH = 45; // used to be 45 lmao
static const long SEC_TO_NANOSEC = 1000000000;

int n_landmarks = 3;

int main(){


    gtsam::Pose2 global_odom = gtsam::Pose2(1,2,0);
    std::vector<Point2> cone_obs;
    cone_obs.push_back(gtsam::Point2(1,1));
    cone_obs.push_back(gtsam::Point2(2,2));
    cone_obs.push_back(gtsam::Point2(3,3));


    //Make the incoming cones into an eigen matrix
    Eigen::MatrixXd cone_meas(n_landmarks,2);
    //TODO: make sure this actually works
    for (int i = 0; i < n_landmarks; i++) {
        cone_meas(i) = cone_obs.at(i).x(),cone_obs.at(i).y();
    }

    //Range
    Eigen::MatrixXd range(n_landmarks,1);
    range = cone_meas.rowwise().norm();

    //Angle: eigen atan2 is sus
    //TODO: find a faster way to do this
    Eigen::MatrixXd bearing(n_landmarks,1);
    for (int i = 0; i < n_landmarks; i++) {
        bearing(i) = atan2(cone_obs.at(i).y(),cone_obs.at(i).x());
    }

    Eigen::MatrixXd totalBearing = bearing.array()+global_odom.theta();
    Eigen::MatrixXd global_cone_x(n_landmarks,1);
    Eigen::MatrixXd global_cone_y(n_landmarks,1);

    global_cone_x = global_odom.x() + range*(totalBearing.cos());
    global_cone_y = global_odom.y() + range*totalBearing.sin();


}

