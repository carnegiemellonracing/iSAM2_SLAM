// #include "isam2.hpp"
#include <bits/stdc++.h>
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

// iSAM2 requires as input a set of new factors to be added stored in a factor
// graph, and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

#include "data_association.hpp"
#include "unary_factor.hpp"
#include "ros_utils.hpp"

using namespace std;
using namespace gtsam;
using namespace std::chrono;

class slamISAM {

private:
    ISAM2Params parameters;
    ISAM2 isam2;
    //Create a factor graph and values for new data
    NonlinearFactorGraph graph;
    Values values;

    int pose_num = 0;
    bool first_pose_added = false;

    gtsam::Symbol X(int robot_pose_id) {
        return Symbol('x', robot_pose_id);
    }

    gtsam::Symbol L(int cone_pose_id) {
        return Symbol('l', cone_pose_id);
    }

    /* Assoc_Args common arguments */
    vector<Point2> cone_obs_blue;
    vector<Point2> cone_obs_yellow;

    Eigen::MatrixXd blue_global_cone_x;
    Eigen::MatrixXd blue_global_cone_y;

    Eigen::MatrixXd yellow_global_cone_x;
    Eigen::MatrixXd yellow_global_cone_y;

    Eigen::MatrixXd blue_bearing;
    Eigen::MatrixXd yellow_bearing;

    vector<Pose2> blue_cone_est;
    vector<Pose2> yellow_cone_est;

    Pose2 global_odom;

    vector<double> m_dist;


public:
    high_resolution_clock::time_point start;
    high_resolution_clock::time_point end;
    int n_landmarks;
    int blue_n_landmarks;
    int yellow_n_landmarks;

    bool heuristic_run;

    //gtsam::Pose2 robot_est;
    std::vector<gtsam::Pose2> landmark_est;

    /* how the landmark estimates are organized */
    vector<int> blue_cone_IDs;
    vector<int> yellow_cone_IDs;

    gtsam::Vector LandmarkNoiseModel;
    noiseModel::Diagonal::shared_ptr landmark_model;
    gtsam::Vector PriorNoiseModel;
    noiseModel::Diagonal::shared_ptr prior_model;
    gtsam::Vector OdomNoiseModel;
    noiseModel::Diagonal::shared_ptr odom_model;
    gtsam::Vector UnaryNoiseModel;
    noiseModel::Diagonal::shared_ptr unary_model;
    optional<rclcpp::Logger> logger;

    slamISAM(optional<rclcpp::Logger> input_logger);

    void update_poses(Pose2 &cur_pose, Pose2 &prev_pose, Pose2 &global_odom,
            Pose2 &velocity,double dt, bool new_gps, optional<rclcpp::Logger> logger);

    void update_landmarks(vector<tuple<Point2, double, int>> &old_cones,
                        vector<tuple<Point2, double, Point2>> &new_cones,
                        Pose2 &cur_pose);

    void step(gtsam::Pose2 global_odom, vector<Point2> &cone_obs,
                vector<Point2> &cone_obs_blue, vector<Point2> &cone_obs_yellow,
                vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity,
                double dt);
};