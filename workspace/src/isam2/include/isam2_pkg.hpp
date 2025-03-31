// #include "isam2.hpp"
#include <bits/stdc++.h>
// Camera observations of landmarks will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <eigen3/Eigen/Dense>
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
#include "loop_closure.hpp"

const string ESTIMATES_FILE = "src/isam2/data/current_estimates.txt";
const bool EUFS_SIM_SETTINGS = false;
/* Bearing and range error
* Corresponds to the usage of the BearingRangeFactor we are using
*
* Source: https://chem.libretexts.org/Bookshelves/Analytical_Chemistry/
*         Supplemental_Modules_(Analytical_Chemistry)/Quantifying_Nature/
*         Significant_Digits/Propagation_of_Error
* Source: User manual for the AT128 Hesai LiDAR
* 
* bearing error in radians
* Calculation for error per radian: 
* We use atan2(y, x) and we know that z = atan(u) has derivative 1/(1+u^2) 
* std.dev_{u} = sqrt(0.03^2 + 0.03^2) = 0.0424
* std.dev_{z} = (1/(1+1^2))^2 * std.dev_{u}^2 = 1/4 * 0.0424^2 = 0.00045
* 
* Source: User manual for the AT128 Hesai LiDAR
* range error in meters
* std.dev_{u} = 0.0424 -> changed to 0.03
* 
*/
const double BEARING_STD_DEV = 0.00045;
const double RANGE_STD_DEV = 0.03;


/* Go from 1 pose to another pose
* Source: https://www.movella.com/products/sensor-modules/xsens-mti-680g-rtk-gnss-ins
*
* x error in meters (must be based on velocity error)
* y error in meters (must be based on velocity error)
* velocity error = 0.05 m/s RMS
* Calculation for the error per meter: 1m = (1 +- 0.05 m/s) * (1 +- 1e-9 s)
* std. dev = 1 * sqrt((0.05/1)^2 + (1e-9/1)^2) = 0.05 -> changed to 0.05
*
* yaw error in radians changed to 0.009 
*/

const double IMU_X_STD_DEV = 0.22;
const double IMU_Y_STD_DEV = 0.22;
const double IMU_HEADING_STD_DEV = degrees_to_radians(0.009);



/* GPS noise model 
* Use the covariances from positionlla
*
* Covariance matrix diagonal elements represent variances
* Variance = (std.dev)^2 meaning:
* Variance = 0.2 => std.dev = sqrt(0.2) = 0.45
* 
* However positionlla already accounts for the covariance
* so we are using std.dev = 0.01
* 
*/
const double GPS_X_STD_DEV = 0.01;
const double GPS_Y_STD_DEV = 0.01;


/***** EUFS_SIM ******/
const double EUFS_SIM_RANGE_STD_DEV = 0.0;
const double EUFS_SIM_BEARING_STD_DEV = 0.0;
const double EUFS_SIM_IMU_X_STD_DEV = 0.0;
const double EUFS_SIM_IMU_Y_STD_DEV = 0.0;
const double EUFS_SIM_IMU_HEADING_STD_DEV = 0.0;
const double EUFS_SIM_GPS_X_STD_DEV = 0.0;
const double EUFS_SIM_GPS_Y_STD_DEV = 0.0;


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
    std::vector<Point2> cone_obs_blue;
    std::vector<Point2> cone_obs_yellow;

    Eigen::MatrixXd blue_global_cone_x;
    Eigen::MatrixXd blue_global_cone_y;

    Eigen::MatrixXd yellow_global_cone_x;
    Eigen::MatrixXd yellow_global_cone_y;

    Eigen::MatrixXd blue_bearing;
    Eigen::MatrixXd yellow_bearing;

    std::vector<Pose2> blue_cone_est;
    std::vector<Pose2> yellow_cone_est;

    Pose2 global_odom;

    std::vector<double> m_dist;

    bool loop_closure;
    bool new_lap;
    std::size_t lap_count;

    Pose2 first_pose;




public:
    high_resolution_clock::time_point start;
    high_resolution_clock::time_point end;
    int n_landmarks;
    int blue_n_landmarks;
    int yellow_n_landmarks;

    bool heuristic_run;

    /* how the landmark estimates are organized */
    std::vector<int> blue_cone_IDs;
    std::vector<int> yellow_cone_IDs;

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
            Pose2 &velocity,double dt, optional<rclcpp::Logger> logger);

    void update_landmarks(std::vector<Old_cone_info> &old_cones,
                        std::vector<New_cone_info> &new_cones,
                        Pose2 &cur_pose);

    void step(gtsam::Pose2 global_odom, std::vector<Point2> &cone_obs,
                std::vector<Point2> &cone_obs_blue, std::vector<Point2> &cone_obs_yellow,
                std::vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity,
                double dt);

    void print_estimates();
};
