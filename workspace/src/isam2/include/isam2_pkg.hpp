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
#include "loop_closure.hpp"

struct NoiseInputs {
    double yaml_prior_imu_x_std_dev;
    double yaml_prior_imu_y_std_dev;
    double yaml_prior_imu_heading_std_dev;

    double yaml_bearing_std_dev;
    double yaml_range_std_dev;

    double yaml_imu_x_std_dev;
    double yaml_imu_y_std_dev;
    double yaml_imu_heading_std_dev;
    double yaml_gps_x_std_dev;
    double yaml_gps_y_std_dev;

    int yaml_look_radius;
    int yaml_min_cones_update_all;
    int yaml_window_update;

    double yaml_imu_offset;
    double yaml_lidar_offset;
    double yaml_max_cone_range;
    double yaml_turning_max_cone_range;
    double yaml_dist_from_start_loop_closure_th;
    double yaml_m_dist_th;
    double yaml_turning_m_dist_th;
    int yaml_update_iterations_n;
};

enum class RunSettings {
    Real,
    EUFSSim,
    ControlsSim
};

enum class ConeColor {
    Blue,
    Yellow
};

class slamISAM {

private:
    /* Variables related to ISAM2 factor graph*/
    ISAM2Params parameters;
    ISAM2 isam2;
    NonlinearFactorGraph graph;
    Values values;

    /* Functions for adding symbols to the ISAM2 factor graph */
    static gtsam::Symbol X(int robot_pose_id);
    static gtsam::Symbol BLUE_L(int cone_pose_id);
    static gtsam::Symbol YELLOW_L(int cone_pose_id);

    /* Pose and odometry information */
    gtsam::Pose2 first_pose;
    std::size_t pose_num;
    bool first_pose_added = false;
    gtsam::Pose2 global_odom;

    /* Current vector of mahalanobis distance calculations */
    std::vector<double> m_dist;

    /* Lap and loop closure related variables */
    bool loop_closure;
    bool new_lap;
    std::size_t lap_count;


    /* SLAM history of estimates and marginal covariance matrices */
    std::vector<gtsam::Point2> blue_slam_est;
    std::vector<Eigen::MatrixXd> blue_slam_mcov;
    std::vector<gtsam::Point2> yellow_slam_est;
    std::vector<Eigen::MatrixXd> yellow_slam_mcov;


    std::size_t checkpoint_to_update_beginning;
    std::size_t blue_checkpoint_id;
    std::size_t yellow_checkpoint_id;

    high_resolution_clock::time_point start;
    high_resolution_clock::time_point end;
    std::size_t blue_n_landmarks;
    std::size_t yellow_n_landmarks;

    /* Noise models */
    gtsam::Vector LandmarkNoiseModel;
    noiseModel::Diagonal::shared_ptr landmark_model;
    gtsam::Vector PriorNoiseModel;
    noiseModel::Diagonal::shared_ptr prior_model;
    gtsam::Vector OdomNoiseModel;
    noiseModel::Diagonal::shared_ptr odom_model;
    gtsam::Vector UnaryNoiseModel;
    noiseModel::Diagonal::shared_ptr unary_model;
    optional<rclcpp::Logger> logger;


    /* Tunable and adjustable parameters */
    int look_radius;
    int min_cones_update_all;
    int window_update;

    double imu_offset; // meters; offset from the center of the car
    double lidar_offset; // meters; offset from the center of the car
    double max_cone_range; // meters; how far from the car will we accept a cone to process
    double turning_max_cone_range; //meters; how far from the car will we accept a cone to process
    double dist_from_start_loop_closure_th;
    double m_dist_th;
    double turning_m_dist_th;
    int update_iterations_n;

    void update_slam_est_and_mcov_with_new(int old_n_landmarks, int new_n_landmarks, 
                                                                std::vector<gtsam::Point2>& color_slam_est, 
                                                                std::vector<Eigen::MatrixXd>& color_slam_mcov, 
                                                                gtsam::Symbol(*cone_key)(int));

    std::pair<int, int> update_slam_est_and_mcov_with_old(std::vector<Old_cone_info>& old_cones,
                                    std::vector<gtsam::Point2>& color_slam_est, 
                                    std::vector<Eigen::MatrixXd>& color_slam_mcov, gtsam::Symbol(*cone_key)(int));

    std::pair<int, int> update_all_est_mcov(int old_n_landmarks, std::vector<Old_cone_info>& old_cones,
                                    std::vector<gtsam::Point2>& color_slam_est, 
                                    std::vector<Eigen::MatrixXd>& color_slam_mcov, gtsam::Symbol(*cone_key)(int));

    void cone_proximity_updates(std::size_t pivot, std::size_t n_landmarks,
                                    std::vector<gtsam::Point2> &color_slam_est, std::vector<Eigen::MatrixXd>& color_slam_mcov, 
                                    gtsam::Symbol (*cone_key)(int));

    void update_all_est_mcov(int n_landmarks, std::vector<gtsam::Point2>& color_slam_est, 
                                                    std::vector<Eigen::MatrixXd>& color_slam_mcov, 
                                                    gtsam::Symbol(*cone_key)(int));

    void update_beginning(std::vector<gtsam::Point2>& color_slam_est, 
                          std::vector<Eigen::MatrixXd>& color_slam_mcov, 
                          gtsam::Symbol(*cone_key)(int));
    void print_estimates();

    void stability_update(bool sliding_window);

    void log_params_in_use(bool has_value);



public:
    slamISAM(std::optional<rclcpp::Logger> input_logger, std::optional<NoiseInputs>& yaml_noise_inputs);
    slamISAM(){}; /* Empty constructor */

    void update_poses(Pose2 &cur_pose, Pose2 &prev_pose, Pose2 &global_odom,
            Pose2 &velocity,double dt, std::optional<rclcpp::Logger> logger);

    int update_landmarks(const std::vector<Old_cone_info> &old_cones,
                          const std::vector<New_cone_info> &new_cones,
                                int n_landmarks, 
                                Pose2 &cur_pose, gtsam::Symbol (*cone_key)(int));

    void step(gtsam::Pose2 global_odom, std::vector<Point2> &cone_obs,
                std::vector<Point2> &cone_obs_blue, std::vector<Point2> &cone_obs_yellow,
                std::vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity,
                double dt);
    

};
