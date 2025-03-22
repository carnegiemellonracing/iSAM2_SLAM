#pragma once

#include <vector>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <algorithm>
#include <float.h>
#include <unordered_set>
#include <stack>
#include <queue>

#include "ros_utils.hpp"
const double M_DIST_TH = 0.0009;
const double TURNING_M_DIST_TH = 0.0009;
const double JC_TH = 0.1; /* Joint compatibility threshold */
// const double M_DIST_TH = 0.0005;
// const double TURNING_M_DIST_TH = 0.0005;

struct New_cone_info {
    Point2 local_cone_pos;
    double bearing;
    Point2 global_cone_pos;
    int new_id;
    New_cone_info (Point2 local_cone_pos, double bearing, Point2 global_cone_pos, int new_id)
        : local_cone_pos(local_cone_pos)
        , bearing(bearing)
        , global_cone_pos(global_cone_pos)
        , new_id(new_id)
    {}
};

struct Old_cone_info {
    Point2 local_cone_pos;
    Point2 global_cone_pos;
    double bearing;
    int min_id; // The id of the old cone observed cone was associated with
    Old_cone_info (Point2 local_cone_pos, Point2 global_cone_pos, double bearing, int min_id)
        : local_cone_pos(local_cone_pos)
        , global_cone_pos(global_cone_pos)
        , bearing(bearing)
        , min_id(min_id)
    {}
};

/**
 * @brief Generate all feasible association sets by treating this as a 
 * Constraint Satisfaction Problem.
 * 
 * Variables represent the observed cone index
 * Domain values are possible old cone IDs to assign the observed cone to 
 */
class CSP {
    public: 
    struct CarInfo  {
        gtsam::Pose2 prev_pose;
        gtsam::Pose2 cur_pose;
        gtsam::Pose2 velocity;
        double dt;
        int num_obs;
    };

    /**  
     * Represents a previously seen landmark that has an index.
     * Used as the domain values of a variable/observed cone
     */
    struct EstimateConeInfo {
        int index;
        gtsam::Point2 global_cone_position;
    };

    using association_list_t = std::vector<std::optional<EstimateConeInfo>>;
    association_list_t assignment;


    std::optional<rclcpp::Logger> csp_logger;
    /**
     * @brief A constructor for the CSP
     * 
     * @param m_dist A vector of double representing the mahalanobis distance between
     * an observed cone and an old cone ID. 
     * Structure: each ith multiple of n_landmarks represents distances between the 
     * ith observed cone and all n_landmarks old landmarks. 
     * 
     * @param old_n_landmarks The number of landmarks in the previous time step. This 
     * is used for traversing the m_dist vector. 
     * 
     * @param new_n_landmarks The number of landmarks after get_old_new_cones in the 
     * current time step. This is used for the measurement model jacobian later. 
     */
    CSP(CSP::CarInfo input_car_info, std::vector<Old_cone_info>& old_cones, std::vector<double>& m_dist, std::vector<gtsam::Point2>& slam_est, 
            Eigen::MatrixXd input_covariance_est, Eigen::VectorXd landmark_noise, int old_n_landmarks, int new_n_landmarks, std::optional<rclcpp::Logger> logger);

    
    Eigen::MatrixXd get_best_hypothesis_msmt_jacobian();

    std::vector<Old_cone_info> find_best_association_list();

    private: 

    /* State covariances */
    Eigen::MatrixXd covariance_est; 
    Eigen::MatrixXd innovation_noise;

    /* Car info */
    CSP::CarInfo car_info;

    /* Cones */
    int num_obs_old_cones;
    int n_landmarks;
    std::vector<gtsam::Point2> obs_cone_global_positions;
    
    /**
     * Track information about the observed cone variable
     * 
     * a.) Track the domain values available for the observed cone
     * b.) Track the removal history for backtracking and rollbacks
     * c.) Track the global cone position of the observed cone
     */
    struct VariableInfo {
        /* Domain values*/
        using variable_domain_t = std::unordered_map<int, EstimateConeInfo>;
        variable_domain_t domain;


        struct HistoryItem {
            int backtracking_index;
            EstimateConeInfo removed_domain_info; 
        };

        std::stack<HistoryItem> removal_history;

        gtsam::Point2 global_cone_position;
        gtsam::Point2 local_cone_position;
        double bearing;

        std::optional<int> assigned_cone_ID;
    };

    /* Using a map to track the variables and their domain */
    std::vector<VariableInfo> all_variable_info;
    int cur_variable_to_be_assigned; /* Variables are assigned sequentially */

    Eigen::MatrixXd best_association_list_measurement_jacobian;
    association_list_t best_association_list;
    double best_joint_compatibility;

    /**
     * @brief A function to get the global positions of the observed cones.
     * The ith element represents the global position for the ith cone.
     * 
     */
    std::vector<gtsam::Point2> get_obs_cone_global_positions();

    /**
     * @brief A function to check if the current assignment is consistent
     * 
     * @param assignment The current assignment
     * @return True if the assignment is consistent, false otherwise
     */

    /**
     * @brief A function to rollback removals made from filtering
     */
    void rollback(int backtracking_index);

    /**
     * @brief The main function to find all feasible solutions for the CSP
     * 
     * @param assignment The current assignment as a vector where the 
     * index represents the observed cone index being assigned. The element
     * represents the old cone ID being assigned to the observed cone. 
     */
    void backtracking_search(int backtracking_index);

    /**
     * @brief A function to enforce arc consistency on the CSP.
     * Enforcing arc consistency means that given an arc from variable A to B,
     * we remove domain values from A that aren't consistent with a value in B
     * 
     * Here are the following constraints we employ:
     * a.) No other observed cone can have the current assignment in their domain
     * b.) The joint compatibility must be valid among the options in the other variables
     * 
     * Must also update the removal history so that we can undo later.
     * 
     * This function will be used whenever we assign a value to variable B
     * 
     * @param arc_to_enforce A pair where the first element is the tail
     * and the last element is the head of the arc.
     * 
     * @param backtracking_index The current backtracking index. Important for rollback
     * 
     * @param logger An optional logger for debugging
     * 
     * @return A boolean to indicate if a domain value was removed. Will inform
     * AC-3 algo if we need to enforce any more edges
     */
    bool enforce_arc_consistency(std::pair<int, int> arc_to_enforce, int backtracking_index);

    

    /** 
     * @brief A function to perform AC-3 on the CSP. Each time you 
     * remove a value from the domain of variable A when enforcing
     * arc consistency of A->B, you must enforce the arc consistency
     * from N->A for all neighbors N of A 
     */
    bool ac3(int index);
};

void populate_m_dist(MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                    int num_obs, std::vector<double> &m_dist, double threshold,
                    std::vector<Point2> &slam_est, std::vector<MatrixXd> &slam_mcov,
                    rclcpp::Logger &logger);

void get_old_new_cones(std::vector<Old_cone_info> &old_cones, std::vector<New_cone_info> &new_cones,
            Eigen::MatrixXd &covariance, Eigen::MatrixXd &noise, Eigen::VectorXd& landmark_noise, 
            MatrixXd &global_cone_x,MatrixXd &global_cone_y,MatrixXd &bearing,
            std::vector<Point2> &cone_obs, std::vector<double> &m_dist, int& n_landmarks,
            rclcpp::Logger &logger);

void jcbb(std::vector<Old_cone_info> &old_cones, std::vector<New_cone_info> &new_cones, 
            gtsam::Vector& landmark_noise, Eigen::MatrixXd &jcbb_state_covariance, Eigen::MatrixXd &jcbb_state_noise, 
            gtsam::Pose2 &prev_pose, gtsam::Pose2 &cur_pose, gtsam::Pose2 &velocity, 
            double dt, int num_obs, int& n_landmarks, bool is_turning,
            std::vector<gtsam::Point2> &cone_obs, std::optional<rclcpp::Logger> &logger,
            std::vector<gtsam::Point2> &slam_est, std::vector<gtsam::Matrix> &slam_mcov);


Eigen::MatrixXd get_measurement_model_jacobian(gtsam::Pose2 cur_pose, CSP::EstimateConeInfo est_cone_info, int n_landmarks, std::optional<rclcpp::Logger> logger);

double compute_joint_compatibility(Eigen::MatrixXd &covariance_est, std::vector<gtsam::Point2> obs_global_cones,
                                   CSP::association_list_t association_list_from_csp, gtsam::Pose2 cur_pose,
                                   Eigen::MatrixXd innovation_noise, int num_obs, int n_landmarks, std::optional<rclcpp::Logger> logger);