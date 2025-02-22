/**
 * @file data_association.cpp
 * @brief Perform data association in preparation for feeding data to SLAM
 */
#include "data_association.hpp"

void populate_m_dist(MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                    int num_obs, vector<double> &m_dist, double threshold,
                    vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov,
                    optional<rclcpp::Logger> &logger) {
    for (int i = 0; i < num_obs; i++) {
        for (int j = 0; j < slam_est.size(); j++) {

            MatrixXd diff(1, 2);
            diff << global_cone_x(i,0) - slam_est.at(j).x(),
                    global_cone_y(i,0) - slam_est.at(j).y();

            m_dist.push_back((diff * slam_mcov.at(j) * diff.transpose())(0, 0));

        }

        m_dist.push_back(threshold);
    }

}

/* @brief Determines which of the observed cones are new or old 
 * @param old_cones a vector of tuples. Point2 is car-relative position of the cone
 *          double is the bearing from the car, and int is the ID assoc. with cone
 * 
 * @param new_cones a vector of tuples. Point2 is car-relative position of the cone
 *          double is the bearing from the car, and Point2 is the global position.
 * 
 */
void get_old_new_cones(vector<tuple<Point2, double, int>> &old_cones,
            vector<tuple<Point2, double, Point2>> &new_cones,
            MatrixXd &global_cone_x,MatrixXd &global_cone_y,MatrixXd &bearing,
            vector<Point2> &cone_obs, vector<double> &m_dist, int n_landmarks,
            optional<rclcpp::Logger> &logger) {

    int lo = 0;
    int hi = 0;

    for (int i = 0; i < cone_obs.size(); i++) {
        hi += n_landmarks + 1;
        vector<double>::iterator min_dist = min_element(m_dist.begin() + lo,
                                                        m_dist.begin() + hi);
        int min_id = distance(m_dist.begin() + lo, min_dist);
        if (min_id == n_landmarks) {
            Point2 global_cone_pos = Point2(global_cone_x(i,0), global_cone_y(i,0));
            new_cones.emplace_back(cone_obs.at(i),
                                    bearing(i, 0),
                                    global_cone_pos);
        } else {
            old_cones.emplace_back(cone_obs.at(i), 
                                    bearing(i, 0), 
                                    min_id);
        }

        lo = hi;
    }
}

void data_association(vector<tuple<Point2, double, int>> &old_cones,
                vector<tuple<Point2, double, Point2>> &new_cones,
                Pose2 &cur_pose, Pose2 &prev_pose, bool is_turning,
                vector<Point2> &cone_obs, optional<rclcpp::Logger> &logger,
                vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov) {

    vector<double> m_dist = {};
    int n_landmarks = slam_est.size();

    double m_dist_th = M_DIST_TH;
    double cone_dist_th = MAX_CONE_RANGE;
    if (is_turning) {
        m_dist_th = TURNING_M_DIST_TH;
        cone_dist_th = TURNING_MAX_CONE_RANGE;
    }

    remove_far_cones(cone_obs, cone_dist_th);

    // Populating m_dist with mahalanobis distances
    MatrixXd global_cone_x(cone_obs.size(), 1);
    MatrixXd global_cone_y(cone_obs.size(), 1);
    MatrixXd bearing(cone_obs.size(), 1);
    MatrixXd range(cone_obs.size(), 1);

    calc_cone_range_from_car(range, cone_obs);
    calc_cone_bearing_from_car(bearing, cone_obs);

    cone_to_global_frame(range, bearing,
                         global_cone_x, global_cone_y,
                         cone_obs, cur_pose);
    
    
    populate_m_dist(global_cone_x, global_cone_y, cone_obs.size(), m_dist,
                    m_dist_th, slam_est, slam_mcov, logger);

    get_old_new_cones(old_cones, new_cones,
                    global_cone_x, global_cone_y,bearing,
                    cone_obs, m_dist, n_landmarks, logger);
}

/**
 * The 1st element in the pair is the observed cone idx
 * The 2nd element in the pair is the old cone id
 */
typedef std::vector<int> association_list_t;

/**
 * @brief Calculates the measurement model jacobian for the given innovation.
 * The jacobian should be a 2 row by 5 column matrix.
 * 
 * The cone should be in global frame coordinates. This is because we need the
 * change in x and y in the global frame. 
 */
Eigen::MatrixXd get_measurement_model_jacobian(gtsam::Pose2 pose, gtsam::Point2 cone_global_frame, int num_obs, int association_idx) {
    Eigen::Vector2d diff = Eigen::Vector2d(cone_global_frame.x() - pose.x(), cone_global_frame.y() - pose.y());
    double q = diff.squaredNorm();
    Eigen::MatrixXd jacobian(2, 2 * num_obs + 3);
    Eigen::MatrixXd range_bearing_state_deriv_block(2, 3);

    /* Calculate the jacobian in blocks */
    range_bearing_state_deriv_block << (-sqrt(q) * diff.x()), (-sqrt(q) * diff.y()), 0,
                                                   diff.y(), -diff.x(), -q;
    range_bearing_state_deriv_block = range_bearing_state_deriv_block *   1/q;
    
    Eigen::MatrixXd measurement_block(2, 2);
    measurement_block << (sqrt(q) * diff.x()), (sqrt(q) * diff.y()),
                         -diff.y(), diff.x();
    measurement_block = measurement_block * 1/q;

    jacobian.block<2, 3>(0, 0) = range_bearing_state_deriv_block;
    jacobian.block<2, 2>(0, 3 + 2 * association_idx) = measurement_block;

    return jacobian;

}

void get_covariance_estimate(gtsam::Pose2 prev_pose, Eigen::MatrixXd &covariance, gtsam::Pose2 velocity, double dt, int num_obs) {
    Eigen::MatrixXd state_jacobian_G = Eigen::MatrixXd::Identity(2 * num_obs + 3, 2 * num_obs + 3);
    double velocity_mag = norm2(Point2(velocity.x(), velocity.y()));
    state_jacobian_G(0, 2) = (-(velocity_mag / velocity.theta()) * cos(prev_pose.theta()) + 
                                (velocity_mag/velocity.theta()) *  cos(prev_pose.theta() + velocity.theta() * dt));
    state_jacobian_G(1, 2) = (-(velocity_mag / velocity.theta()) * sin(prev_pose.theta()) + 
                                (velocity_mag/velocity.theta()) *  sin(prev_pose.theta() + velocity.theta() * dt));

    covariance = state_jacobian_G * covariance * state_jacobian_G.transpose();

    covariance(0, 0) = pow(0.22, 2);
    covariance(1, 1) = pow(0.22, 2);
    covariance(2, 2) = pow(degrees_to_radians(0.009), 2);

}


/**
 * @brief Gets the kalman gain matrix for the ith observed cone. Provide the covariance estimate matrix
 * and the measurement jacobian for the ith observed cone.
 * 
 * This is to be used to correct the covariance matrix.
 */
Eigen::MatrixXd get_kalman_gain(Eigen::MatrixXd &covariance, Eigen::MatrixXd &msmt_jacobian) {
    Eigen::MatrixXd msmt_noise_Q(2, 2);
    msmt_noise_Q << pow(0.00045, 2), 0.0, 
                    0.0, pow(0.03, 2); 
    
    return (covariance * msmt_jacobian.transpose() * 
            (msmt_jacobian * covariance * msmt_jacobian.transpose() + msmt_noise_Q).inverse());
}

/**
 * @brief Correct the covariance matrix using the kalman gain and the measurement jacobian.
 */
void covariance_correction(Eigen::MatrixXd &kalman_gain, Eigen::MatrixXd &msmt_jacobian, Eigen::MatrixXd &covariance, int num_obs) {
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(2 * num_obs + 3, 2 * num_obs + 3);

    covariance = (identity - (kalman_gain * msmt_jacobian)) * covariance;
}


/** 
 * @brief This function will take in a vector of association sets.
 * 
 * The input is a vector of pairs.
 * Each pair is an association list and a vector of global cone positions.
 * 
 * An association list is a list where the indices represents the ith 
 * observed cone and the element represents the old cone ID mapped to the 
 * observed cone.
 */
void compute_joint_compatibilities (std::vector<std::pair<association_list_t, std::vector<gtsam::Point2>>> &association_data, 
                                    gtsam::Pose2 prev_pose, gtsam::Pose2 pose, gtsam::Pose2 velocity, double dt, int num_obs) {
    std::vector<double> compatibilities = {};
    Eigen::MatrixXd prev_cov_matrix; /* TODO INITIALIZE THIS GLOBALLY */
    
    /* Get covariance estimate */
    association_list_t best_association_list;
    std::vector<gtsam::Point2> best_association_estimates;
    get_covariance_estimate(prev_pose, covariance, velocity, dt, num_obs);

    for (int i = 0; i < association_data.size(); i++) {
        /* Get the current association set */
        association_list_t association_list = association_data.at(i).first;
        std::vector<gtsam::Point2> global_cone_est = association_data.at(i).second;

        
        /* Get the vector of innovations for the current association set */
        /* Recall that innovation is the different between the 
         * observation from the estimate */

        /* innov. cov = stack jacobians vertically; multiply them with the previous covariance matrix; multilpy with stacked jacobs. transposed add R */
        /* joint compatibility is the innovation vector transposed * innovation covariance * innovation vector */


    }

    /* After you have identified the association set to use */
    for (int j = 0; j < num_obs; j++) {
        /* For each landmark, calculate the measurement jacobian and the kalman gain to update the covariance */
        /* association_idx is the old cone_ID the current/jth observed cone is associated to */
        Eigen::MatrixXd measurement_jacobian = get_measurement_model_jacobian(pose, best_association_list.at(j), num_obs, best_association_estimates.at(j));
        Eigen::MatrixXd kalman_gain = get_kalman_gain(covariance, measurement_jacobian);

        covariance_correction(kalman_gain, measurement_jacobian, covariance, num_obs);
    }
}

/**
 * @brief This function will perform Joint Compatibility Branch and Bound
 * 1.) Calculate all of the association lists and filter out the ones that 
 * obey the individual association check (The associations where the 
 * distance between the observed and old cone being associated < M_DIST_TH)
 * 
 * 2.) Calculate the Joint compatibility of the association set E_{k}. 
 * 
 * 3.) Find the association set with the highest compatibility
 */
void jcbb() {
    std::vector<association_sets> all_association_sets = {};
    /**
     * Get the cones and their global positions so that we can do 
     * data association using the global positions of the cones
     */
    vector<double> m_dist = {};
    int n_landmarks = slam_est.size();

    double m_dist_th = M_DIST_TH;
    double cone_dist_th = MAX_CONE_RANGE;
    if (is_turning) {
        m_dist_th = TURNING_M_DIST_TH;
        cone_dist_th = TURNING_MAX_CONE_RANGE;
    }

    remove_far_cones(cone_obs, cone_dist_th);

    // Populating m_dist with mahalanobis distances
    MatrixXd global_cone_x(cone_obs.size(), 1);
    MatrixXd global_cone_y(cone_obs.size(), 1);
    MatrixXd bearing(cone_obs.size(), 1);
    MatrixXd range(cone_obs.size(), 1);

    calc_cone_range_from_car(range, cone_obs);
    calc_cone_bearing_from_car(bearing, cone_obs);

    cone_to_global_frame(range, bearing,
                         global_cone_x, global_cone_y,
                         cone_obs, cur_pose);
    
    
    populate_m_dist(global_cone_x, global_cone_y, cone_obs.size(), m_dist,
                    m_dist_th, slam_est, slam_mcov, logger);    

    /**
     * Generate association sets
     * 1.) Find the valid matches for a given observed cone (each match must
     * satisfy the individual compatibility test)
     * 
     * 2.) Generate all possible sets from these matches
     */
    generate_pairings(all_association_sets);

    /* Calculate compatibility for each association set*/
    compute_joint_compatibilities(all_association_sets);
}

/**
 * Generates all viable pairings by checking if each Mahalanobis
 * Distance is under the threshold.
 */
vector<vector<pair<int, gtsam::Point2>>> get_viable_pairings(int num_obs, int num_landmarks, const vector<double> &m_dist, 
    const vector<gtsam::Point2> &old_cone_ests, const vector<gtsam::Point2> &cone_obs) {
    vector<vector<int>> ans(num_obs);
    for(int i = 0; i < num_obs; i++) {
        for(int j = 0; j < num_landmarks; j++) {
            if(m_dist[i * num_landmarks + j] < M_DIST_TH) {
                gtsam::Point2 cur_diffs = gtsam::Point2(cone_obs[i].x - old_cone_ests[j].x, cone_obs[i].y - old_cone_ests[j].y);
                ans[i].push_back(make_pair(j, cur_diffs));
            }
        }
    }
    return ans;
}

/**
 * Helper function for generate_pairings that uses dfs to
 * generate all possible injective functions and adds them to the
 * vector of all association sets.
 */
void generate_pairings_helper(int cur, int num_obs, const vector<vector<pair<int, gtsam::Point2>>> &allowed, 
    vector<bool> &visited, vector<pair<int, gtsam::Point2>> pairing, vector<pair<association_list_t, vector<gtsam::Point2>>> &ans) {
        if(cur == num_obs) {
            association_list_t cur_pairs;
            vector<gtsam::Point2> cur_dists;
            for(auto p: pairing) {
                cur_pairs.push_back(p.first);
                cur_dists.push_back(p.second);
            }
            ans.push_back(make_pair(cur_pairs, cur_dists));
        }
        else {
            for(auto j: allowed[cur]) {
                if(!visited[j.first]) {
                    visited[j.first] = true;
                    pairing[cur] = j;
                    dfs(cur + 1, num_obs, allowed, visited, pairing, ans);
                    visited[j.first] = false;
                }
            }
        }
    }

    /**
     * Generates all association sets. Takes in a adjacency list
     * of all possible landmarks given a observation and returns 
     * a set of all injective functions from landmarks to observations.
     */
vector<pair<association_list_t, vector<gtsam::Point2>>> generate_pairings(int num_obs, int num_landmarks, const vector<vector<pair<int, gtsam::Point2>>> &allowed) {
    vector<bool> visited(num_landmarks, false);
    vector<pair<int, gtsam::Point2>> pairing(num_obs);
    vector<pair<association_list_t, vector<gtsam::Point2>>> ans;
    
    generate_pairings_helper(0, n, allowed, visited, pairing, ans);
    return ans;
}
