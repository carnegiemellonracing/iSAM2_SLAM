/**
 * @file data_association.cpp
 * @brief Perform data association in preparation for feeding data to SLAM
 */
#include "data_association.hpp"



void populate_m_dist(Eigen::MatrixXd &global_cone_x, Eigen::MatrixXd &global_cone_y,
                    int num_obs, std::vector<double> &m_dist, double threshold,
                    std::vector<gtsam::Point2> &slam_est, std::vector<Eigen::MatrixXd> &slam_mcov,
                    std::optional<rclcpp::Logger> &logger) {
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

void extend_cov_and_noise_matrix(Eigen::MatrixXd &covariance, Eigen::MatrixXd &noise, 
                                Eigen::VectorXd &landmark_noise, int num_new_cones, optional<rclcpp::Logger>& logger) {
    if (num_new_cones == 0) {
        return;
    }
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "extend_cov_and_noise_matrix initial dimensions");
        RCLCPP_INFO(logger.value(), "\tcovariance rows:%d cols:%d", covariance.rows(), covariance.cols());
        RCLCPP_INFO(logger.value(), "\tnoise rows:%d cols:%d", noise.rows(), noise.cols());
        RCLCPP_INFO(logger.value(), "\tnum_new_cones:%d", num_new_cones);
    }
    
    int cov_rows = covariance.rows();
    int cov_cols = covariance.cols();
    Eigen::MatrixXd new_cov = Eigen::MatrixXd::Zero(cov_rows + (2 * num_new_cones), cov_cols+ (2 * num_new_cones));
    new_cov.block(0, 0, cov_rows, cov_cols) = covariance;
    covariance = new_cov;
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "\tfinished covariance matrix");
    }
    // TODO: need to check if we need to initialize diagonals to infinity
    


    int noise_rows = noise.rows();
    int noise_cols = noise.cols();
    Eigen::MatrixXd new_noise = Eigen::MatrixXd::Zero(noise_rows + 2 * num_new_cones, noise_cols + 2 * num_new_cones);
    new_noise.block(0, 0, noise_rows, noise_cols) = noise;
    noise = new_noise;

    noise(noise_rows, noise_cols) = landmark_noise(0);
    noise(noise_rows+1, noise_cols+1) = landmark_noise(1);


    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "extend_cov_and_noise_matrix resulting dimensions");
        RCLCPP_INFO(logger.value(), "\tcovariance rows:%d cols:%d", covariance.rows(), covariance.cols());
        RCLCPP_INFO(logger.value(), "\tnoise rows:%d cols:%d", noise.rows(), noise.cols());
    }
    
}

/**
 * @brief Determines which of the observed cones are new or old. 
 * Updates the covariance and noise matrix for JCBB by adding new rows and columns
 * for new landmarks when a new landmark is encountered.
 * 
 * @param old_cones a vector of tuples. Point2 is car-relative position of the cone
 *          double is the bearing from the car, and int is the ID assoc. with cone
 * 
 * @param new_cones a vector of tuples. Point2 is car-relative position of the cone
 *          double is the bearing from the car, and Point2 is the global position.
 * 
 */
void get_old_new_cones(std::vector<Old_cone_info> &old_cones,std::vector<New_cone_info> &new_cones,
                        Eigen::MatrixXd &covariance, Eigen::MatrixXd &noise, Eigen::VectorXd &landmark_noise,
                    Eigen::MatrixXd &global_cone_x, Eigen::MatrixXd &global_cone_y, Eigen::MatrixXd &bearing,
                    std::vector<gtsam::Point2> &cone_obs, std::vector<double> &m_dist, int& n_landmarks,
                    optional<rclcpp::Logger> &logger) {

    
    int lo = 0;
    int hi = 0;

    int num_new_cones = 0;
    int old_n_landmarks = n_landmarks;
    for (int i = 0; i < cone_obs.size(); i++) {
        hi += old_n_landmarks + 1;
        vector<double>::iterator min_dist = min_element(m_dist.begin() + lo,
                                                        m_dist.begin() + hi);
        int min_id = distance(m_dist.begin() + lo, min_dist);
        if (min_id == old_n_landmarks) {
            Point2 global_cone_pos = Point2(global_cone_x(i,0), global_cone_y(i,0));
            new_cones.emplace_back(cone_obs.at(i),
                                    bearing(i, 0),
                                    global_cone_pos, 
                                    n_landmarks);

            n_landmarks++;
            num_new_cones++;


        } else {
            Point2 global_cone_pos = Point2(global_cone_x(i,0), global_cone_y(i,0));
            old_cones.emplace_back(cone_obs.at(i), 
                                    global_cone_pos,
                                    bearing(i, 0), 
                                    min_id);
        }

        lo = hi;
    }
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "inside get_old_new_cones");
        RCLCPP_INFO(logger.value(), "\tn_landmarks: %d", n_landmarks);
        RCLCPP_INFO(logger.value(), "\tnum_obs: %d", cone_obs.size());
    }
    extend_cov_and_noise_matrix(covariance, noise, landmark_noise, num_new_cones, logger);
}


/* STATUS: Verified */
/**
 * @brief Calculates jacobian of the measurement model for the jth landmark 
 * with respect to the state vector. This function will be used to calculate
 * the jacobian of the measurement model for all observed landmarks associated
 * to some jth old cone ID. 
 * 
 * Note that the "jacobian with respect to pose x_{t}" really means that 
 * that we are taking partial derivatives with respect to the components 
 * making up x_{t}. 
 * 
 * @param cur_pose Current pose
 * @param est_cone_info Represents information about the cone/landmark that is being 
 * associated with an observed cone. Contains information about the global
 * position of the cone and its index. 
 * 
 * @return Returns a 2 by 2n+3 matrix
 */
Eigen::MatrixXd get_measurement_model_jacobian(gtsam::Pose2 cur_pose, CSP::EstimateConeInfo est_cone_info, int n_landmarks, std::optional<rclcpp::Logger> logger) {
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "inside get_measurement_model_jacobian");
    }
    gtsam::Point2 cone_global_frame = est_cone_info.global_cone_position;
    int association_idx = est_cone_info.index;

    Eigen::Vector2d diff = Eigen::Vector2d(cone_global_frame.x() - cur_pose.x(), cone_global_frame.y() - cur_pose.y());
    double q = diff.squaredNorm();
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "get_measurement_model_jacobian: calculated q vector");
    }
    Eigen::MatrixXd jacobian(2, 2 * n_landmarks + 3);
    Eigen::MatrixXd range_bearing_state_deriv_block(2, 3);

    /* Calculate the jacobian in blocks */
    range_bearing_state_deriv_block << (-sqrt(q) * diff.x()), (-sqrt(q) * diff.y()), 0,
                                                   diff.y(), -diff.x(), -q;
    range_bearing_state_deriv_block = range_bearing_state_deriv_block *   1/q;

    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "get_measurement_model_jacobian: calculated range_bearing state deriv block");
    }
    Eigen::MatrixXd measurement_block(2, 2);
    measurement_block << (sqrt(q) * diff.x()), (sqrt(q) * diff.y()),
                         -diff.y(), diff.x();
    measurement_block = measurement_block * 1/q;

    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "measurement_block shape: %d x %d", measurement_block.rows(), measurement_block.cols());
    }
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "jacobian shape: %d x %d", jacobian.rows(), jacobian.cols());
    }
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "association_idx: %d ", association_idx);
    }
    jacobian.block<2, 3>(0, 0) = range_bearing_state_deriv_block;
    jacobian.block<2, 2>(0, 3 + 2 * association_idx) = measurement_block;

    return jacobian;

}

/* STATUS: verified */
/**
 * @brief Estimates the state covariance matrix for the current time step, based on the 
 * previous pose and the odometry of the current time step. 
 * 
 * @return Returns the updated covariance matrix of the same dimensions: 2n+3 by 2n+3
 * where n is the number of old landmarks previously observed. 
 */
void get_covariance_estimate(Eigen::MatrixXd &covariance, Eigen::MatrixXd &noise, gtsam::Pose2 prev_pose, 
                            gtsam::Pose2 velocity, double dt, int n_landmarks, optional<rclcpp::Logger>& logger) {

    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "inside get_covariance_estimate");
        RCLCPP_INFO(logger.value(), "\tn_landmarks: %d", n_landmarks);
    }
    Eigen::MatrixXd state_jacobian_G = Eigen::MatrixXd::Identity(2 * n_landmarks + 3, 2 * n_landmarks + 3);
    double velocity_mag = norm2(Point2(velocity.x(), velocity.y()));
    state_jacobian_G(0, 2) = (-(velocity_mag / velocity.theta()) * cos(prev_pose.theta()) + 
                                (velocity_mag/velocity.theta()) *  cos(prev_pose.theta() + velocity.theta() * dt));

    state_jacobian_G(1, 2) = (-(velocity_mag / velocity.theta()) * sin(prev_pose.theta()) + 
                                (velocity_mag/velocity.theta()) *  sin(prev_pose.theta() + velocity.theta() * dt));



    /* Updating the covariance matrix */
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "Dimensions:");
        RCLCPP_INFO(logger.value(), "state_jacobian_G rows:%d cols:%d", state_jacobian_G.rows(), state_jacobian_G.cols());
        RCLCPP_INFO(logger.value(), "covariance rows:%d cols:%d", covariance.rows(), covariance.cols());
    }
    covariance = state_jacobian_G * covariance * state_jacobian_G.transpose();

    /* Applying the noise model to the covariance matrix*/
    covariance(0, 0) += pow(noise(0, 0), 2);
    covariance(1, 1) += pow(noise(1, 1), 2);
    covariance(2, 2) += pow(noise(2, 2), 2);

}

/* STATUS: verified */
/**
 * @brief Gets the kalman gain matrix for landmark ID associated to an observed cone. 
 * This function is used when correcting the covariance matrix.
 * 
 * @param covariance The covariance matrix
 * @param msmt_jacobian The measurement jacobian for the ith landmark
 * 
 */
Eigen::MatrixXd get_kalman_gain(Eigen::MatrixXd &covariance, Eigen::VectorXd &landmark_noise, Eigen::MatrixXd &msmt_jacobian) {
    Eigen::MatrixXd msmt_noise_Q = Eigen::MatrixXd::Zero(2, 2);
    msmt_noise_Q(0, 0) = pow(landmark_noise(0), 2);
    msmt_noise_Q(1, 1) = pow(0.03, 2); 
    
    return (covariance * msmt_jacobian.transpose() * 
            (msmt_jacobian * covariance * msmt_jacobian.transpose() + msmt_noise_Q).inverse());
}

/* STATUS: verified */
/**
 * @brief This function corrects the covariance matrix using the measurement jacobians
 * from the best association list.
 * 
 * @param num_obs The number of observed cones in the current time step
 * @param covariance The covariance matrix to be updated
 * @param hypothesis_msmt_jacobian The measurement jacobian for the num_obs hypotheses. 
 * This matrix is the measurement jacobian created from vertically stacking 
 * measurement jacobians of each jth landmark associated to an observed cone. 
 */
void covariance_correction(Eigen::MatrixXd &covariance, Eigen::VectorXd &landmark_noise, 
                            int num_obs, int n_landmarks, Eigen::MatrixXd& hypothesis_msmt_jacobian, optional<rclcpp::Logger>& logger) {
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "inside covariance_correction");
        RCLCPP_INFO(logger.value(), "\tnum_obs: %d", num_obs);
        RCLCPP_INFO(logger.value(), "\tn_landmarks: %d", n_landmarks);
        RCLCPP_INFO(logger.value(), "Dimensions:");
        RCLCPP_INFO(logger.value(), "covariance rows:%d cols:%d", covariance.rows(), covariance.cols());
        RCLCPP_INFO(logger.value(), "hypothesis_msmt_jacobian rows:%d cols:%d", hypothesis_msmt_jacobian.rows(), hypothesis_msmt_jacobian.cols());
    }

    if (num_obs == 0) {
        return; /* no observations to perform corrections with */
    }

    for (int i = 0; i < num_obs; i++) {
        Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(2 * num_obs + 3, 2 * num_obs + 3);

        /**  
         * The robot pose contains 3 pieces of data: x, y, and theta; 
         * Each landmark has a range and bearing: 2 * n_landmarks 
         */
        int msmt_jacobian_cols = 2 * n_landmarks + 3; 
        int msmt_jacobian_rows = 2;
        Eigen::MatrixXd ith_msmt_jacobian = hypothesis_msmt_jacobian.block(i * msmt_jacobian_rows, 0,
                                                                            msmt_jacobian_rows, msmt_jacobian_cols);

        Eigen::MatrixXd kalman_gain = get_kalman_gain(covariance, landmark_noise, ith_msmt_jacobian);
        covariance = (identity - (kalman_gain * ith_msmt_jacobian)) * covariance;
    }

}

/* STATUS: Verified */
/** 
 * @brief Computes the joint compatibility among the associations/assignments that have been made. 
 * 
 * @param covariance_est The estimated covariance for the current time step. Observe that 
 * this is different from the previous covariance (make sure to call get_covariance_estimate
 * on the corrected covariance matrix from the previous time step). This is because 
 * this function will be called many times within 1 time step, but there's only 1
 * covariance estimate within 1 time step.
 * 
 * @param obs_global_cones The global cone positions of the current observed cones. 
 * 
 * @param association_list A vector of domain values representing an old cone, its index, 
 * and its global position. 
 */
double compute_joint_compatibility (Eigen::MatrixXd& covariance_est, std::vector<gtsam::Point2> obs_global_cones, 
                                    CSP::association_list_t association_list_from_csp, gtsam::Pose2 cur_pose,
                                    Eigen::MatrixXd innovation_noise, int num_assigned, int n_landmarks, std::optional<rclcpp::Logger> logger) {
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "inside compuate_joint_compatibility");
    }
    std::vector<CSP::EstimateConeInfo> association_list = {};
    for (int i = 0; i < num_assigned; i++) {
        assert(association_list_from_csp.at(i).has_value());
        association_list.push_back(association_list_from_csp.at(i).value());
    }

    assert(association_list.size() == num_assigned);
    /**
     * Calculate the innovation between observation and estimate
     * innovation = observation - estimate
     */
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "compute_joint_compatibility: computing innovation");
    }
    Eigen::VectorXd innovation(2 * num_assigned);
    for (int i = 0; i < num_assigned; i++) {
        int x_idx = i * 2;
        int y_idx = i * 2 + 1; 
        
        /* Note that we are working with the global positions of the cones */
        gtsam::Point2 observed_cone = obs_global_cones.at(i);
        gtsam::Point2 associated_old_cone_estimate = association_list.at(i).global_cone_position;
        innovation(x_idx) = observed_cone.x() - associated_old_cone_estimate.x();
        innovation(y_idx) = observed_cone.y() - associated_old_cone_estimate.y();
    }

    /**
     * Calculate the hypothesis measurement jacobian 
     * "Hypothesis" represents the possible best association list */
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "compute_joint_compatibility: computing hypothesis_msmt_jacobian");
    }
    Eigen::MatrixXd hypothesis_msmt_jacobian(2 * num_assigned, 2 * n_landmarks + 3);
    for (int i = 0; i < num_assigned; i++) {
        Eigen::MatrixXd ith_msmt_jacobian = get_measurement_model_jacobian(cur_pose, association_list.at(i), n_landmarks, logger);
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "ith_msmt_jacobian shape: %ld x %ld", ith_msmt_jacobian.rows(), ith_msmt_jacobian.cols());
        }   
        hypothesis_msmt_jacobian.block(2 * i, 0, 2, 2 * n_landmarks + 3) = ith_msmt_jacobian;
    }

    /**
     * Calculate the innovation covariance
     * This will be a 
     */
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "hypothesis_msmt_jacobian shape: %ld x %ld", 
                            hypothesis_msmt_jacobian.rows(), hypothesis_msmt_jacobian.cols());
        RCLCPP_INFO(logger.value(),"covariance_est shape: %ld x %ld", covariance_est.rows(), covariance_est.cols());
        RCLCPP_INFO(logger.value(),"innovation_noise shape: %ld x %ld", innovation_noise.rows(), innovation_noise.cols());
        RCLCPP_INFO(logger.value(),"innovation shape: %ld", innovation.size());
    }
    Eigen::MatrixXd innovation_covariance = hypothesis_msmt_jacobian * covariance_est * hypothesis_msmt_jacobian.transpose() + innovation_noise;


    return innovation.transpose() * innovation_covariance.inverse() * innovation;
    


}



/**
 * @brief This function will perform Joint Compatibility Branch and Bound
 * 1.) Calculate all of the association lists and filter out the ones that 
 * obey the individual association check (The associations where the 
 * distance between the observed and old cone being associated < M_DIST_TH)
 * 
 * 
 * 2.) Calculate the Joint compatibility of the association set E_{k}. 
 * 
 * 3.) Find the association set with the highest compatibility
 * 
 * NOTE: This function will modify cone_obs because we only care about the 
 * cones within an acceptable range of the car
 */
void jcbb(std::vector<Old_cone_info> &old_cones, std::vector<New_cone_info> &new_cones, 
            gtsam::Vector& landmark_noise, Eigen::MatrixXd &jcbb_state_covariance, Eigen::MatrixXd &jcbb_state_noise, 
            gtsam::Pose2 &prev_pose, gtsam::Pose2 &cur_pose, gtsam::Pose2 &velocity, 
            double dt, int num_obs, int& n_landmarks, bool is_turning,
            std::vector<gtsam::Point2> &cone_obs, optional<rclcpp::Logger> &logger,
            std::vector<gtsam::Point2> &slam_est, std::vector<gtsam::Matrix> &slam_mcov) {

    assert(num_obs == cone_obs.size());
    assert(n_landmarks == slam_est.size());

    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "<---- Starting JCBB ---->");
        RCLCPP_INFO(logger.value(), "\tn_landmarks: %d | slam_est.size(): %d", n_landmarks, slam_est.size());
        RCLCPP_INFO(logger.value(), "\tnum_obs: %d | cone_obs.size(): %d", num_obs, cone_obs.size());
    } 


    /**
     * Get the cones and their global positions so that we can do 
     * data association using the global positions of the cones
     */
    std::vector<double> m_dist = {};

    double m_dist_th = M_DIST_TH;
    double cone_dist_th = MAX_CONE_RANGE;
    if (is_turning) {
        m_dist_th = TURNING_M_DIST_TH;
        cone_dist_th = TURNING_MAX_CONE_RANGE;
    }

    remove_far_cones(cone_obs, cone_dist_th);

    num_obs = cone_obs.size();
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

   
    auto start_get_old_new_cones = std::chrono::high_resolution_clock::now();
    get_old_new_cones(old_cones, new_cones,
                    jcbb_state_covariance, jcbb_state_noise, landmark_noise,
                    global_cone_x, global_cone_y,bearing,
                    cone_obs, m_dist, n_landmarks, logger);
    auto end_get_old_new_cones = std::chrono::high_resolution_clock::now();
    auto dur_get_old_new_cones = std::chrono::duration_cast<std::chrono::milliseconds>(end_get_old_new_cones - start_get_old_new_cones);
   
    

    /**
     * Preparation for JCBB
     */ 
    
    auto start_get_cov_est = std::chrono::high_resolution_clock::now();
    get_covariance_estimate(jcbb_state_covariance, jcbb_state_noise, prev_pose, velocity, dt, n_landmarks, logger);
    auto end_get_cov_est = std::chrono::high_resolution_clock::now();
    auto dur_get_cov_est = std::chrono::duration_cast<std::chrono::milliseconds>(end_get_cov_est - start_get_cov_est);
    



    int num_obs_old_cones = old_cones.size();
    if (num_obs_old_cones > 0) {
        auto start_constructor = std::chrono::high_resolution_clock::now();
        CSP::CarInfo car_info = {prev_pose, cur_pose, velocity, dt, num_obs_old_cones}; 
        CSP csp = CSP(car_info, old_cones, m_dist, slam_est, 
                        jcbb_state_covariance, landmark_noise, slam_est.size(), n_landmarks, logger);
        
                        
        auto end_constructor = std::chrono::high_resolution_clock::now();
        auto dur_constructor = std::chrono::duration_cast<std::chrono::milliseconds>(end_constructor - start_constructor);

        if (logger.has_value())  {
            RCLCPP_INFO(logger.value(), "\tconstructor time: %ld", dur_constructor.count());
        }


        /* Return a vector of Old_cone_info in preparation for updating the SLAM model*/
        

        auto start_find_best = std::chrono::high_resolution_clock::now();
        std::vector<Old_cone_info> association_list = csp.find_best_association_list();
        auto end_find_best = std::chrono::high_resolution_clock::now();
        auto dur_find_best = std::chrono::duration_cast<std::chrono::milliseconds>(end_find_best - start_find_best);


        /* Apply corrections to the covariance matrix */
        Eigen::MatrixXd best_hypothesis_msmt_jacobian = csp.get_best_hypothesis_msmt_jacobian();
        covariance_correction(jcbb_state_covariance, landmark_noise, num_obs_old_cones, n_landmarks, best_hypothesis_msmt_jacobian, logger);

        /* Set old cones to be the association list */
        old_cones = association_list;
    } 

    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "<---- End JCBB ---->");
        RCLCPP_INFO(logger.value(), "\tn_landmarks: %d ", n_landmarks);
    }

}



CSP::CSP(CSP::CarInfo input_car_info, std::vector<Old_cone_info>& old_cones, std::vector<double>& m_dist, std::vector<gtsam::Point2>& slam_est, 
    Eigen::MatrixXd input_covariance_est, Eigen::VectorXd landmark_noise, int old_n_landmarks, int new_n_landmarks, std::optional<rclcpp::Logger> logger) {  

    csp_logger = logger;
    
    
    car_info = input_car_info;

    covariance_est = input_covariance_est;
    
    int dim = landmark_noise.size();
    innovation_noise = MatrixXd::Zero(dim, dim);
    for(int i = 0; i < dim; i++) {
        innovation_noise(i, i) = landmark_noise(i) * landmark_noise(i);
    }
    
    num_obs_old_cones = old_cones.size();

    /* Defining the variables in the CSP and their domains 
     * We use old_n_landmarks because m_dist was created with respect to old_n_landmarks 
     * (before we registered any new cones)
     */
    int lo = 0;
    for (int i = 0; i < num_obs_old_cones; i++) {
        obs_cone_global_positions.push_back(old_cones[i].global_cone_pos);
        VariableInfo cur_info;
        for (int n = 0; n < old_n_landmarks + 1; n++) {
            if (m_dist.at(n + lo) < M_DIST_TH) {
                cur_info.domain[n] = {n, slam_est.at(n)};
                cur_info.bearing = old_cones[i].bearing;
                cur_info.local_cone_position = old_cones[i].local_cone_pos;
                cur_info.global_cone_position = old_cones[i].global_cone_pos;
            }
        }

        lo += old_n_landmarks + 1;
        all_variable_info.push_back(cur_info);
    }

    /* The state and other matrices for computing the joint compatibility 
     * uses new_n_landmarks (because the new_cones have been incorporated in the dimensions)
     */
    n_landmarks = new_n_landmarks;

    best_association_list_measurement_jacobian = Eigen::MatrixXd::Zero(2 * num_obs_old_cones, 2 * n_landmarks + 3);
    std::vector<std::optional<EstimateConeInfo>> v(num_obs_old_cones);
    assignment = v;
}

std::vector<Old_cone_info> CSP::find_best_association_list() {

    backtracking_search(0);

    std::vector<Old_cone_info> ans;
    for(int i = 0; i < num_obs_old_cones; i++) {
        auto& elem = best_association_list[i].value();
        ans.push_back({all_variable_info[i].local_cone_position, all_variable_info[i].global_cone_position, all_variable_info[i].bearing, elem.index});
    }
    return ans;
}

void CSP::backtracking_search(int backtracking_index) {
    bool all_assigned = std::all_of(assignment.cbegin(), assignment.cend(), [](const std::optional<EstimateConeInfo>& opt) {return opt.has_value();});
    if(csp_logger.has_value() && all_assigned) {
        RCLCPP_INFO(csp_logger.value(), "all variables are assigned");
    }

    if(all_assigned) {

        if(csp_logger.has_value()) {
            RCLCPP_INFO(csp_logger.value(), "Found association list");
        }

        double cur_jcbb = compute_joint_compatibility(covariance_est, obs_cone_global_positions, 
            assignment, car_info.cur_pose, innovation_noise, num_obs_old_cones, n_landmarks, csp_logger);
        if (cur_jcbb < best_joint_compatibility) {
            best_joint_compatibility = cur_jcbb;
            best_association_list = assignment;

            for (int i = 0; i < num_obs_old_cones; i++) {
                Eigen::MatrixXd ith_msmt_jacobian = get_measurement_model_jacobian(car_info.cur_pose, best_association_list.at(i).value(), n_landmarks, csp_logger);
                best_association_list_measurement_jacobian.block(2 * i, 0, 2, 2 * n_landmarks + 3) = ith_msmt_jacobian;
            }

        }
        return;
    }
    if (csp_logger.has_value()) {
        RCLCPP_INFO(csp_logger.value(), "backtracking search index: %d", backtracking_index);
    }

    //TODO implement MRV and LCV for faster runtime
    /* This queue is used to store the indices for which observed cone ID to process
     * While it is useless right now, in the future we may consider using a priority
     * queue to sort the observed cones by the ones that have the smallest domain (MRV)*/
    queue<int> q;
    if (csp_logger.has_value()) {
        RCLCPP_INFO(csp_logger.value(), "backtracking_search: num_obs_old_cones: %d | assignment.size = %d",
                                    num_obs_old_cones, assignment.size());
    }
    for (int i = 0; i < num_obs_old_cones; i++) {
        if (!assignment[i].has_value())
            q.push(i);
    }

    if (csp_logger.has_value()) {
        RCLCPP_INFO(csp_logger.value(), "finished work queue update");
    }

    if(q.empty()) return;

    /* Get the observed cone variable to process */
    int cur_obs = q.front();

    auto& cur_domain = all_variable_info[cur_obs].domain;
    if (csp_logger.has_value()) {
        RCLCPP_INFO(csp_logger.value(), "all_variable_info size: %d | cur_obs: %d", all_variable_info.size(), cur_obs);
    }   
    /* Go through each domain element and try to assign */
    bool is_assigned = false;
    for(const std::pair<int, CSP::EstimateConeInfo>& pair : cur_domain) {
        /* Make the assignment */
        if (csp_logger.has_value()) {
            RCLCPP_INFO(csp_logger.value(), "Assigning %d to %d", cur_obs, pair.first);
        }
        assignment[cur_obs] = pair.second;

        /* Remove the other domain elements from the domain because we've made the assignment */
        for(const std::pair<int, CSP::EstimateConeInfo>& pair2 : cur_domain) {
            if(pair2.first != pair.first) { /* Updating the removal history for rollback */
                all_variable_info[cur_obs].removal_history.push({backtracking_index, pair2.second});
            }
        }
        /* Updating the domains and removing other elements */
        all_variable_info[cur_obs].domain.clear();
        all_variable_info[cur_obs].domain.insert(pair);

        association_list_t partial_assignment(assignment.begin(), assignment.begin()+backtracking_index + 1);
        
        if (csp_logger.has_value()) {
            RCLCPP_INFO(csp_logger.value(), "backtracking: before compute_joint_compatibility");
        }
        double compatibility = compute_joint_compatibility(covariance_est, obs_cone_global_positions, partial_assignment,
                                    car_info.cur_pose, innovation_noise, backtracking_index + 1, n_landmarks, csp_logger);
        if (csp_logger.has_value()) {
            RCLCPP_INFO(csp_logger.value(), "backtracking: after compute_joint_compatibility");
        }
        
        /* Continue the search and build off of the current association list */
        if( compatibility < JC_TH && ac3(backtracking_index)) {
            backtracking_search(backtracking_index + 1);
        }
        rollback(backtracking_index);
    }
}

void CSP::rollback(int index) {
    /* Restore the domain */
    for(auto info: all_variable_info) {
        while(!info.removal_history.empty() && info.removal_history.top().backtracking_index >= index) {
            auto& cur_removal = info.removal_history.top().removed_domain_info;
            info.domain.insert({cur_removal.index, cur_removal});
            info.removal_history.pop();
        }
    }

    /* Restore assignment */
    assignment[index] = std::nullopt;
}

bool CSP::enforce_arc_consistency(std::pair<int, int> arc_to_enforce, int backtracking_index) {
    if (csp_logger.has_value()) {
        RCLCPP_INFO(csp_logger.value(), "inside enforce_arc_consistency");
    }
    bool changed = false;
    auto& info_a = all_variable_info[arc_to_enforce.first];
    auto& info_b = all_variable_info[arc_to_enforce.second];
    for(const auto &paira: info_a.domain) {
        bool consistent = false;
        for (const auto &pairb: info_b.domain) {
            if(paira.first != pairb.first) {
                consistent = true;
                break;
            }
        }
        if(!consistent) {
            info_a.domain.erase(paira.first);
            info_a.removal_history.push({backtracking_index, paira.second});
            changed = true;
        }
    }
    return changed;
}

bool CSP::ac3(int index) {
    if (csp_logger.has_value()) {
        RCLCPP_INFO(csp_logger.value(), "inside ac3");
    }

    queue<pair<int, int>> arcs;
    for(int i = 0; i < num_obs_old_cones; i++) 
        for(int j = 0; j < num_obs_old_cones; j++)
            if (i != j)
                arcs.push({i, j});


    while(!arcs.empty()) {
        auto [xi, xj] = arcs.front();
        arcs.pop();
        if (enforce_arc_consistency({xi, xj}, index)) {
            if (all_variable_info[xi].domain.empty())
                return false;
            for (int k = 0; k < num_obs_old_cones; k++)
                if (k != xi)
                    arcs.push({k, xi});
        }
    }
    return true;
}

Eigen::MatrixXd CSP::get_best_hypothesis_msmt_jacobian() {
    return best_association_list_measurement_jacobian;
}
