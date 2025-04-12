#include "isam2_pkg.hpp"
using namespace std;
using namespace gtsam;
using namespace std::chrono;
using std::size_t;

slamISAM::slamISAM(std::optional<rclcpp::Logger> input_logger, std::optional<NoiseInputs> &yaml_noise_inputs) {

    // Initializing SLAM Parameters
    parameters = ISAM2Params(ISAM2DoglegParams(),0.1,10,true);
    parameters.setFactorization("QR");
    logger = input_logger;
    isam2 = gtsam::ISAM2(parameters);
    graph = gtsam::NonlinearFactorGraph();
    values = gtsam::Values();

    pose_num = 0;
    first_pose_added = false;
    blue_n_landmarks = 0;
    yellow_n_landmarks = 0;


    
    LandmarkNoiseModel = gtsam::Vector(2);
    OdomNoiseModel = gtsam::Vector(3);
    PriorNoiseModel = gtsam::Vector(3);
    UnaryNoiseModel = gtsam::Vector(2);
    if (!yaml_noise_inputs.has_value()) {
        switch (RunSettings::Real) {
            case RunSettings::Real:
                LandmarkNoiseModel(0) = BEARING_STD_DEV; 
                LandmarkNoiseModel(1) = RANGE_STD_DEV; 

                OdomNoiseModel(0) = IMU_X_STD_DEV;
                OdomNoiseModel(1) = IMU_Y_STD_DEV; 
                OdomNoiseModel(2) = IMU_HEADING_STD_DEV; 
                // used to be all 0s for EUFS_SIM
                PriorNoiseModel(0) = IMU_X_STD_DEV;
                PriorNoiseModel(1) = IMU_Y_STD_DEV;
                PriorNoiseModel(2) = IMU_HEADING_STD_DEV;

                UnaryNoiseModel(0) = GPS_X_STD_DEV;
                UnaryNoiseModel(1) = GPS_Y_STD_DEV;
                break;
            case RunSettings::EUFSSim:
                LandmarkNoiseModel(0) = EUFS_SIM_BEARING_STD_DEV; 
                LandmarkNoiseModel(1) = EUFS_SIM_RANGE_STD_DEV; 

                OdomNoiseModel(0) = EUFS_SIM_IMU_X_STD_DEV;
                OdomNoiseModel(1) = EUFS_SIM_IMU_Y_STD_DEV; 
                OdomNoiseModel(2) = EUFS_SIM_IMU_HEADING_STD_DEV; 
                // used to be all 0s for EUFS_SIM
                PriorNoiseModel(0) = EUFS_SIM_IMU_X_STD_DEV;
                PriorNoiseModel(1) = EUFS_SIM_IMU_Y_STD_DEV;
                PriorNoiseModel(2) = EUFS_SIM_IMU_HEADING_STD_DEV;

                UnaryNoiseModel(0) = EUFS_SIM_GPS_X_STD_DEV;
                UnaryNoiseModel(1) = EUFS_SIM_GPS_Y_STD_DEV;
                break;
            case RunSettings::ControlsSim:
                LandmarkNoiseModel(0) = CONTROLS_BEARING_STD_DEV; 
                LandmarkNoiseModel(1) = CONTROLS_RANGE_STD_DEV; 

                OdomNoiseModel(0) = CONTROLS_IMU_X_STD_DEV;
                OdomNoiseModel(1) = CONTROLS_IMU_Y_STD_DEV; 
                OdomNoiseModel(2) = CONTROLS_IMU_HEADING_STD_DEV; 
                // used to be all 0s for EUFS_SIM
                PriorNoiseModel(0) = CONTROLS_IMU_X_STD_DEV;
                PriorNoiseModel(1) = CONTROLS_IMU_Y_STD_DEV;
                PriorNoiseModel(2) = CONTROLS_IMU_HEADING_STD_DEV;

                UnaryNoiseModel(0) = CONTROLS_GPS_X_STD_DEV;
                UnaryNoiseModel(1) = CONTROLS_GPS_Y_STD_DEV;
                break;
        }
    } else {
        LandmarkNoiseModel(0) = yaml_noise_inputs.value().yaml_bearing_std_dev; 
        LandmarkNoiseModel(1) = yaml_noise_inputs.value().yaml_range_std_dev; 

        OdomNoiseModel(0) = yaml_noise_inputs.value().yaml_imu_x_std_dev;
        OdomNoiseModel(1) = yaml_noise_inputs.value().yaml_imu_y_std_dev; 
        OdomNoiseModel(2) = yaml_noise_inputs.value().yaml_imu_heading_std_dev; 
        // used to be all 0s for EUFS_SIM
        PriorNoiseModel(0) = yaml_noise_inputs.value().yaml_imu_x_std_dev;
        PriorNoiseModel(1) = yaml_noise_inputs.value().yaml_imu_y_std_dev;
        PriorNoiseModel(2) = yaml_noise_inputs.value().yaml_imu_heading_std_dev;

        UnaryNoiseModel(0) = yaml_noise_inputs.value().yaml_gps_x_std_dev;          
        UnaryNoiseModel(1) = yaml_noise_inputs.value().yaml_gps_y_std_dev;
    }



    landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);
    odom_model = noiseModel::Diagonal::Sigmas(OdomNoiseModel);
    prior_model = noiseModel::Diagonal::Sigmas(PriorNoiseModel);
    unary_model = noiseModel::Diagonal::Sigmas(UnaryNoiseModel);

    // Resetting the log file for gtsam
    ofstream init_log_reset_stream;
    init_log_reset_stream.open("/home/danielnguyen/gtsam_log.txt");
    init_log_reset_stream << "STARTING LOG" << endl;
    init_log_reset_stream.close();

    ofstream init_step_input_stream;
    init_step_input_stream.open(STEP_INPUT_FILE, ofstream::out | ofstream::trunc);
    init_step_input_stream.close();


    loop_closure = false;
    new_lap = false;
    lap_count = 0;
}

gtsam::Symbol slamISAM::X(int robot_pose_id) {
    return Symbol('x', robot_pose_id);
}

gtsam::Symbol slamISAM::BLUE_L(int cone_pose_id) {
    return Symbol('b', cone_pose_id);
}

gtsam::Symbol slamISAM::YELLOW_L(int cone_pose_id) {
    return Symbol('y', cone_pose_id);
}
/**
 * @brief Updates the poses in the SLAM model. 
 * 
 * @param cur_pose: the current pose of the car
 * @param prev_pose: the previous pose of the car
 * @param global_odom: the global odometry measurement
 * @param velocity: the velocity of the car
 * @param dt: the change in time
 * @param logger: the logger
 * 
 * @return: void
 */
void slamISAM::update_poses(Pose2 &cur_pose, Pose2 &prev_pose, Pose2 &global_odom,
            Pose2 &velocity,double dt, optional<rclcpp::Logger> logger) {
    /* Adding poses to the SLAM factor graph */
    double offset_x = 0;
    double offset_y = 0;
    calc_offset_imu_to_car_center(offset_x, offset_y, global_odom.theta());

    if (pose_num == 0)
    {
        log_string(logger, "Processing first pose", DEBUG_POSES);

        PriorFactor<Pose2> prior_factor = PriorFactor<Pose2>(X(0),
                                                            global_odom, prior_model);
        //add prior
        //TODO: need to record the initial bearing because it could be erroneous
        graph.add(prior_factor);


        cur_pose = Pose2(-offset_x, -offset_y, global_odom.theta());
        first_pose = cur_pose;
        values.insert(X(0), Pose2(cur_pose.x(), cur_pose.y(), global_odom.theta()));

        first_pose_added = true;

        //ASSUMES THAT YOU SEE ORANGE CONES ON YOUR FIRST MEASUREMENT OF LANDMARKS
        //Add orange cone left and right
        //hopefully it's only 2 cones
        log_string(logger, "Finished processing first pose", DEBUG_POSES);
    }
    else
    {
        //create a factor between current and previous robot pose
        //add odometry estimates
        //Motion model
        Pose2 new_pose = Pose2(0, 0, 0);
        Pose2 odometry = Pose2(0, 0, 0);

        prev_pose = isam2.calculateEstimate(X(pose_num - 1)).cast<Pose2>();

        //global_odom holds our GPS measurements
        velocity_motion_model(new_pose, odometry, velocity, dt, prev_pose, global_odom);

        /* Do not continue updating if the car is not moving. Only update during the 0th pose. */
        

        BetweenFactor<Pose2> odom_factor = BetweenFactor<gtsam::Pose2>(X(pose_num - 1),
                                                                        X(pose_num),
                                                                        odometry,
                                                                        odom_model);
        cur_pose = gtsam::Pose2(new_pose.x(), new_pose.y(), new_pose.theta());
        graph.add(odom_factor);

        Pose2 imu_offset_global_odom = Pose2(global_odom.x() - offset_x, global_odom.y() - offset_y, global_odom.theta());
        graph.emplace_shared<UnaryFactor>(X(pose_num), imu_offset_global_odom, unary_model);
        values.insert(X(pose_num), new_pose);
    }

    isam2.update(graph, values);
    graph.resize(0);
    values.clear();

}




/**
 * @brief Updates the landmarks in the SLAM model. This function
 * is used to update the landmarks for a given cone color at a time.
 * This function will update the SLAM model accordingly using the 
 * cone information stored in old_cones and new_cones.
 * 
 * @param old_cones: the old cones
 * @param new_cones: the new cones
 * @param n_landmarks: the number of landmarks
 * @param color: the color of the cones
 * @param cur_pose: the current pose of the car
 * 
 * @return: void
 */
int slamISAM::update_landmarks(const std::vector<Old_cone_info> &old_cones,
                                const std::vector<New_cone_info> &new_cones,
                                int n_landmarks,
                                gtsam::Pose2 &cur_pose, gtsam::Symbol (*cone_key)(int))
{

    /* Bearing range factor will need
     * Types for car pose to landmark node (Pose2, Point2)
     * Bearing of type Rot2 (Rot2 fromAngle)
     * Range of type double
     * Look at PlanarSLAM example in gtsam
     *
     * When adding values:
     * insert Point2 for the cones and their actual location
     *
     */
    for (std::size_t o = 0; o < old_cones.size(); o++)
    {
        Point2 cone_pos_car_frame = old_cones.at(o).local_cone_pos;
        int min_id = (old_cones.at(o)).min_id;
        Rot2 b = Rot2::fromAngle((old_cones.at(o)).bearing);
        double r = norm2(cone_pos_car_frame);


        gtsam::Symbol landmark_symbol = cone_key(min_id);
        graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), landmark_symbol,
                                                    b,
                                                    r,
                                                    landmark_model));
    }
    isam2.update(graph, values);
    graph.resize(0);
    // values should be empty

    for (std::size_t n = 0; n < new_cones.size(); n++)
    {
        Point2 cone_pos_car_frame = (new_cones.at(n).local_cone_pos);
        Rot2 b = Rot2::fromAngle((new_cones.at(n)).bearing);
        double r = norm2(cone_pos_car_frame);

        Point2 cone_global_frame = (new_cones.at(n).global_cone_pos);

        gtsam::Symbol landmark_symbol = cone_key(n_landmarks);
        graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), landmark_symbol,
                                                    b,
                                                    r,
                                                    landmark_model));

        values.insert(landmark_symbol, cone_global_frame);
        n_landmarks++;
    }

    /* NOTE: All values in graph must be in values parameter */

    values.insert(X(pose_num), cur_pose);
    Values optimized_val = LevenbergMarquardtOptimizer(graph, values).optimize();
    optimized_val.erase(X(pose_num));
    isam2.update(graph, optimized_val);

    graph.resize(0); // Not resizing your graph will result in long update times
    values.clear();
    return n_landmarks;
}

void slamISAM::cone_proximity_updates(std::size_t pivot, std::size_t n_landmarks,
                                    std::vector<gtsam::Point2> &color_slam_est, std::vector<Eigen::MatrixXd>& color_slam_mcov, 
                                    gtsam::Symbol (*cone_key)(int)) {

    for (std::size_t i = std::max(pivot - LOOK_RADIUS, 0uz); i < std::min(pivot + LOOK_RADIUS, n_landmarks); i++) {
        gtsam::Point2 updated_est = isam2.calculateEstimate(cone_key(i)).cast<Point2>();
        color_slam_est.at(i) = updated_est;
    }

    for (std::size_t i = std::max(pivot- LOOK_RADIUS, 0uz); i < std::min(pivot + LOOK_RADIUS, n_landmarks); i++) {
        Eigen::MatrixXd updated_mcov = isam2.marginalCovariance(cone_key(i));
        color_slam_mcov.at(i) = updated_mcov;
    }
}

std::pair<int, int> slamISAM::update_slam_est_and_mcov_with_old(std::vector<Old_cone_info>& old_cones,
                                    std::vector<gtsam::Point2>& color_slam_est, 
                                    std::vector<Eigen::MatrixXd>& color_slam_mcov, gtsam::Symbol(*cone_key)(int)) {

    int lowest_id = INT_MAX;
    int highest_id = -1;
    for (int i=0; i < old_cones.size(); i++) {
        Old_cone_info curr = old_cones.at(i);
        gtsam::Point2 updated_est = isam2.calculateEstimate(cone_key(curr.min_id)).cast<Point2>();
        color_slam_est.at(curr.min_id) = updated_est;

        if (curr.min_id < lowest_id) {
            lowest_id = curr.min_id;
        }
        if (curr.min_id > highest_id) {
            highest_id = curr.min_id;
        }
    }

    for (int i=0; i < old_cones.size(); i++) {
        Old_cone_info curr = old_cones.at(i);
        Eigen::MatrixXd updated_mcov = isam2.marginalCovariance(cone_key(curr.min_id));
        color_slam_mcov.at(curr.min_id) = updated_mcov;
    }                                     
    assert(highest_id >= 0 && lowest_id < std::max(blue_n_landmarks, yellow_n_landmarks));
    std::pair<int, int> lo_and_hi(lowest_id, highest_id);
    return lo_and_hi;
}

void slamISAM::update_slam_est_and_mcov_with_new(int old_n_landmarks, int new_n_landmarks, 
                                                                std::vector<gtsam::Point2>& color_slam_est, 
                                                                std::vector<Eigen::MatrixXd>& color_slam_mcov, 
                                                                gtsam::Symbol(*cone_key)(int)) {
    for (int i = old_n_landmarks; i < new_n_landmarks; i++) {
        gtsam::Point2 new_est = isam2.calculateEstimate(cone_key(i)).cast<Point2>();
        color_slam_est.push_back(new_est);
    }

    for (int i = old_n_landmarks; i < new_n_landmarks; i++) {
        Eigen::MatrixXd new_mcov = isam2.marginalCovariance(cone_key(i));
        color_slam_mcov.push_back(new_mcov);
    }
}

void slamISAM::update_all_est_mcov(int n_landmarks, std::vector<gtsam::Point2>& color_slam_est, 
                                                    std::vector<Eigen::MatrixXd>& color_slam_mcov, 
                                                    gtsam::Symbol(*cone_key)(int)) {
    color_slam_est.clear();
    color_slam_mcov.clear();
    for (int i= 0; i < n_landmarks; i++) {
        color_slam_est.push_back(isam2.calculateEstimate(cone_key(i)).cast<Point2>());
    }
    for (int i= 0; i < n_landmarks; i++) {
        color_slam_mcov.push_back(isam2.marginalCovariance(cone_key(i)));
    }
}

void slamISAM::update_beginning(std::vector<gtsam::Point2>& color_slam_est, 
                                                    std::vector<Eigen::MatrixXd>& color_slam_mcov, 
                                                    gtsam::Symbol(*cone_key)(int)) {
    for (int i = 0; i < UPDATE_START_N; i++) {
        color_slam_est.at(i) = isam2.calculateEstimate(cone_key(i)).cast<gtsam::Point2>();
    }

    for (int i = 0; i < UPDATE_START_N; i++) {
        color_slam_mcov.at(i) = isam2.marginalCovariance(cone_key(i));
    }
}

/** 
 * @brief Performs a series of updates, either over the entire map,  
 * over the cones at the beginning, or in a sliding window approach. 
 * 
 * @param sliding_window A flag to decide whether to update the 
 * landmarks in a sliding window approach, or to only update the 
 * landmarks at the beginning
 */
void slamISAM::stability_update(bool sliding_window) {
    /* For numerical stability */
    if (!(blue_n_landmarks >= MIN_CONES_UPDATE_ALL && yellow_n_landmarks >= MIN_CONES_UPDATE_ALL)) {
        update_all_est_mcov(blue_n_landmarks, blue_slam_est, blue_slam_mcov, BLUE_L);
        update_all_est_mcov(yellow_n_landmarks, yellow_slam_est, yellow_slam_mcov, YELLOW_L);
    } else {
        if (sliding_window) {
            cone_proximity_updates(blue_checkpoint_id, blue_n_landmarks, blue_slam_est, blue_slam_mcov, BLUE_L); 
            cone_proximity_updates(yellow_checkpoint_id, yellow_n_landmarks, yellow_slam_est, yellow_slam_mcov, YELLOW_L); 
            blue_checkpoint_id = (blue_checkpoint_id + WINDOW_UPDATE) % (blue_n_landmarks - WINDOW_UPDATE);
            yellow_checkpoint_id = (yellow_checkpoint_id + WINDOW_UPDATE) % (yellow_n_landmarks - WINDOW_UPDATE);
        } else if (blue_n_landmarks > checkpoint_to_update_beginning && yellow_n_landmarks > checkpoint_to_update_beginning) {
            update_beginning(blue_slam_est, blue_slam_mcov, BLUE_L);
            update_beginning(yellow_slam_est, yellow_slam_mcov, YELLOW_L);
            checkpoint_to_update_beginning += UPDATE_START_N;
        }
    }
}

/**
 * @brief Processes odometry information and cone information 
 * to perform an update step on the SLAM model. 
 * 
 * @param global_odom: the global odometry measurement
 * @param cone_obs: the observed cones
 * @param cone_obs_blue: the observed blue cones
 * @param cone_obs_yellow: the observed yellow cones
 * @param orange_ref_cones: the orange reference cones
 * @param velocity: the velocity of the car
 * @param dt: the change in time
 * 
 * @return: void
 */
void slamISAM::step(Pose2 global_odom, std::vector<Point2> &cone_obs,
            std::vector<Point2> &cone_obs_blue, std::vector<Point2> &cone_obs_yellow,
            std::vector<Point2> &orange_ref_cones, Pose2 velocity,
            double dt) {

    if (blue_n_landmarks + yellow_n_landmarks > 0)
    {
        auto start_step  = high_resolution_clock::now();
        auto dur_betw_step = duration_cast<milliseconds>(start_step - start);
        log_string(logger, fmt::format("--------End of prev step. Time between step calls: {}--------\n\n", dur_betw_step.count()), DEBUG_STEP);
    }

    start = high_resolution_clock::now();
    
    log_string(logger, "--------Start of SLAM Step--------", DEBUG_STEP);

    Pose2 cur_pose = Pose2(0, 0, 0);
    Pose2 prev_pose = Pose2(0, 0, 0);
    bool is_moving = false;
    bool is_turning = false;

    determine_movement(is_moving, is_turning, velocity);
    
    /*Quit the update step if the car is not moving*/ 
    if (!is_moving && pose_num > 0) {
        return;
    }

    /**** Update the car pose ****/
    auto start_update_poses = high_resolution_clock::now();
    update_poses(cur_pose, prev_pose, global_odom, velocity, dt, logger);
    auto end_update_poses = high_resolution_clock::now();
    auto dur_update_poses = duration_cast<milliseconds>(end_update_poses - start_update_poses);

    // std::ostringstream ss;
    // ss << "\tUpdate_poses time: {}" << dur_update_poses.count();
    log_string(logger, fmt::format("\tUpdate_poses time: {}", dur_update_poses.count()) , true);
    


    /**** Perform loop closure ****/
    auto start_loop_closure = high_resolution_clock::now();
    bool prev_new_lap_value = new_lap;
    new_lap = detect_loop_closure(cur_pose, first_pose, pose_num, logger);

    if (!loop_closure && new_lap) {
        loop_closure = true;
    }

    bool completed_new_lap = prev_new_lap_value && !new_lap && !start_pose_in_front(cur_pose, first_pose, logger);
    if (completed_new_lap) {
        lap_count++;
    }

    auto end_loop_closure = high_resolution_clock::now();
    auto dur_loop_closure = duration_cast<milliseconds>(end_loop_closure - start_loop_closure);
    log_string(logger, fmt::format("\tLoop closure time: {}", dur_loop_closure.count()), DEBUG_STEP);

    if (loop_closure) {
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "\tLoop closure detected. No longer updating");
        }
    }

    /**** Retrieve the old cones SLAM estimates & marginal covariance matrices ****/
    if (!loop_closure) {

        /**** Data association ****/
        std::vector<Old_cone_info> blue_old_cones = {};
        std::vector<New_cone_info> blue_new_cones = {};
        std::vector<Old_cone_info> yellow_old_cones = {};
        std::vector<New_cone_info> yellow_new_cones = {};

        auto start_DA = high_resolution_clock::now();
        /* For numerical stability */
        stability_update(false); 
        
        data_association(blue_old_cones, blue_new_cones, cur_pose, prev_pose, is_turning,
                            cone_obs_blue, logger, blue_slam_est, blue_slam_mcov);
        data_association(yellow_old_cones, yellow_new_cones, cur_pose, prev_pose, is_turning,
                            cone_obs_yellow, logger, yellow_slam_est, yellow_slam_mcov);
        
        auto end_DA = high_resolution_clock::now();
        auto dur_DA = duration_cast<milliseconds>(end_DA - start_DA);
        log_string(logger, fmt::format("\tData association time: {}", dur_DA.count()), DEBUG_STEP);

        auto start_update_landmarks = high_resolution_clock::now();

        int old_blue_n_landmarks = blue_n_landmarks;
        int old_yellow_n_landmarks = yellow_n_landmarks;
        log_string(logger, fmt::format("\t\tStarted updating isam2 model with new and old cones"), DEBUG_STEP);

        blue_n_landmarks = update_landmarks(blue_old_cones, blue_new_cones, blue_n_landmarks, cur_pose, BLUE_L);
        yellow_n_landmarks = update_landmarks(yellow_old_cones, yellow_new_cones, yellow_n_landmarks, cur_pose, YELLOW_L);
        log_string(logger, fmt::format("\t\tFinished updating isam2 model with new and old cones"), DEBUG_STEP);


        /* Updating slam_est and slam_mcov vectors with the old cone information */         
        if (blue_n_landmarks >= MIN_CONES_UPDATE_ALL && yellow_n_landmarks >= MIN_CONES_UPDATE_ALL) {
            std::pair<int, int> blue_lo_and_hi = update_slam_est_and_mcov_with_old(blue_old_cones, 
                                                                            blue_slam_est, blue_slam_mcov, BLUE_L);
            std::pair<int, int> yellow_lo_and_hi = update_slam_est_and_mcov_with_old(yellow_old_cones, 
                                                                            yellow_slam_est, yellow_slam_mcov, YELLOW_L);
            int lowest_blue_id = blue_lo_and_hi.first;
            int highest_blue_id = blue_lo_and_hi.second;
            int lowest_yellow_id =  yellow_lo_and_hi.first;
            int highest_yellow_id = yellow_lo_and_hi.second;       

            /* Updating slam_est and slam_mcov with the new cone information */
            update_slam_est_and_mcov_with_new(old_blue_n_landmarks, blue_n_landmarks, blue_slam_est, blue_slam_mcov, BLUE_L);
            update_slam_est_and_mcov_with_new(old_yellow_n_landmarks, yellow_n_landmarks, yellow_slam_est, yellow_slam_mcov, YELLOW_L);


            /* Extra updates */
            cone_proximity_updates(lowest_blue_id, blue_n_landmarks, blue_slam_est, blue_slam_mcov, BLUE_L);
            cone_proximity_updates(highest_blue_id, blue_n_landmarks, blue_slam_est, blue_slam_mcov, BLUE_L);
            cone_proximity_updates(lowest_yellow_id, yellow_n_landmarks, yellow_slam_est, yellow_slam_mcov, YELLOW_L);
            cone_proximity_updates(highest_yellow_id,yellow_n_landmarks, yellow_slam_est, yellow_slam_mcov, YELLOW_L);
        }

        auto end_update_landmarks = high_resolution_clock::now();
        auto dur_update_landmarks = duration_cast<milliseconds>(end_update_landmarks - start_update_landmarks);

        log_string(logger, fmt::format("\tUpdate_landmarks time: {}", dur_update_landmarks.count()), DEBUG_STEP);
    }

    pose_num++;



    /* Logging estimates for visualization */
    auto start_vis_setup = high_resolution_clock::now();
    ofstream ofs;
    
    ofs.open(ESTIMATES_FILE, std::ofstream::out | std::ofstream::trunc);
    streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    cout.rdbuf(ofs.rdbuf());

    auto estimate = isam2.calculateEstimate();
    estimate.print("Estimate:");

    ofs.close();
    cout.rdbuf(coutbuf); //reset to standard output again
    auto end_vis_setup = high_resolution_clock::now();
    auto dur_vis_setup = duration_cast<milliseconds>(end_vis_setup - start_vis_setup);

    log_string(logger, fmt::format("\tVis_setup time: {}", dur_vis_setup.count()), DEBUG_VIZ);

    end = high_resolution_clock::now();
    


    auto dur_step_call = duration_cast<milliseconds>(end - start);
    log_string(logger, fmt::format("\tSLAM run step | Step call time: {}\n", dur_step_call.count()), DEBUG_STEP);

    log_string(logger, fmt::format("\tpose_num: {} | blue_n_landmarks: {} | yellow_n_landmarks : {}", 
                        pose_num - 1, blue_n_landmarks, yellow_n_landmarks), DEBUG_STEP);
}

void slamISAM::print_estimates() {
    auto estimate = isam2.calculateEstimate();
    estimate.print("Estimate:");
}




