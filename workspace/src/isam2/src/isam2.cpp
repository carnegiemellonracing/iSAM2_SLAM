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
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "Processing first pose");
        }

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
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "Finished processing first pose");
        }
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
void slamISAM::update_landmarks(std::vector<Old_cone_info> &old_cones,
                                std::vector<New_cone_info> &new_cones,
                                int &n_landmarks, ConeColor color,
                                Pose2 &cur_pose)
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


        gtsam::Symbol landmark_symbol;

        switch (color) {
            case ConeColor::Blue:
                landmark_symbol = BLUE_L(min_id);
                break;
            case ConeColor::Yellow:
                landmark_symbol = YELLOW_L(min_id);
                break;
        }

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

        gtsam::Symbol landmark_symbol;
        switch (color) {
            case ConeColor::Blue:
                landmark_symbol = BLUE_L(n_landmarks);
                break;
            case ConeColor::Yellow:
                landmark_symbol = YELLOW_L(n_landmarks);
                break;
        }
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
        auto dur_betw_step = duration_cast<milliseconds>(start_step - end);
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "--------End of prev step to cur step: %ld--------\n\n", dur_betw_step.count());
        }
    }

    start = high_resolution_clock::now();
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "--------Start of SLAM Step--------");
    }

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

    if(logger.has_value()) {
        RCLCPP_INFO(logger.value(), "\tUpdate_poses time: %ld", dur_update_poses.count());
    }


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
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "\tLoop closure time: %ld", dur_loop_closure.count());
    }

    if (loop_closure) {
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "\tLoop closure detected. No longer updating");
        }
    }

    /**** Retrieve the old cones SLAM estimates & marginal covariance matrices ****/
    if (!loop_closure) {
        auto start_est_retrieval = high_resolution_clock::now();
        std::vector<gtsam::Point2> blue_slam_est = {};
        for (int i = 0; i < blue_n_landmarks; i++) {
            blue_slam_est.push_back(isam2.calculateEstimate(BLUE_L(i)).cast<Point2>());
        }
        std::vector<Eigen::MatrixXd> blue_slam_mcov = {};
        for (int i = 0; i < blue_n_landmarks; i++) {
            blue_slam_mcov.push_back(isam2.marginalCovariance(BLUE_L(i)));
        }

        std::vector<gtsam::Point2> yellow_slam_est = {};
        for (int i = 0; i < yellow_n_landmarks; i++) {
            yellow_slam_est.push_back(isam2.calculateEstimate(YELLOW_L(i)).cast<Point2>());
        }
        std::vector<Eigen::MatrixXd> yellow_slam_mcov = {};
        for (int i = 0; i < yellow_n_landmarks; i++) {
            yellow_slam_mcov.push_back(isam2.marginalCovariance(YELLOW_L(i)));
        }

        auto end_est_retrieval = high_resolution_clock::now();
        auto dur_est_retrieval = duration_cast<milliseconds>(end_est_retrieval - start_est_retrieval);
        if(logger.has_value()) {
            RCLCPP_INFO(logger.value(), "\tEst_retrieval time: %ld", dur_est_retrieval.count());
        }


        /**** Data association ***/
        std::vector<Old_cone_info> blue_old_cones = {};
        std::vector<New_cone_info> blue_new_cones = {};
        std::vector<Old_cone_info> yellow_old_cones = {};
        std::vector<New_cone_info> yellow_new_cones = {};

        auto start_DA = high_resolution_clock::now();
        data_association(blue_old_cones, blue_new_cones, cur_pose, prev_pose, is_turning,
                            cone_obs_blue, logger, blue_slam_est, blue_slam_mcov);
        data_association(yellow_old_cones, yellow_new_cones, cur_pose, prev_pose, is_turning,
                            cone_obs_yellow, logger, yellow_slam_est, yellow_slam_mcov);

        auto end_DA = high_resolution_clock::now();
        auto dur_DA = duration_cast<milliseconds>(end_DA - start_DA);
        if(logger.has_value()) {
            RCLCPP_INFO(logger.value(), "\tData association time: %ld", dur_DA.count());
        }

        auto start_update_landmarks = high_resolution_clock::now();
        update_landmarks(blue_old_cones, blue_new_cones, blue_n_landmarks, ConeColor::Blue, cur_pose);
        update_landmarks(yellow_old_cones, yellow_new_cones, yellow_n_landmarks, ConeColor::Yellow, cur_pose);
        auto end_update_landmarks = high_resolution_clock::now();
        auto dur_update_landmarks = duration_cast<milliseconds>(end_update_landmarks - start_update_landmarks);

        if(logger.has_value()) {
            RCLCPP_INFO(logger.value(), "\tUpdate_landmarks time: %ld", dur_update_landmarks.count());
        }
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

    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "\tVis_setup time: %ld", dur_vis_setup.count());
    } 

    end = high_resolution_clock::now();
    


    auto dur_step_call = duration_cast<milliseconds>(end - start);
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "\tSLAM run step | Step call time: %ld\n", dur_step_call.count());
    }

    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "\tpose_num: %d | blue_n_landmarks: %d | yellow_n_landmarks : %d", 
                        pose_num - 1, blue_n_landmarks, yellow_n_landmarks);
    }
}

void slamISAM::print_estimates() {
    auto estimate = isam2.calculateEstimate();
    estimate.print("Estimate:");
}




