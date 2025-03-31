#include "isam2_pkg.hpp"
using namespace std;
using namespace gtsam;
using namespace std::chrono;
using std::size_t;

slamISAM::slamISAM(optional<rclcpp::Logger> input_logger) {

    // Initializing SLAM Parameters
    parameters = ISAM2Params(ISAM2DoglegParams(),0.1,10,true);
    parameters.setFactorization("Cholesky");
    // parameters.enablePartialRelinearizationCheck = true;
    logger = input_logger;
    isam2 = gtsam::ISAM2(parameters);
    graph = gtsam::NonlinearFactorGraph();
    values = gtsam::Values();

    pose_num = 0;
    first_pose_added = false;
    n_landmarks = 0;


    
    LandmarkNoiseModel = gtsam::Vector(2);
    OdomNoiseModel = gtsam::Vector(3);
    PriorNoiseModel = gtsam::Vector(3);
    UnaryNoiseModel = gtsam::Vector(2);
    if (true) {

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
    } else {
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

        // Do not continue updating if the car is not moving. Only update during the 0th pose. 
        //TODO: consider when the car slows to a stop?

        if (logger.has_value()) {
            //RCLCPP_INFO(logger.value(), "||velocity|| = %f", norm2(Point2(velocity.x(), velocity.y())));
            //if (norm2(Point2(velocity.x(), velocity.y())) > VELOCITY_MOVING_TH) {
                //RCLCPP_INFO(logger.value(), "Car is moving");
            //} else {
                //RCLCPP_INFO(logger.value(), "Car stopped");
            //}

            //RCLCPP_INFO(logger.value(), "|angular velocity| = %f", abs(velocity.theta()));

        }

        BetweenFactor<Pose2> odom_factor = BetweenFactor<gtsam::Pose2>(X(pose_num - 1),
                                                                        X(pose_num),
                                                                        odometry,
                                                                        odom_model);
        cur_pose = gtsam::Pose2(new_pose.x(), new_pose.y(), new_pose.theta());
        graph.add(odom_factor);

        Pose2 imu_offset_global_odom = Pose2(global_odom.x() - offset_x, global_odom.y() - offset_y, global_odom.theta());
        graph.emplace_shared<UnaryFactor>(X(pose_num), imu_offset_global_odom, unary_model);
        values.insert(X(pose_num), new_pose);

        print_update_poses(prev_pose, new_pose, odometry, imu_offset_global_odom, logger);
    }

    isam2.update(graph, values);
    graph.resize(0);
    values.clear();

    // Pose2 est_pose = isam2.calculateEstimate(X(pose_num)).cast<Pose2>(); // Safe for pose_num == 0
    // RCLCPP_INFO(logger, "Diff: x: %.10f | y: %.10f", est_pose.x() - cur_pose.x(), est_pose.y() - cur_pose.y());


}

/* Cones represented by a tuple: the 1st element is the relative position
 * to the car
 * 2nd Point2 for new_cones represents the global position
 */
void slamISAM::update_landmarks(std::vector<Old_cone_info> &old_cones,
                        std::vector<New_cone_info> &new_cones,
                        gtsam::Pose2 &cur_pose) {


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
    for (std::size_t o = 0; o < old_cones.size(); o++) {
        Old_cone_info old_cone = old_cones.at(o);
        Point2 cone_pos_car_frame = old_cone.local_cone_pos;
        int min_id = old_cone.min_id;
        Rot2 b = Rot2::fromAngle(old_cone.bearing);
        double r = norm2(cone_pos_car_frame);

        graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), L(min_id),
                                        b,
                                        r,
                                        landmark_model));
    }
    isam2.update(graph, values);
    graph.resize(0);
    // values should be empty

    for (std::size_t n = 0; n < new_cones.size(); n++) {
        New_cone_info new_cone = new_cones.at(n);
        Point2 cone_pos_car_frame = new_cone.local_cone_pos;
        Rot2 b = Rot2::fromAngle(new_cone.bearing);
        double r = norm2(cone_pos_car_frame);

        Point2 cone_global_frame = new_cone.global_cone_pos;

        graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), L(n_landmarks),
                                        b,
                                        r,
                                        landmark_model));

        values.insert(L(n_landmarks), cone_global_frame);
        n_landmarks++;
    }
    
    /* NOTE: All values in graph must be in values parameter */

    values.insert(X(pose_num), cur_pose);
    Values optimized_val = LevenbergMarquardtOptimizer(graph, values).optimize();
    optimized_val.erase(X(pose_num));
    isam2.update(graph, optimized_val);
    

    graph.resize(0); //Not resizing your graph will result in long update times
    values.clear();


}


/**
 * @brief step takes in info about the observed cones and the odometry info
 *        and performs an update step to our iSAM2 model. 
 */
void slamISAM::step(Pose2 global_odom, std::vector<Point2> &cone_obs,
            std::vector<Point2> &cone_obs_blue, std::vector<Point2> &cone_obs_yellow,
            std::vector<Point2> &orange_ref_cones, Pose2 velocity,
            double dt) {

    // print_step_input(logger, global_odom, cone_obs, cone_obs_blue, cone_obs_yellow, orange_ref_cones, velocity, dt);
    log_step_inputs(logger, global_odom, cone_obs, cone_obs_blue, cone_obs_yellow, orange_ref_cones, velocity, dt);


    if (n_landmarks > 0)
    {
        auto start_step  = high_resolution_clock::now();
        auto dur_betw_step = duration_cast<milliseconds>(start_step - end);
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "End of prev step to cur step: %ld", dur_betw_step.count());
        }
    }

    start = high_resolution_clock::now();

    Pose2 cur_pose = Pose2(0, 0, 0);
    Pose2 prev_pose = Pose2(0, 0, 0);
    bool is_moving = false;
    bool is_turning = false;

    determine_movement(is_moving, is_turning, velocity);
    
    /*Quit the update step if the car is not moving*/ 
    if (!is_moving && pose_num > 0) {
        return;
    }

    auto start_update_poses = high_resolution_clock::now();
    update_poses(cur_pose, prev_pose, global_odom, velocity, dt, logger);
    auto end_update_poses = high_resolution_clock::now();
    auto dur_update_poses = duration_cast<milliseconds>(end_update_poses - start_update_poses);

    if(logger.has_value()) {
        RCLCPP_INFO(logger.value(), "update_poses time: %ld", dur_update_poses.count());
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
        RCLCPP_INFO(logger.value(), "loop closure time: %ld", dur_loop_closure.count());
    }

    if (loop_closure) {
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "Loop closure detected. No longer updating");
        }
    }

    /**** Retrieve the old cones SLAM estimates & marginal covariance matrices ****/
    if (!loop_closure) {
        auto start_est_retrieval = high_resolution_clock::now();
        std::vector<Point2> slam_est = {};
        for (int i = 0; i < n_landmarks; i++) {
            slam_est.push_back(isam2.calculateEstimate(L(i)).cast<Point2>());
        }

        std::vector<MatrixXd> slam_mcov = {};
        for (int i = 0; i < n_landmarks; i++) {
            slam_mcov.push_back(isam2.marginalCovariance(L(i)));
        }
        auto end_est_retrieval = high_resolution_clock::now();
        auto dur_est_retrieval = duration_cast<milliseconds>(end_est_retrieval - start_est_retrieval);
        if(logger.has_value()) {
            RCLCPP_INFO(logger.value(), "est_retrieval time: %ld", dur_est_retrieval.count());
        }


        /**** Data association ***/
        std::vector<Old_cone_info> old_cones = {};
        std::vector<New_cone_info> new_cones = {};


        auto start_DA = high_resolution_clock::now();
        data_association(old_cones, new_cones, cur_pose, prev_pose, is_turning,
                            cone_obs, logger, slam_est, slam_mcov);
        auto end_DA = high_resolution_clock::now();
        auto dur_DA = duration_cast<milliseconds>(end_DA - start_DA);
        if(logger.has_value()) {
            RCLCPP_INFO(logger.value(), "Data association time: %ld", dur_DA.count());
	    RCLCPP_INFO(logger.value(), "Num cone_obs: %ld", cone_obs.size());
        }

        auto start_update_landmarks = high_resolution_clock::now();
        update_landmarks(old_cones, new_cones, cur_pose);
        auto end_update_landmarks = high_resolution_clock::now();
        auto dur_update_landmarks = duration_cast<milliseconds>(end_update_landmarks - start_update_landmarks);
        if(logger.has_value()) {
            RCLCPP_INFO(logger.value(), "update_landmarks time: %ld", dur_update_landmarks.count());
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
        RCLCPP_INFO(logger.value(), "vis_setup time: %ld", dur_vis_setup.count());
    } 

    end = high_resolution_clock::now();
    


    auto dur_step_call = duration_cast<milliseconds>(end - start);
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "SLAM run step | Step call time: %ld\n", dur_step_call.count());
    }

    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "pose_num: %d | n_landmarks: %d\n\n", pose_num - 1, n_landmarks);
    }
}

void slamISAM::print_estimates() {
    auto estimate = isam2.calculateEstimate();
    estimate.print("Estimate:");
}




