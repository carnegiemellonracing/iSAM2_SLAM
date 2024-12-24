#include "isam2_pkg.hpp"

slamISAM::slamISAM(optional<rclcpp::Logger> input_logger) {

    // Initializing SLAM Parameters
    parameters = ISAM2Params(ISAM2DoglegParams(),0.1,10,true);
    parameters.setFactorization("QR");
    // parameters.enablePartialRelinearizationCheck = true;
    logger = input_logger;
    isam2 = gtsam::ISAM2(parameters);
    graph = gtsam::NonlinearFactorGraph();
    values = gtsam::Values();

    pose_num = 0;
    first_pose_added = false;
    n_landmarks = 0;
    landmark_est = std::vector<gtsam::Pose2>();

    LandmarkNoiseModel = gtsam::Vector(2);
    // used to be 0.01 for real data
    // 0 for EUFS_SIM
    //TODO: have a different noise model at the beginning
    LandmarkNoiseModel(0) = 0;
    LandmarkNoiseModel(1) = 0;
    landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);

    // used to be all 0s for EUFS_SIM
    PriorNoiseModel = gtsam::Vector(3);
    PriorNoiseModel(0) = 0;
    PriorNoiseModel(1) = 0;
    PriorNoiseModel(2) = 0;

    prior_model = noiseModel::Diagonal::Sigmas(PriorNoiseModel);

    /* Go from 1 pose to another pose*/
    OdomNoiseModel = gtsam::Vector(3);
    OdomNoiseModel(0) = 0;
    OdomNoiseModel(1) = 0;
    OdomNoiseModel(2) = 0;
    odom_model = noiseModel::Diagonal::Sigmas(OdomNoiseModel);

    UnaryNoiseModel = gtsam::Vector(2);
    UnaryNoiseModel(0) = 0;
    UnaryNoiseModel(1) = 0;
    // UnaryNoiseModel(2) = 0.5;
    unary_model = noiseModel::Diagonal::Sigmas(UnaryNoiseModel);

    // Resetting the log file for gtsam
    ofstream init_log_reset_stream;
    init_log_reset_stream.open("/home/danielnguyen/gtsam_log.txt");
    init_log_reset_stream << "STARTING LOG" << endl;
    init_log_reset_stream.close();

    ofstream init_step_input_stream;
    init_step_input_stream.open(STEP_INPUT_FILE, ofstream::out | ofstream::trunc);
    init_step_input_stream.close();
}

void slamISAM::update_poses(Pose2 &cur_pose, Pose2 &prev_pose, Pose2 &global_odom,
            Pose2 &velocity,double dt, bool new_gps, optional<rclcpp::Logger> logger) {
    /* Adding poses to the SLAM factor graph */
    double offset_x = 0;
    double offset_y = 0;
    calc_offset_imu_to_car_center(offset_x, offset_y, global_odom.theta());

    if (pose_num == 0)
    {
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "Processing first pose");
        }

        gtsam::PriorFactor<Pose2> prior_factor = gtsam::PriorFactor<Pose2>(X(0),
                                                            global_odom, prior_model);
        //add prior
        //TODO: need to record the initial bearing because it could be erroneous
        graph.add(prior_factor);


        cur_pose = Pose2(-offset_x, -offset_y, global_odom.theta());
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

        gtsam::BetweenFactor<Pose2> odom_factor = BetweenFactor<Pose2>(X(pose_num - 1),
                                                                    X(pose_num),
                                                                        odometry,
                                                                        odom_model);
        cur_pose = Pose2(new_pose.x(), new_pose.y(), new_pose.theta());
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
void slamISAM::update_landmarks(vector<tuple<Point2, double, int>> &old_cones,
                        vector<tuple<Point2, double, Point2>> &new_cones,
                        Pose2 &cur_pose) {


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
    for (size_t o = 0; o < old_cones.size(); o++) {
        Point2 cone_car_frame = get<0>(old_cones.at(o));
        int min_id = get<2>(old_cones.at(o));
        Rot2 b = Rot2::fromAngle(get<1>(old_cones.at(o)));
        double r = norm2(get<0>(old_cones.at(o)));

        graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), L(min_id),
                                        b,
                                        r,
                                        landmark_model));
    }
    isam2.update(graph, values);
    graph.resize(0);
    // values should be empty

    for (size_t n = 0; n < new_cones.size(); n++) {
        Point2 cone_car_frame = get<0>(new_cones.at(n));
        Rot2 b = Rot2::fromAngle(get<1>(new_cones.at(n)));
        double r = norm2(get<0>(new_cones.at(n)));
        Point2 cone_global_frame = get<2>(new_cones.at(n));

        graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), L(n_landmarks),
                                        b,
                                        r,
                                        landmark_model));

        values.insert(L(n_landmarks), cone_global_frame);
        n_landmarks++;
    }
    /* All values in graph must be in values parameter */
    if (n_landmarks < 400) {
        values.insert(X(pose_num), cur_pose);
        Values optimized_val = LevenbergMarquardtOptimizer(graph, values).optimize();
        optimized_val.erase(X(pose_num));
        isam2.update(graph, optimized_val);

    } else {
        isam2.update(graph, values);
    }
    // isam2.update(graph, values);

    graph.resize(0); //Not resizing your graph will result in long update times
    values.clear();


}


/**
 * @brief Obtains information about the observed cones from the current time
 *        step as well as odometry information (to use motion model to
 *        calculate current pose).
 */
void slamISAM::step(gtsam::Pose2 global_odom, vector<Point2> &cone_obs,
            vector<Point2> &cone_obs_blue, vector<Point2> &cone_obs_yellow,
            vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity,
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
    auto start_update_poses = high_resolution_clock::now();
    update_poses(cur_pose, prev_pose, global_odom, velocity, dt, false, logger);
    auto end_update_poses = high_resolution_clock::now();
    auto dur_update_poses = duration_cast<milliseconds>(end_update_poses - start_update_poses);
    if(logger.has_value()) {
        RCLCPP_INFO(logger.value(), "update_poses time: %ld", dur_update_poses.count());
    }


    /**** Retrieve the old cones SLAM estimates & marginal covariance matrices ****/
    auto start_est_retrieval = high_resolution_clock::now();
    vector<Point2> slam_est = {};
    for (int i = 0; i < n_landmarks; i++) {
        slam_est.push_back(isam2.calculateEstimate(L(i)).cast<Point2>());
    }

    vector<MatrixXd> slam_mcov = {};
    for (int i = 0; i < n_landmarks; i++) {
        slam_mcov.push_back(isam2.marginalCovariance(L(i)));
    }
    auto end_est_retrieval = high_resolution_clock::now();
    auto dur_est_retrieval = duration_cast<milliseconds>(end_est_retrieval - start_est_retrieval);
    if(logger.has_value()) {
        RCLCPP_INFO(logger.value(), "est_retrieval time: %ld", dur_est_retrieval.count());
    }


    /**** Data association ***/
    vector<tuple<Point2, double, int>> old_cones = {};
    vector<tuple<Point2, double, Point2>> new_cones = {};


    auto start_DA = high_resolution_clock::now();
    data_association(old_cones, new_cones, cur_pose, prev_pose,
                        cone_obs, logger, slam_est, slam_mcov);
    auto end_DA = high_resolution_clock::now();
    auto dur_DA = duration_cast<milliseconds>(end_DA - start_DA);
    if(logger.has_value()) {
        RCLCPP_INFO(logger.value(), "Data association time: %ld", dur_DA.count());
    }

    auto start_update_landmarks = high_resolution_clock::now();
    update_landmarks(old_cones, new_cones, cur_pose);
    auto end_update_landmarks = high_resolution_clock::now();
    auto dur_update_landmarks = duration_cast<milliseconds>(end_update_landmarks - start_update_landmarks);
    if(logger.has_value()) {
        RCLCPP_INFO(logger.value(), "update_landmarks time: %ld", dur_update_landmarks.count());
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




