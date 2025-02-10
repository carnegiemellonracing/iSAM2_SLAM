#include "isam2_pkg.hpp"
using namespace std;
using namespace gtsam;
using namespace std::chrono;
using std::size_t;

slamISAM::slamISAM(optional<rclcpp::Logger> input_logger) {

    // Initializing SLAM Parameters
    blue_cone_ids = {};
    yellow_cone_ids = {};
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
     * 
     */
    LandmarkNoiseModel = gtsam::Vector(2);
    LandmarkNoiseModel(0) = 0.00045; 
    LandmarkNoiseModel(1) = 0.03; 
    landmark_model = noiseModel::Diagonal::Sigmas(LandmarkNoiseModel);

    // used to be all 0s for EUFS_SIM
    PriorNoiseModel = gtsam::Vector(3);
    PriorNoiseModel(0) = 0.22;
    PriorNoiseModel(1) = 0.22;
    PriorNoiseModel(2) = degrees_to_radians(0.009);

    prior_model = noiseModel::Diagonal::Sigmas(PriorNoiseModel);

    /* Go from 1 pose to another pose
     * Source: https://www.movella.com/products/sensor-modules/xsens-mti-680g-rtk-gnss-ins
     *
     * x error in meters (must be based on velocity error)
     * y error in meters (must be based on velocity error)
     * velocity error = 0.05 m/s RMS
     * Calculation for the error per meter: 1m = (1 +- 0.05 m/s) * (1 +- 1e-9 s)
     * std. dev = 1 * sqrt((0.05/1)^2 + (1e-9/1)^2) = 0.05
     * yaw error in radians
     */
    OdomNoiseModel = gtsam::Vector(3);
    OdomNoiseModel(0) = 0.22;
    OdomNoiseModel(1) = 0.22; 
    OdomNoiseModel(2) = degrees_to_radians(0.009); 
    odom_model = noiseModel::Diagonal::Sigmas(OdomNoiseModel);


    /* GPS noise model 
     * Use the covariances from positionlla
     *
     * Covariance matrix diagonal elements represent variances
     * Variance = (std.dev)^2 meaning:
     * Variance = 0.2 => std.dev = sqrt(0.2) = 0.45
     * 
     * However positionlla already accounts for the covariance
     * 
     */
    UnaryNoiseModel = gtsam::Vector(2);
    UnaryNoiseModel(0) = 0.01;
    UnaryNoiseModel(1) = 0.01;
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

    completed_chunking = false;
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
            RCLCPP_INFO(logger.value(), "||velocity|| = %f", norm2(Point2(velocity.x(), velocity.y())));
            if (norm2(Point2(velocity.x(), velocity.y())) > VELOCITY_MOVING_TH) {
                RCLCPP_INFO(logger.value(), "Car is moving");
            } else {
                RCLCPP_INFO(logger.value(), "Car stopped");
            }

            RCLCPP_INFO(logger.value(), "|angular velocity| = %f", abs(velocity.theta()));

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
                        std::vector<int> &cone_ids,
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
    for (std::size_t o = 0; o < old_cones.size(); o++) {
        Point2 cone_pos_car_frame = old_cones.at(o).local_cone_pos;
        int min_id = (old_cones.at(o)).min_id;
        Rot2 b = Rot2::fromAngle((old_cones.at(o)).bearing);
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
        Point2 cone_pos_car_frame = (new_cones.at(n).local_cone_pos);
        Rot2 b = Rot2::fromAngle((new_cones.at(n)).bearing);
        double r = norm2(cone_pos_car_frame);

        Point2 cone_global_frame = (new_cones.at(n).global_cone_pos);

        graph.add(BearingRangeFactor<Pose2, Point2>(X(pose_num), L(n_landmarks),
                                        b,
                                        r,
                                        landmark_model));
        cone_ids.push_back(n_landmarks);

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
// TODO: change void to option type
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
        // std::vector<tuple<Point2, double, int>> old_cones = {};
        // std::vector<tuple<Point2, double, Point2>> new_cones = {};

        std::vector<Old_cone_info> blue_old_cones;
        std::vector<Old_cone_info> yellow_old_cones;

        std::vector<New_cone_info> blue_new_cones;
        std::vector<New_cone_info> yellow_new_cones;
        
        auto start_DA = high_resolution_clock::now();
        data_association(blue_old_cones, blue_new_cones, blue_cone_ids, cur_pose, prev_pose, is_turning,
                            cone_obs_blue, logger, slam_est, slam_mcov);
        
        data_association(yellow_old_cones, yellow_new_cones, yellow_cone_ids, cur_pose, prev_pose, is_turning,
                            cone_obs_yellow, logger, slam_est, slam_mcov);
        
        auto end_DA = high_resolution_clock::now();
        auto dur_DA = duration_cast<milliseconds>(end_DA - start_DA);
        if(logger.has_value()) {
            RCLCPP_INFO(logger.value(), "Data association time: %ld", dur_DA.count());
        }

        auto start_update_landmarks = high_resolution_clock::now();
        update_landmarks(blue_old_cones, blue_new_cones, blue_cone_ids, cur_pose);
        update_landmarks(yellow_old_cones, yellow_new_cones, yellow_cone_ids, cur_pose);
        auto end_update_landmarks = high_resolution_clock::now();
        auto dur_update_landmarks = duration_cast<milliseconds>(end_update_landmarks - start_update_landmarks);
        if(logger.has_value()) {
            RCLCPP_INFO(logger.value(), "update_landmarks time: %ld", dur_update_landmarks.count());
        }
    } else{
        // populate yellow and blue cone arrays with cone locations
        for (int i = 0; i < blue_cones_id.size(); i++){
            int id = blue_cones_id.at(i);
            Point2 curr_cone = isam2.calculateEstimate(L(id)).cast<Point2>();
            blue_cones.emplace_back(curr_cone.x(),
                                    curr_cone.y(),
                                    id);
        }
        for (int i = 0; i < yellow_cones_id.size(); i++){
            int id = yellow_cones_id.at(i);
            Point2 curr_cone = isam2.calculateEstimate(L(id)).cast<Point2>();
            yellow_cones.emplace_back(curr_cone.x(),
                                    curr_cone.y(),
                                    id);
        }
        std::vector<Chunks*> = *generateChunks(blue_cones, yellow_cones);
        identify_chunk
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

void slamISAM::identify_chunk(vector<tuple<Point2, double, vector<Point2> &cone_obs,
            vector<tuple<Point2, double, Point2>> &new_cones) {
    //add new cones to lookup table
    //  Assume that chunks has the cones associated with each track bound
    // TODO: Tell chunking to store this information in the chunk struct

    //define int variable chunkid_sum
    //for each element in observed_cones
    //chunkid_sum+=lookuptable[element.cone_id]
    //return the average of chunkid (chunkid_sum/observed_cones.length)
    //call get_new_old_cones to update new cones and old cones

    // 1.) Identify the cone IDs
    //- because we have a lookup table 
    //- New cones will be empty; old_cones will contain vector of tuples, cone position + ID
    //old_cones is a vector of tuples
    // what's the type of each tuple

    // TODO: build this data structure
    unordered_map<int, int> coneID_to_chunkID = {}

    unordered_map<int, int> chunkID_to_vote = {}
    // Get the ID of the old_cones
    get_old_new_cones(old_cones, new_cones,
                    global_cone_x, global_cone_y,bearing,
                    cone_obs, m_dist, n_landmarks, logger);

    // Start tallying/voting
    for (size_t i = 0; i < old_cones.size();i++){
        int cone_id = get<2>(old_cones.at(i));
        int chunk_id = coneID_to_chunkID[cone_id];

        //Check if the chunkID is already in the voting map
        if (chunkID_to_vote.find(chunk_id)!=chunkID_to_vote.end()){
            chunkID_to_vote[chunk_id]++;
        } else{
            chunkID_to_vote[chunk_id]=0;
        }

    }
    //find max vote
    for (size_t i=0; i<chunkID_to_vote.size();i++){
        int id = (chunkID_to_vote.begin()+i)->first;
        int vote = (chunkID_to_vote.begin()+i)->second;
        
    }
    

    RCLCPP_INFO(identify_chunk(), "Chunk you are in");

}
