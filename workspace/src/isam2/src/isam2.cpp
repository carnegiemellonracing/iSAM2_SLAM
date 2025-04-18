#include "isam2_pkg.hpp"
using namespace std;
using namespace gtsam;
using namespace std::chrono;
using std::size_t;

slamISAM::slamISAM(std::optional<rclcpp::Logger> input_logger, std::optional<NoiseInputs> &yaml_noise_inputs) {

    // Chunking 
    all_chunks = {};
    blue_cone_to_chunk = {};
    yellow_cone_to_chunk = {};

    // Initializing SLAM Parameters
    parameters = ISAM2Params(ISAM2DoglegParams(),0.1,10,true);
    parameters.setFactorization("QR");
    logger = input_logger;
    isam2 = gtsam::ISAM2(parameters);
    graph = gtsam::NonlinearFactorGraph();
    values = gtsam::Values();

    pose_num = 0uz;
    first_pose_added = false;
    blue_n_landmarks = 0uz;
    yellow_n_landmarks = 0uz;


    
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
 * @brief Orders newly detected cones based on proximity and directional continuity with existing path
 *
 * Uses a greedy algorithm that prioritizes:
 * - Closest cones first when starting
 * - Combines distance (70% weight) and angular continuity (30% weight) for subsequent selections
 *
 * @param color_slam_est Existing ordered cones from SLAM estimation
 * @param new_cones Newly detected cones to be sorted into path sequence
 * @return Ordered cones for path continuity
 */
// std::vector<New_cone_info> slamISAM::sort_cone_ids(const std::vector<gtsam::Point2> &color_slam_est, std::vector<New_cone_info> &new_cones) {
//     std::vector<New_cone_info> ordered;
//     gtsam::Point2 current;
//     double prev_angle;

//     // Starting position at origin if no previous position
//     if (color_slam_est.empty()) {
//         current = gtsam::Point2(0.0, 0.0);
//     }
//     else {
//         // Continue from last known cone position
//         current = color_slam_est.back();

//         // Calculate initial direction based on existing path
//         if (color_slam_est.size() >= 2) {
//             const auto &prev_point = color_slam_est[color_slam_est.size() - 2];
//             double dx = current.x() - prev_point.x();
//             double dy = current.y() - prev_point.y();
//             prev_angle = std::atan2(dy, dx);
//         }
//         else {
//             // Use direction from origin to single existing cone
//             prev_angle = std::atan2(current.y(), current.x());
//         }
//     }

//     // Handle empty case
//     if (new_cones.empty()) {
//         return ordered;
//     }

//     // Track used cones
//     std::vector<bool> used(new_cones.size(), false);
//     int remaining = new_cones.size();

//     while (remaining > 0) {
//         if (ordered.empty()) {
//             // First iteration: select closest to current
//             double best_dist = DBL_MAX;
//             int best_idx = -1;
//             for (int i = 0; i < new_cones.size(); ++i) {
//                 if (used[i]) {
//                     continue;
//                 }
//                 double dist = (new_cones[i].local_cone_pos - current).norm();
//                 if (dist < best_dist) {
//                     best_dist = dist;
//                     best_idx = i;
//                 }
//             }
//             if (best_idx == -1) {
//                 break;
//             }

//             ordered.push_back(new_cones[best_idx]);
//             used[best_idx] = true;
//             remaining--;

//             // Update current and prev_angle
//             current = new_cones[best_idx].local_cone_pos;
//             if (color_slam_est.empty()) {
//                 // Starting from origin, prev_angle is from origin to first cone
//                 prev_angle = std::atan2(current.y(), current.x());
//             }
//             else {
//                 // prev_angle is from last old cone to first new cone
//                 const auto &old_current = color_slam_est.back();
//                 double dx = current.x() - old_current.x();
//                 double dy = current.y() - old_current.y();
//                 prev_angle = std::atan2(dy, dx);
//             }
//         }
//         else {
//             // Subsequent cones: select based on score
//             double min_score = DBL_MAX;
//             int best_idx = -1;
//             for (int i = 0; i < new_cones.size(); ++i) {
//                 if (used[i]) {
//                     continue;
//                 }
//                 const auto &candidate = new_cones[i];
//                 double dx = candidate.local_cone_pos.x() - current.x();
//                 double dy = candidate.local_cone_pos.y() - current.y();
//                 double distance = std::hypot(dx, dy);
//                 double angle = std::atan2(dy, dx);
//                 double angle_diff = std::abs(angle - prev_angle);
//                 // Consider the smallest angle difference (acute angle)
//                 angle_diff = std::min(angle_diff, 2 * M_PI - angle_diff);
//                 double score = 0.7 * distance + 0.3 * angle_diff;

//                 if (score < min_score){
//                     min_score = score;
//                     best_idx = i;
//                 }
//             }
//             if (best_idx == -1) {
//                 break;
//             }

//             ordered.push_back(new_cones[best_idx]);
//             used[best_idx] = true;
//             remaining--;

//             // Update current and prev_angle
//             const auto &new_cone_pos = new_cones[best_idx].local_cone_pos;
//             const auto &prev_cone_pos = ordered[ordered.size() - 2].local_cone_pos;
//             double new_dx = new_cone_pos.x() - prev_cone_pos.x();
//             double new_dy = new_cone_pos.y() - prev_cone_pos.y();
//             prev_angle = std::atan2(new_dy, new_dx);
//             current = new_cone_pos;
//         }
//     }

//     assert(ordered.size() == new_cones.size());
//     return ordered;
// }

std::vector<New_cone_info> slamISAM::sort_cone_ids(const std::vector<gtsam::Point2>& color_slam_est, std::vector<New_cone_info>& new_cones) {
    std::vector<New_cone_info> ordered;
    gtsam::Point2 current;
    double prev_angle;
    bool has_prior_path = color_slam_est.size() >= 2;

    // Initialize position and direction
    if (color_slam_est.empty()) {
        current = gtsam::Point2(0.0, 0.0);
    } else {
        current = color_slam_est.back();
        if (has_prior_path) {
            const auto& prev_point = color_slam_est[color_slam_est.size()-2];
            prev_angle = std::atan2(current.y()-prev_point.y(), 
            current.x()-prev_point.x());
        } else {
            prev_angle = std::atan2(current.y(), current.x());
        }
    }

    if (new_cones.empty()) return ordered;

    // Split into front/back cones if we have prior path
    std::vector<int> front_indices, back_indices;
    if (has_prior_path) {
        const gtsam::Point2& last_blue = color_slam_est.back();
        const gtsam::Point2& second_last_blue = color_slam_est[color_slam_est.size()-2];
        gtsam::Point2 direction_vector(last_blue.x() - second_last_blue.x(),
        last_blue.y() - second_last_blue.y());

        for (int i = 0; i < new_cones.size(); ++i) {
            gtsam::Point2 to_cone(new_cones[i].local_cone_pos.x() - last_blue.x(),
            new_cones[i].local_cone_pos.y() - last_blue.y());

            double dot = direction_vector.x() * to_cone.x() + direction_vector.y() * to_cone.y();
            if (dot > 0) {
                front_indices.push_back(i);
            } else {
                back_indices.push_back(i);
            }
        }
    }

    std::vector<bool> used(new_cones.size(), false);
    int remaining = new_cones.size();

    while (remaining > 0) {
        if (has_prior_path && !back_indices.empty()) {
            // Process back cones first, sorted by distance from last blue cone
            std::sort(back_indices.begin(), back_indices.end(),
            [&](int a, int b) {
            double dist_a = (new_cones[a].local_cone_pos - color_slam_est.back()).norm();
            double dist_b = (new_cones[b].local_cone_pos - color_slam_est.back()).norm();
            return dist_a < dist_b;
            });

            // Add closest back cone
            for (int i : back_indices) {
                if (!used[i]) {
                    ordered.push_back(new_cones[i]);
                    used[i] = true;
                    remaining--;

                    // Update tracking variables using original path direction
                    current = color_slam_est.back();
                    if (color_slam_est.size() >= 2) {
                        const auto& prev_point = color_slam_est[color_slam_est.size()-2];
                        prev_angle = std::atan2(current.y()-prev_point.y(),
                        current.x()-prev_point.x());
                    }
                    break;
                }
            }
            continue;
        }

        if (ordered.empty() && color_slam_est.empty()) {
            // Initial case: sort first 20 by Y-coordinate
            std::vector<int> indices(new_cones.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), [&](int a, int b) {
            return new_cones[a].local_cone_pos.y() < new_cones[b].local_cone_pos.y();
            });

            for (int i = 0; i < std::min(20, (int)indices.size()); ++i) {
                ordered.push_back(new_cones[indices[i]]);
                used[indices[i]] = true;
                remaining--;
            }
            continue;
        }

        // Standard scoring for front cones or general case
        double min_score = DBL_MAX;
        int best_idx = -1;
        for (int i = 0; i < new_cones.size(); ++i) {
            if (used[i]) continue;

            const auto& candidate = new_cones[i];
            double dx = candidate.local_cone_pos.x() - current.x();
            double dy = candidate.local_cone_pos.y() - current.y();
            double distance = std::hypot(dx, dy);
            double angle = std::atan2(dy, dx);
            double angle_diff = std::abs(angle - prev_angle);
            angle_diff = std::min(angle_diff, 2*M_PI - angle_diff);
            double score = 0.7*distance + 0.3*angle_diff;

            if (score < min_score) {
                min_score = score;
                best_idx = i;
            }
        }

        if (best_idx != -1) {
            ordered.push_back(new_cones[best_idx]);
            used[best_idx] = true;
            remaining--;

            // Update tracking variables
            const auto& new_pos = new_cones[best_idx].local_cone_pos;
            if (ordered.size() >= 2) {
                const auto& prev_pos = ordered[ordered.size()-2].local_cone_pos;
                prev_angle = std::atan2(new_pos.y() - prev_pos.y(),
                new_pos.x() - prev_pos.x());
            } else {
                prev_angle = std::atan2(new_pos.y(), new_pos.x());
            }
            current = new_pos;
        }
    }

    assert(ordered.size() == new_cones.size());
    return ordered;
}

void slamISAM::write_chunk_data(const std::vector<Chunk *> &chunks)
{
    std::ofstream file(CHUNKS_FILE);
    int chunk_id = 0;

    for (const Chunk *chunk : chunks)
    {
        file << "CHUNK:" << chunk->tStart << "," << chunk->tEnd << "\n";

        // Blue cones with chunk_id
        for (int id : chunk->blueConeIds)
        {
            file << "BLUE:" << id << ":" << chunk_id << "\n";
        }

        // Yellow cones with chunk_id
        for (int id : chunk->yellowConeIds)
        {
            file << "YELLOW:" << id << ":" << chunk_id << "\n";
        }

        chunk_id++;\
    }
    file.close();
}

int slamISAM::identify_chunk(std::vector<Old_cone_info> &blue_old_cones,
    std::vector<Old_cone_info> &yellow_old_cones) {
    unordered_map<int, int> chunkID_to_vote = {};

    // vote on behalf of blue cones! 
    for (size_t i = 0; i < blue_old_cones.size(); i++) {
        int cone_id = blue_old_cones.at(i).min_id;
        int chunk_id = blue_cone_to_chunk[cone_id];

        /* Check if the chunkID is already in the voting map */ 
        if (chunkID_to_vote.find(chunk_id) != chunkID_to_vote.end()) {
            chunkID_to_vote[chunk_id]++;
        }
        else {
            chunkID_to_vote[chunk_id] = 1;
        }
    }

    // vote on behalf of yellow cones! 
    for (size_t i = 0; i < yellow_old_cones.size(); i++) {
        int cone_id = yellow_old_cones.at(i).min_id;
        int chunk_id = yellow_cone_to_chunk[cone_id];

        /* Check if the chunkID is already in the voting map */ 
        if (chunkID_to_vote.find(chunk_id) != chunkID_to_vote.end()) {
            chunkID_to_vote[chunk_id]++;
        }
        else {
            chunkID_to_vote[chunk_id] = 1;
        }
    }

    // find max vote
    int max_chunk_id = -1;
    int max_votes = -1;
    for (const std::pair<int, int>& chunkID_and_vote : chunkID_to_vote) {
        if (chunkID_and_vote.second > max_votes) {
            max_votes = chunkID_and_vote.second;
            max_chunk_id = chunkID_and_vote.first;
        }
    }
    return max_chunk_id;
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
        if (blue_n_landmarks + yellow_n_landmarks > 0) {
            stability_update(true); 
        }
        
        data_association(blue_old_cones, blue_new_cones, cur_pose, prev_pose, is_turning,
                            cone_obs_blue, logger, blue_slam_est, blue_slam_mcov);
        data_association(yellow_old_cones, yellow_new_cones, cur_pose, prev_pose, is_turning,
                            cone_obs_yellow, logger, yellow_slam_est, yellow_slam_mcov);
        
        auto end_DA = high_resolution_clock::now();
        auto dur_DA = duration_cast<milliseconds>(end_DA - start_DA);
        log_string(logger, fmt::format("\tData association time: {}", dur_DA.count()), DEBUG_STEP);


        /* Sort new cones with path-aware ordering */
        auto start_sort = high_resolution_clock::now();
        std::vector<New_cone_info> sorted_blue = sort_cone_ids(blue_slam_est, blue_new_cones);
        std::vector<New_cone_info> sorted_yellow = sort_cone_ids(yellow_slam_est, yellow_new_cones);
        auto end_sort = high_resolution_clock::now();
        auto dur_sort = duration_cast<milliseconds>(end_sort - start_sort);
        log_string(logger, fmt::format("\tSort time: {}", dur_sort.count()), DEBUG_STEP);

        auto start_update_landmarks = high_resolution_clock::now();

        int old_blue_n_landmarks = blue_n_landmarks;
        int old_yellow_n_landmarks = yellow_n_landmarks;
        log_string(logger, fmt::format("\t\tStarted updating isam2 model with new and old cones"), DEBUG_STEP);

        blue_n_landmarks = update_landmarks(blue_old_cones, sorted_blue, blue_n_landmarks, cur_pose, BLUE_L);
        yellow_n_landmarks = update_landmarks(yellow_old_cones, sorted_yellow, yellow_n_landmarks, cur_pose, YELLOW_L);
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

            cone_proximity_updates(blue_n_landmarks, blue_n_landmarks, blue_slam_est, blue_slam_mcov, BLUE_L);
            cone_proximity_updates(yellow_n_landmarks,yellow_n_landmarks, yellow_slam_est, yellow_slam_mcov, YELLOW_L);
        }
        stability_update(true);
        auto end_update_landmarks = high_resolution_clock::now();
        auto dur_update_landmarks = duration_cast<milliseconds>(end_update_landmarks - start_update_landmarks);

        log_string(logger, fmt::format("\tUpdate_landmarks time: {}", dur_update_landmarks.count()), DEBUG_STEP);
    }
    else {
        /* Chunking section */
        if (!completed_chunking) {
            if (logger.has_value()) {
                RCLCPP_INFO(logger.value(), "performing chunking");
            }
            /* This is a map from cone ID (the index) to chunk ID, the element*/
            blue_cone_to_chunk.resize(blue_n_landmarks, 0);
            yellow_cone_to_chunk.resize(yellow_n_landmarks, 0);

            std::vector<std::tuple<double, double, int>> blue_cones;
            std::vector<std::tuple<double, double, int>> yellow_cones;

            for (int i = 0; i < blue_n_landmarks; i++) {
                int id = i;
                Point2 curr_cone = isam2.calculateEstimate(BLUE_L(id)).cast<Point2>();
                blue_cones.emplace_back(curr_cone.x(),
                                        curr_cone.y(),
                                        id);
            }

            for (int i = 0; i < yellow_n_landmarks; i++) {
                int id = i;
                Point2 curr_cone = isam2.calculateEstimate(YELLOW_L(id)).cast<Point2>();
                yellow_cones.emplace_back(curr_cone.x(),
                                          curr_cone.y(),
                                          id);
            }
            if (logger.has_value()) {
                RCLCPP_INFO(logger.value(), "obtained estimates for chunking");
            }
            all_chunks = *generateChunks(blue_cones, yellow_cones);
            if (logger.has_value()) {
                RCLCPP_INFO(logger.value(), "finish chunking");
            }
            /* Process the chunks to create the map from cone_ids to chunk_ids*/
            // make cone to chunk tables (for blue and yellow cones separately)
            if (logger.has_value()) {
                RCLCPP_INFO(logger.value(), "Number of chunks: %d", all_chunks.size());
            }
            for (int chunk_id = 0; chunk_id < all_chunks.size(); chunk_id++)
            {
                Chunk *curr_chunk = all_chunks.at(chunk_id);

                if (logger.has_value()) {
                    RCLCPP_INFO(logger.value(), "processing chunk: %d", chunk_id);
                }
                // blue cones
                for (int j = 0; j < curr_chunk->blueConeIds.size(); j++)
                {
                    int cone_id = curr_chunk->blueConeIds.at(j);
                    if (cone_id >= blue_cone_to_chunk.size()) {
                        continue;
                    }
                    blue_cone_to_chunk.at(cone_id) = chunk_id;
                }
                if (logger.has_value())
                {
                    RCLCPP_INFO(logger.value(), "\tsanity check 1");
                }
                // yellow cones
                for (int j = 0; j < curr_chunk->yellowConeIds.size(); j++)
                {
                    int cone_id = curr_chunk->yellowConeIds.at(j);
                    if (cone_id >= yellow_cone_to_chunk.size()) {
                        continue;
                    }
                    yellow_cone_to_chunk.at(cone_id) = chunk_id;
                }
                if (logger.has_value())
                {
                    RCLCPP_INFO(logger.value(), "\tsanity check 2");
                }
            }
            if (logger.has_value()) {
                RCLCPP_INFO(logger.value(), "created cone to chunk map");
            }

            completed_chunking = true;
            // Write chunk data for visualization
            write_chunk_data(all_chunks);
            log_string(logger, "Chunk data written", DEBUG_STEP);
        }
        // else {
        //     assert(completed_chunking);
        //     int cur_chunk_id = identify_chunk(blue_old_cones, yellow_old_cones);
        //     log_string(logger, fmt::format("Current chunk ID: %d",  cur_chunk_id), DEBUG_STEP);
        // }
    }

    pose_num++;

    /* Logging estimates for visualization */
    auto start_vis_setup = high_resolution_clock::now();
    print_estimates();
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
    ofstream ofs;
    
    ofs.open(ESTIMATES_FILE, std::ofstream::out | std::ofstream::trunc);
    streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    cout.rdbuf(ofs.rdbuf());

    for (std::size_t i = 0; i < blue_n_landmarks; i++) {
        gtsam::Point2 blue_cone = blue_slam_est.at(i);
        std::cout << "Value b:" << blue_cone.x() << ":" << blue_cone.y() << std::endl;
    }
    
    for (std::size_t i = 0; i < yellow_n_landmarks; i++) {
        gtsam::Point2 yellow_cone = yellow_slam_est.at(i);
        std::cout << "Value y:" << yellow_cone.x() << ":" << yellow_cone.y() << std::endl;
    }

    for (std::size_t i = 0; i < pose_num; i++) {
        gtsam::Pose2 cur_pose = isam2.calculateEstimate(X(i)).cast<gtsam::Pose2>();
        std::cout << "Value x:" << cur_pose.x() << ":" << cur_pose.y() << std::endl;
    }
    ofs.close();
    cout.rdbuf(coutbuf); //reset to standard output again
}




