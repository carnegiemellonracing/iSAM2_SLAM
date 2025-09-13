/**
 * @file loop_closure.cpp
 * @brief Utility functions used in loop closure detection for SLAM
 */

#include "loop_closure.hpp"

/**
 * @namespace loop_closure_utils
 * @brief Contains helper functions to support loop closure detection in SLAM
*/
namespace loop_closure_utils {
    /**
     * @brief Determines the x and y sign directions from the global yaw angle
     *
     * Divides the plane into 4 quadrants based on the robot's heading (yaw) and set x_sign and y_sign accordingly
     * 
     * @param x_sign Output sign of the x direction (+1 or -1)
     * @param y_sign Output sign of the y direction (+1 or -1)
     * @param cur_pose The current pose of the robot    
    */
    void xy_sign_from_global_bearing(double &x_sign, double &y_sign, gtsam::Pose2 &cur_pose) {
        double yaw = cur_pose.theta();
        if (yaw >= motion_modeling::degrees_to_radians(0) && yaw <= motion_modeling::degrees_to_radians(90)) { 
            /* Quadrant 1 */
            x_sign = 1.0;
            y_sign = 1.0;
        } else if (yaw > motion_modeling::degrees_to_radians(90) && yaw <= motion_modeling::degrees_to_radians(180)) {
            /* Quadrant 2 */
            x_sign = -1.0;
            y_sign = 1.0;
        } else if (yaw > motion_modeling::degrees_to_radians(180) && yaw <= motion_modeling::degrees_to_radians(270)) {
            /* Quadrant 3 */
            x_sign = -1.0;
            y_sign = -1.0;
        } else {
            /* Quadrant 4 */
            x_sign = 1.0;
            y_sign = -1.0;
        }
    }

    /**
     * @brief Determiens whether the robot is facing the start pose
     * 
     * This function compares the heading of the robot and its displacement from the start pose to determine whether it is approaching the start
     * 
     * @param cur_pose The current pose of the robot
     * @param first_pose The initial pose (start position)
     * @param logger Optional logger for debugging
     * @return true if the first pose is in front of the robot, false otherwise
     */
    bool start_pose_in_front(gtsam::Pose2 &cur_pose, gtsam::Pose2 &first_pose, std::optional<rclcpp::Logger> logger) {
        /* Create a vector representing:
        * a.) the heading of the car 
        * b.) the displacement from cur_pose to first_pose 
        */
        double x_sign = 0.0;
        double y_sign = 0.0;
        xy_sign_from_global_bearing(x_sign, y_sign, cur_pose);
        Eigen::Vector2d heading(x_sign * 1, y_sign * abs(tan(cur_pose.theta())));
        Eigen::Vector2d first_cur_diff(first_pose.x() - cur_pose.x(), 
        first_pose.y() - cur_pose.y());
                            
                            /* Calculate dy and dx in the right triangle formed by the 
        * first_pose and the line parallel to heading
        */
        double heading_mag = heading.norm();
        double proj_scaling_factor = (double)(heading.dot(first_cur_diff)) / std::pow(heading_mag, 2);
        Eigen::Vector2d proj_diff_on_heading = proj_scaling_factor * heading;

        double dx = proj_scaling_factor * heading.norm();

        Eigen::Vector2d error = first_cur_diff - proj_diff_on_heading;
        double dy =  error.norm();

        return std::abs(std::atan2(dy, dx)) < motion_modeling::degrees_to_radians(160);

    }

    /**
    * @brief Detects if a loop closure should be triggered
    * 
    * Uses a heuristic based on the distance to the start pose, heading similarity, and whether the robot has moved far enough from the start pose.
    * 
    * @param dist_from_start_loop_closure_th Distance threshold to consider loop closure
    * @param cur_pose The current pose of the robot
    * @param first_pose The initial pose
    * @param pose_num Current pose index (number of poses so far)
    * @param logger Optional logger for debugging
    * 
    * @return true if a loop closure condition is satisfied
    */
    bool detect_loop_closure(double dist_from_start_loop_closure_th, gtsam::Pose2 &cur_pose, gtsam::Pose2 &first_pose, int pose_num, std::optional<rclcpp::Logger> logger) {
        return false;
        auto start_loop_closure = std::chrono::high_resolution_clock::now();
        
        bool moved_away_from_start = pose_num > 300;  //! TODO: add this as a constant
        bool approaching_first_pose = start_pose_in_front(cur_pose, first_pose, logger);
        bool close_to_start = gtsam::norm2(gtsam::Point2(cur_pose.x(), cur_pose.y())) < dist_from_start_loop_closure_th;
        bool heading_like_start = std::abs(first_pose.theta() - cur_pose.theta()) < (motion_modeling::degrees_to_radians(90));

        auto end_loop_closure = std::chrono::high_resolution_clock::now();
        auto dur_loop_closure = std::chrono::duration_cast<std::chrono::milliseconds>(end_loop_closure - start_loop_closure);
        logging_utils::log_string(logger, fmt::format("\tLoop closure time: {}", dur_loop_closure.count()), DEBUG_STEP);

        return (moved_away_from_start && approaching_first_pose && close_to_start && heading_like_start);
    }
}