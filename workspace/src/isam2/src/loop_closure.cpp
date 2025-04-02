#include "loop_closure.hpp"

void xy_sign_from_global_bearing(double &x_sign, double &y_sign, Pose2 &cur_pose) {
    double yaw = cur_pose.theta();
    if (yaw >= degrees_to_radians(0) && yaw <= degrees_to_radians(90)) { 
        /* Quadrant 1 */
        x_sign = 1.0;
        y_sign = 1.0;
    } else if (yaw > degrees_to_radians(90) && yaw <= degrees_to_radians(180)) {
        /* Quadrant 2 */
        x_sign = -1.0;
        y_sign = 1.0;
    } else if (yaw > degrees_to_radians(180) && yaw <= degrees_to_radians(270)) {
        /* Quadrant 3 */
        x_sign = -1.0;
        y_sign = -1.0;
    } else {
        /* Quadrant 4 */
        x_sign = 1.0;
        y_sign = -1.0;
    }
}

/* @brief Calculates the bearing of the first_pose from the car*/
bool start_pose_in_front(Pose2 &cur_pose, Pose2 &first_pose, optional<rclcpp::Logger> logger) {
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
    double proj_scaling_factor = (double)(heading.dot(first_cur_diff)) / pow(heading_mag, 2);
    Eigen::Vector2d proj_diff_on_heading = proj_scaling_factor * heading;

    double dx = proj_scaling_factor * heading.norm();

    Eigen::Vector2d error = first_cur_diff - proj_diff_on_heading;
    double dy =  error.norm();

    return abs(atan2(dy, dx)) < degrees_to_radians(90);

}

bool detect_loop_closure(Pose2 &cur_pose, Pose2 &first_pose, int pose_num, optional<rclcpp::Logger> logger) {

    bool moved_away_from_start = pose_num > 10;  //fails when you have 10 poses moved backwards
    bool approaching_first_pose = start_pose_in_front(cur_pose, first_pose, logger);
    bool close_to_start = norm2(Point2(cur_pose.x(), cur_pose.y())) < DIST_FROM_START_LC_TH;
    bool heading_like_start = abs(first_pose.theta() - cur_pose.theta()) < (degrees_to_radians(90));

    /*if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "\nLoop closure detection results: ");

        if (close_to_start) {
            RCLCPP_INFO(logger.value(), "close_to_start: true");
        } else {
            RCLCPP_INFO(logger.value(), "close_to_start: false");
        }

        if (heading_like_start) {
            RCLCPP_INFO(logger.value(), "heading_like_start: true | cur_heading: %f, start_heading %f",
                                        cur_pose.theta(), first_pose.theta());
        } else {
            RCLCPP_INFO(logger.value(), "heading_like_start: false | cur_heading: %f, start_heading %f",
                                        cur_pose.theta(), first_pose.theta());
        }
    }*/

    return (moved_away_from_start && approaching_first_pose && close_to_start && heading_like_start);
}