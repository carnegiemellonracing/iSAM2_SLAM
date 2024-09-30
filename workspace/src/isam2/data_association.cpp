/**
 * @file data_association.cpp
 * @brief Perform data association in preparation for feeding data to SLAM
 */

void data_association(Pose2 &global_odom, Pose2 velocity,
                    vector<Point2> &blue_cones, vector<Point2> &yellow_cones,
                    vector<Pose2> slam_estimates, vector<Point2> &orange_cones) {


