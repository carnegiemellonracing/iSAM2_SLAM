/**
 * @file data_association.cpp
 * @brief Perform data association in preparation for feeding data to SLAM
 */
#include <vector>
#include <tuple>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <algorithm>
#include <float.h>

#include "ros_utils.cpp"
const double M_DIST_TH = 120;

using namespace Eigen;

void populate_m_dist(MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                    int num_obs, vector<double> &m_dist,
                    vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov,
                    rclcpp::Logger &logger) {
    for (int i = 0; i < num_obs; i++) {
        for (int j = 0; j < slam_est.size(); j++) {

            MatrixXd diff(1, 2);
            diff << global_cone_x(i,0) - slam_est.at(j).x(),
                    global_cone_y(i,0) - slam_est.at(j).y();

            RCLCPP_INFO(logger, "mcov dims; rows: %d | cols: %d",
                                slam_mcov.at(j).rows(), slam_mcov.at(j).cols());
            m_dist.push_back((diff * slam_mcov.at(j) * diff.transpose())(0, 0));

        }

        m_dist.push_back(M_DIST_TH);
    }

}

void find_new_cones(vector<tuple<Point2, double, int>> &old_cones,
            vector<tuple<Point2, double, Point2>> &new_cones,
            MatrixXd &global_cone_x,MatrixXd &global_cone_y,MatrixXd &bearing,
            vector<Point2> &cone_obs, vector<double> &m_dist, int n_landmarks,
            rclcpp::Logger &logger) {

    int lo, hi = 0;
    for (int i = 0; i < cone_obs.size(); i++) {
        hi += n_landmarks + 1;
        RCLCPP_INFO(logger, "Calculating min m_dist element");
        vector<double>::iterator min_dist = min_element(m_dist.begin() + lo,
                                                        m_dist.begin() + hi);
        int min_id = distance(m_dist.begin() + lo, min_dist);

        if (min_id == n_landmarks) {
            RCLCPP_INFO(logger, "Adding new cone...");

            new_cones.emplace_back(cone_obs.at(i),
                                    bearing(i, 0),
                                    Point2(global_cone_x(i,0), global_cone_y(i,0)));
            RCLCPP_INFO(logger, "New cone added!");
        } else {
            RCLCPP_INFO(logger, "Updating old cone...");
            old_cones.emplace_back(cone_obs.at(i), bearing(i, 0), min_id);
            RCLCPP_INFO(logger, "Old cone updated!");
        }


        lo = hi;
    }
}

void data_association(vector<tuple<Point2, double, int>> &old_cones,
                vector<tuple<Point2, double, Point2>> &new_cones,
                Pose2 &cur_pose, Pose2 &prev_pose,
                vector<Point2> &cone_obs, rclcpp::Logger &logger,
                vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov) {

    vector<double> m_dist = {};
    int n_landmarks = slam_est.size();

    // Populating m_dist with mahalanobis distances
    MatrixXd global_cone_x(cone_obs.size(), 1);
    MatrixXd global_cone_y(cone_obs.size(), 1);
    MatrixXd bearing(cone_obs.size(), 1);
    MatrixXd range(cone_obs.size(), 1);

    calc_cone_range_from_car(range, cone_obs);
    calc_cone_bearing_from_car(bearing, cone_obs);

    RCLCPP_INFO(logger, "Calculated cone range and bearing wrt car");
    cone_to_global_frame(range, bearing,
                         global_cone_x, global_cone_y,
                         cone_obs, cur_pose, prev_pose);

    RCLCPP_INFO(logger, "Calculated global cone position");

    populate_m_dist(global_cone_x, global_cone_y, cone_obs.size(), m_dist,
                    slam_est, slam_mcov, logger);
    RCLCPP_INFO(logger, "Populated m_dist");

    find_new_cones(old_cones, new_cones,
                    global_cone_x, global_cone_y,bearing,
                    cone_obs, m_dist, n_landmarks, logger);
    RCLCPP_INFO(logger, "Min ID performed, new cones identified");

}



