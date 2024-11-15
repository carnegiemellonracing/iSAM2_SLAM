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
//const double M_DIST_TH = 1200; // Best for velocity motion_model
const double M_DIST_TH = 0.3;

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

    int lo = 0;
    int hi = 0;

    for (int i = 0; i < cone_obs.size(); i++) {
        hi += n_landmarks + 1;
        vector<double>::iterator min_dist = min_element(m_dist.begin() + lo,
                                                        m_dist.begin() + hi);
        int min_id = distance(m_dist.begin() + lo, min_dist);
        if (min_id == n_landmarks) {
            Point2 global_cone_pos = Point2(global_cone_x(i,0), global_cone_y(i,0));
            new_cones.emplace_back(cone_obs.at(i),
                                    bearing(i, 0),
                                    global_cone_pos);
        } else {
            old_cones.emplace_back(cone_obs.at(i), bearing(i, 0), min_id);
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
    int n_obs = cone_obs.size();

    // Populating m_dist with mahalanobis distances
    MatrixXd global_cone_x(cone_obs.size(), 1);
    MatrixXd global_cone_y(cone_obs.size(), 1);
    MatrixXd bearing(cone_obs.size(), 1);
    MatrixXd range(cone_obs.size(), 1);

    calc_cone_range_from_car(range, cone_obs);
    calc_cone_bearing_from_car(bearing, cone_obs);

    cone_to_global_frame(range, bearing,
                         global_cone_x, global_cone_y,
                         cone_obs, cur_pose);
    /*
    populate_m_dist(global_cone_x, global_cone_y, cone_obs.size(), m_dist,
                    slam_est, slam_mcov, logger);

    find_new_cones(old_cones, new_cones,
                    global_cone_x, global_cone_y,bearing,
                    cone_obs, m_dist, n_landmarks, logger);
    */
Eigen::MatrixXd a = Eigen::MatrixXd::Zero(2, n_landmarks * n_obs * 2);
    Eigen::MatrixXd b(2, n_landmarks * n_obs * 2);
    Eigen::MatrixXd c(2, n_landmarks);
    // create matrix A: contains the difference vectors as columns, each diff vector is duplicated
    for (int o = 0; o < n_obs; o++)
    {
        // Populate matrix A with all of the differnce vectors
        for (int i = 0; i < n_landmarks; i++)
        {
            Point2 diff = Point2(global_cone_x(o,0)-slam_est.at(i).x(), global_cone_y(o,0)-slam_est.at(i).y());
            a(0,i*2) = diff.x();
            a(0, i*2+1) = diff.x();
            a(1,i*2) = diff.y();
            a(1,i*2+1) = diff.y();
        }
    }

    MatrixXd smol_b(2, n_landmarks * 2);
    for (int i = 0; i < n_landmarks; i++)
    {
        // For each observed cone, add the marginal_cov matrix to B
        MatrixXd mcov = slam_mcov[i];
        smol_b.block<2, 2>(0, i * 2) = mcov;
    }

    for (int o = 0; o < n_obs; o++)
    {
        b.block(0, o * n_landmarks * 2, 2, n_landmarks * 2) = smol_b;
    }

    for (int o = 0; o < n_obs; o++)
    {
        for (int i = 0; i < n_landmarks; i++)
        {
            Point2 diff = Point2(global_cone_x(o,0)-slam_est.at(i).x(), global_cone_y(o,0)-slam_est.at(i).y());
            // Build c by row stacking each diff vector
            c(0, i) = diff.x();
            c(1, i) = diff.y();
        }
    }
    MatrixXd pre_AB = a.array() * b.array();
    VectorXd AB_vec = pre_AB.colwise().sum();
    MatrixXd dists = Eigen::Map<Eigen::MatrixXd>(AB_vec.data(), 2, n_landmarks);
    dists.array() = dists.array() * c.array();
    VectorXd m_dists_vec = dists.colwise().sum();
    for(int o = 0; o < n_obs; o++) {
        int min_id;
        int min = m_dists_vec.segment(o * n_obs, n_obs).minCoeff(&min_id);
        if (min >= M_DIST_TH)
        {
            Point2 global_cone_pos = Point2(global_cone_x(o, 0), global_cone_y(o, 0));
            new_cones.emplace_back(cone_obs.at(o),
                                    bearing(o, 0),
                                    global_cone_pos);
        }
        else
        {
            old_cones.emplace_back(cone_obs.at(o), bearing(o, 0), min_id);
        }
    }
}




