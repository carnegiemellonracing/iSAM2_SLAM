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

void populate_m_dist(vector<Pose2> &global_cone_obs, vector<Pose2> &slam_est,
                    vector<MatrixXd> &slam_mcov, vector<Pose2> &new_cones,
                    vector<Pose2> &m_dist) {
    for (int i = 0; i < cone_obs.size(); i++) {
        for (int j = 0; j < slam_estimates.size(); j++) {
            MatrixXd diff = Pose2(global_cone_obs.at(i).x() - slam_est.at(j).x(),
                        global_cone_obs.at(i).y() - slam_est.at(j).y(),
                        1);
            m_dist.push_back((diff * slam_mcov(j) * diff.transpose())(0, 0));

        }

        m_dist.push_back(M_DIST_TH);
    }

}

void find_new_cones(vector<Pose2> &cone_obs, vector<Pose2> &m_dist, int n_landmarks,
            vector<tuple<Pose2, int> &old_cones, vector<Pose2> &new_cones,
            MatrixXd &global_cone_x, MatrixXd &global_cone_y) {
    int lo, hi = 0;
    for (int i = 0; i < cone_obs.size(); i++) {
        hi += n_landmarks + 1;
        vector<Pose2>::iterator min_dist = min_element(m_dist.begin() + lo,
                                                        m_dist.begin() + hi);
        int min_id = distance(m_dist.begin() + lo, min_dist);

        if (min_id == n_landmarks) {
            new_cones.push_back(make_tuple(cone_obs.at(i),
                                        Pose2(global_cone_x(i), global_cone_y(i), 1)));
        } else {
            old_cones.emplace_back(cone_obs.at(i), min_id);
        }

        lo = hi;
    }
}

void data_association(Pose2 &global_odom, Point2 &velocity,
                vector<Pose2> &cone_obs, vector<Pose2> &slam_est,
                vector<MatrixXd> &slam_mcov, vector<tuple<Pose2, int>> &old_cones,
                vector<tuple<Pose2, Pose2>> &new_cones) {

    vector<double> m_dist = {};
    int n_landmarks = slam_est.size();

    // Populating m_dist with mahalanobis distances
    MatrixXd global_cone_x = MatrixXd(cone_obs.size(), 1);
    MatrixXd global_cone_y = MatrixXd(cone_obs.size(), 1);

    cone_car_to_global_frame(cone_obs, global_odom, prev_pose,
                            global_cone_x, global_cone_y);

    populate_m_dist(global_cone_obs, slam_est,
                        slam_mcov, new_cones, m_dist);

    find_new_cones(cone_obs, m_dist, n_landmarks,
                    old_cones, new_cones,
                    global_cone_x, global_cone_y);


}




