/**
 * @file data_association.cpp
 * @brief Perform data association in preparation for feeding data to SLAM
 */
#include <vector>
#include <tuple>

#include <algorithm>
#include <float.h>
const double M_DIST_TH = 120;

using namespace Eigen;
void populate_m_dist(vector<Pose2> &blue_cones, vector<Pose2> &yellow_cones,
                    vector<Pose2> &slam_est, vector<MatrixXd> &slam_mcov,
                    vector<Pose2> &new_cones, vector<Pose2> &m_dist) {
    for (int b = 0; b < blue_cones.size(); b++) {
        for (int i = 0; i < slam_estimates.size(); i++) {
            MatrixXd diff = Pose2(blue_cones.at(b).x() - slam_est.at(i).x(),
                        blue_cones.at(b).y() - slam_est.at(i).y(),
                        1);
            m_dist.push_back((diff * slam_mcov(i) * diff.transpose())(0, 0));

        }

        m_dist.push_back(M_DIST_TH);
    }

    for (int y = 0; y < yellow_cones.size(); y++) {
        for (int i = 0; i < slam_estimates.size(); i++) {
            MatrixXd diff = Pose2(yellow_cones.at(y).x() - slam_est.at(i).x(),
                    yellow_cones.at(y).y() - slam_est.at(i).y(),
                    1);
            m_dist.push_back((diff  slam_mcov(i) * diff.transpose())(0, 0));
        }

        m_dist.push_back(M_DIST_TH);
    }


}

void find_new_cones(vector<Pose2> &m_dist, int n_landmarks,
            vector<tuple<Pose2, int> &old_cones, vector<Pose2> &new_cones) {
    int lo, hi = 0;
    for (int b = 0; b < blue_cones.size(); b++) {
        hi += n_landmarks + 1;
        vector<Pose2>::iterator min_dist = min_element(m_dist.begin() + lo,
                                                        m_dist.begin() + hi);
        int min_id = distance(m_dist.begin() + lo, min_dist);

        if (min_id == n_landmarks) {
            new_cones.push_back(blue_cones.at(b));
        } else {
            old_cones.emplace_back(blue_cones.at(b), min_id);
        }

        lo = hi;
    }

    for (int y = 0; y < yellow_cones.size(); y++) {
        hi += n_landmarks + 1;
        vector<Pose2>::iterator min_dist = min_element(m_dist.begin() + lo,
                                                        m_dist.begin() + hi);
        int min_id = distance(m_dist.begin() + lo, min_dist);

        if (min_id == n_landmarks) {
            new_cones.push_back(yellow_cones.at(y));
        } else {
            old_cones.emplace_back(yellow_cones.at(y), min_id);
        }

        lo = hi;
    }
}

void data_association(Pose2 &global_odom, Pose2 velocity,
            vector<Point2> &global_blue_cones, vector<Point2> &global_yellow_cones,
             vector<Point2> &orange_cones, vector<Pose2> &slam_est,
             vector<MatrixXd> &slam_mcov, vector<tuple<Pose2, int>> &old_cones,
             vector<Pose2> &new_cones) {

    vector<double> m_dist = {};
    int n_landmarks = slam_est.size();

    // Populating m_dist with mahalanobis distances
    populate_m_dist(blue_cones, yellow_cones, slam_est,
                        slam_mcov, new_cones, m_dist);

    find_new_cones(m_dist, n_landmarks, old_cones, new_cones);


}




