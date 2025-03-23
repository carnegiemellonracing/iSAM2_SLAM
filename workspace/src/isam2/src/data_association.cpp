/**
 * @file data_association.cpp
 * @brief Perform data association in preparation for feeding data to SLAM
 */
#include "data_association.hpp"
#include "ros_utils.hpp"
#include <cmath>
#define _USE_MATH_DEFINES


void populate_m_dist(MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                    int num_obs, vector<double> &m_dist, double threshold,
                    vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov,
                    optional<rclcpp::Logger> &logger) {
    for (int i = 0; i < num_obs; i++) {
        for (int j = 0; j < slam_est.size(); j++) {

            MatrixXd diff(1, 2);
            diff << global_cone_x(i,0) - slam_est.at(j).x(),
                    global_cone_y(i,0) - slam_est.at(j).y();

            m_dist.push_back((diff * slam_mcov.at(j) * diff.transpose())(0, 0));

        }

        m_dist.push_back(threshold);
    }

}

/* @brief Determines which of the observed cones are new or old 
 * @param old_cones a vector of tuples. Point2 is car-relative position of the cone
 *          double is the bearing from the car, and int is the ID assoc. with cone
 * 
 * @param new_cones a vector of tuples. Point2 is car-relative position of the cone
 *          double is the bearing from the car, and Point2 is the global position.
 * 
 */
void get_old_new_cones(vector<tuple<Point2, double, int>> &old_cones,
            vector<tuple<Point2, double, Point2>> &new_cones,
            MatrixXd &global_cone_x,MatrixXd &global_cone_y,MatrixXd &bearing,
            vector<Point2> &cone_obs, vector<double> &m_dist, int n_landmarks,
            optional<rclcpp::Logger> &logger) {

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
            old_cones.emplace_back(cone_obs.at(i), 
                                    bearing(i, 0), 
                                    min_id);
        }

        lo = hi;
    }
}

void data_association(vector<tuple<Point2, double, int>> &old_cones,
                vector<tuple<Point2, double, Point2>> &new_cones,
                Pose2 &cur_pose, Pose2 &prev_pose, bool is_turning,
                vector<Point2> &cone_obs, optional<rclcpp::Logger> &logger,
                vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov) {

    vector<double> m_dist = {};
    int n_landmarks = slam_est.size();

    double m_dist_th = M_DIST_TH;
    double cone_dist_th = MAX_CONE_RANGE;
    if (is_turning) {
        m_dist_th = TURNING_M_DIST_TH;
        cone_dist_th = TURNING_MAX_CONE_RANGE;
    }

    remove_far_cones(cone_obs, cone_dist_th);

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
    
    populate_m_dist(global_cone_x, global_cone_y, cone_obs.size(), m_dist,
                    m_dist_th, slam_est, slam_mcov, logger);

    get_old_new_cones(old_cones, new_cones,
                    global_cone_x, global_cone_y,bearing,
                    cone_obs, m_dist, n_landmarks, logger);

    //Iterate through all cones; store the ones closest to being immediately to 
    //the left and the right.
    int leftConeIdx = 0
    double leftConeRange = std::numeric_limits<double>::max()
    int rightConeIdx = 0
    double rightConeRange = std::numeric_limits<double>::max()

    for (int i = 0; i < old_cones.size(); i++) {

        //Check left
        if (M_PI/2 - bearings(i, 0) < M_PI/2 - bearings(leftConeIdx, 0)
            && range(i, 0) <= range(leftConeIdx, 0)) {
            leftConeIdx = i;
            leftConeRange = range(leftConeIdx, 0);
        }

        //Check right
        if (-M_PI/2 - bearings(i, 0) < -M_PI/2 - bearings(rightConeIdx, 0)
            && range(i, 0) <= range(rightConeIdx, 0)) {
            rightConeIdx = i;
            rightConeRange = range(rightConeIdx, 0);
        }
    }

    //Now that we have the cones immediately to our left and right, find the cones
    //with the bearings closest to zero
    // sort left cones
    std::vector<int> sortedLeftCones;
    currConeIdx = leftConeIdx
    size_t numSorted = 0
    size_t totalCones = new_cones.size();
    while (numSorted < (totalCones / 2)) {

        Point2 leftConePos;
        leftConePos.x = cur_pose.x + (range(leftConeIdx, 0) * std::cos(cur_pos.theta + bearings(leftConeIdx, 0)));
        leftConePos.y = cur_pose.y + (range(leftConeIdx, 0) * std::sin(cur_pos.theta + bearings(leftConeIdx, 0)));

        size_t leftCloseConeIdx = 0;
        double leftMinDistance = INT_MAX();
        size_t curr_idx = 0;

        for (auto& input_cone : new_cones) {

            double dx = input_cone.x - leftConePos.x;
            double dy = input_cone.y - leftConePos.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < leftMinDistance) {
                leftMinDistance = distance;
                leftCloseConeIdx = curr_idx;
            }
            curr_idx = curr_idx + 1;

        }
        currConeIdx = leftCloseConeIdx;
        sortedLeftCones.push_back(leftCloseConeIdx);
        numSorted = numSorted + 1;
    }

    // sort right cones
    std::vector<int> sortedRightCones;
    currConeIdx = rightConeIdx
    size_t numSorted = 0
    while (numSorted < (totalCones / 2)) {
        
        Point2 rightConePos;
        rightConePos.x = cur_pose.x + (range(rightConeIdx, 0) * std::cos(cur_pos.theta + bearings(rightConeIdx, 0)));
        rightConePos.y = cur_pose.y + (range(rightConeIdx, 0) * std::sin(cur_pos.theta + bearings(rightConeIdx, 0)));

        size_t rightCloseConeIdx = 0;
        double rightMinDistance = INT_MAX();
        size_t curr_idx = 0;

        for (auto& input_cone : new_cones) {

            double dx = input_cone.x - leftConePos.x;
            double dy = input_cone.y - leftConePos.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < rightMinDistance) {
                rightMinDistance = distance;
                rightCloseConeIdx = curr_idx;
            }
            curr_idx = curr_idx + 1;
            
        }
        currConeIdx = rightCloseConeIdx;
        sortedRightCones.push_back(rightCloseConeIdx);
        numSorted = numSorted + 1;
    }
};
