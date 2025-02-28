#pragma once

#include <vector>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <algorithm>
#include <float.h>
#include "ros_utils.hpp"

#include <cone_cache_type.h>

const double M_DIST_TH = 0.0009;
const double TURNING_M_DIST_TH = 0.0005;
// const double M_DIST_TH = 0.0005;
// const double TURNING_M_DIST_TH = 0.0005;
void populate_m_dist(MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                    int num_obs, std::vector<double> &m_dist, double threshold,
                    std::vector<Point2> &slam_est, std::vector<MatrixXd> &slam_mcov,
                    rclcpp::Logger &logger);

void get_old_new_cones(std::vector<tuple<Point2, double, int>> &old_cones,
            std::vector<tuple<Point2, double, Point2>> &candidate_new_cones,
            MatrixXd &global_cone_x,MatrixXd &global_cone_y,MatrixXd &bearing,
            std::vector<Point2> &cone_obs, std::vector<double> &m_dist, int n_landmarks,
            rclcpp::Logger &logger);

void data_association(std::vector<tuple<Point2, double, int>> &old_cones,
                      std::vector<tuple<Point2, double, Point2>> &candidate_new_cones,
                      std::vector<tuple<Point2, double, Point2>> &new_cones,
                      vector<ConeCacheType> &cone_cache,
                      Pose2 &cur_pose, Pose2 &prev_pose, bool is_turning,
                      std::vector<Point2> &cone_obs, optional<rclcpp::Logger> &logger,
                      std::vector<Point2> &slam_est, std::vector<MatrixXd> &slam_mcov,
                      noiseModel::Diagonal::shared_ptr &landmark_model);