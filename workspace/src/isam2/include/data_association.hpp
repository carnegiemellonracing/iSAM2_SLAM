#pragma once

#include <vector>
#include <tuple>

#include <Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <algorithm>
#include <float.h>
#include "ros_utils.hpp"
const double M_DIST_TH = 50.0;


void populate_m_dist(MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                    int num_obs, vector<double> &m_dist,
                    vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov,
                    rclcpp::Logger &logger);

void find_new_cones(vector<tuple<Point2, double, int>> &old_cones,
            vector<tuple<Point2, double, Point2>> &new_cones,
            MatrixXd &global_cone_x,MatrixXd &global_cone_y,MatrixXd &bearing,
            vector<Point2> &cone_obs, vector<double> &m_dist, int n_landmarks,
            rclcpp::Logger &logger);

void data_association(vector<tuple<Point2, double, int>> &old_cones,
                vector<tuple<Point2, double, Point2>> &new_cones,
                Pose2 &cur_pose, Pose2 &prev_pose,
                vector<Point2> &cone_obs, optional<rclcpp::Logger> &logger,
                vector<Point2> &slam_est, vector<MatrixXd> &slam_mcov);