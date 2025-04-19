#pragma once

#include <vector>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <algorithm>
#include <float.h>
#include "ros_utils.hpp"


struct New_cone_info {
    Point2 local_cone_pos;
    double bearing;
    Point2 global_cone_pos;
    New_cone_info (Point2 local_cone_pos, double bearing, Point2 global_cone_pos)
        : local_cone_pos(local_cone_pos)
        , bearing(bearing)
        , global_cone_pos(global_cone_pos)
    {}
};

struct Old_cone_info {
    Point2 local_cone_pos;
    double bearing;
    int min_id; // The id of the old cone observed cone was associated with
    Old_cone_info (Point2 local_cone_pos, double bearing, int min_id)
        : local_cone_pos(local_cone_pos)
        , bearing(bearing)
        , min_id(min_id)
    {}
};



void populate_m_dist(Eigen::MatrixXd &global_cone_x, Eigen::MatrixXd &global_cone_y,
                    int num_obs, std::vector<double> &m_dist, double threshold,
                    std::vector<Point2> &slam_est, std::vector<Eigen::MatrixXd> &slam_mcov,
                    rclcpp::Logger &logger);

void get_old_new_cones(std::vector<Old_cone_info> &old_cones,
            std::vector<New_cone_info> &new_cones,
            MatrixXd &global_cone_x,MatrixXd &global_cone_y,MatrixXd &bearing,
            std::vector<Point2> &cone_obs, std::vector<double> &m_dist, int n_landmarks,
            rclcpp::Logger &logger);

void data_association(std::vector<Old_cone_info> &old_cones,
                std::vector<New_cone_info> &new_cones,
                gtsam::Pose2 &cur_pose, gtsam::Pose2 &prev_pose,
                std::vector<gtsam::Point2> &cone_obs, std::optional<rclcpp::Logger> &logger,
                std::vector<gtsam::Point2> &slam_est, std::vector<MatrixXd> &slam_mcov, double m_dist_th, double cone_dist_th);
