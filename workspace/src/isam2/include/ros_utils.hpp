#pragma once

#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "interfaces/msg/cone_array.hpp"
#include "interfaces/msg/cone_array_with_odom.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <boost/shared_ptr.hpp>
#include <vector>
#include <fstream>
#include <deque>
#include <cmath>
#include <chrono>
#include <climits>
#include <fmt/format.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include "constants.hpp"

using namespace std;
using namespace gtsam;
using namespace Eigen;
using namespace std::chrono;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;



void log_string (std::optional<rclcpp::Logger> logger, std::string input_string, bool flag);

void imu_axes_to_DV_axes(double &x, double &y);

/* Functions for converting ros messages to relevant data types */
void cone_msg_to_vectors(const interfaces::msg::ConeArray::ConstSharedPtr &cone_data,
                                            vector<Point2> &cones,
                                            vector<Point2> &blue_cones,
                                            vector<Point2> &yellow_cones,
                                            vector<Point2> &orange_cones);
void velocity_msg_to_pose2(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data,
                            Pose2 &velocity);

void posestamped_msg_to_gps(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &vehicle_pos_data,
                        Pose2 &global_odom, optional<Point2> &init_x_y, rclcpp::Logger logger);

void quat_msg_to_yaw(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data,
                        double &yaw, Pose2 &global_odom, rclcpp::Logger logger);
void vector3_msg_to_gps(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data,
                        Pose2 &global_odom, optional<Point2> &init_lon_lat, rclcpp::Logger logger);

/* Utility functions for motion modeling */
void calc_lateral_velocity_error(double& dx_error, double& dy_error, 
                                double ang_velocity, double yaw);

void velocity_motion_model(Pose2 &new_pose, Pose2 &odometry,Pose2 &velocity, double dt,
                    Pose2 &prev_pose, Pose2 global_odom);

void calc_offset_imu_to_car_center(double& offset_x, double& offset_y, double yaw);
void calc_offset_lidar_to_car_center(double& offset_x, double& offset_y, double yaw);
void gps_motion_model(Pose2 &new_pose, Pose2 &odometry, Pose2 &velocity, double dt,
                    Pose2 &prev_pose, Pose2 global_odom);

void determine_movement(bool &is_moving, bool &is_turning, Pose2 &velocity);

void remove_far_cones(std::vector<gtsam::Point2> &cone_obs, double threshold);

double header_to_nanosec(const std_msgs::msg::Header &header);
void header_to_dt(const optional<std_msgs::msg::Header> &prev, const optional<std_msgs::msg::Header> &cur, double &dt);

/* Calculations for cone positions */
void calc_cone_range_from_car(MatrixXd &range, vector<Point2> &cone_obs);
void calc_cone_bearing_from_car(MatrixXd &bearing, vector<Point2> &cone_obs);
void cone_to_global_frame(MatrixXd &range, MatrixXd &bearing,
                            MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                            vector<Point2> &cone_obs, Pose2 &cur_pose);
double degrees_to_radians(double degrees);

void print_cone_obs(vector<Point2> &cone_obs, optional<rclcpp::Logger> logger);

void print_step_input(optional<rclcpp::Logger> logger, gtsam::Pose2 global_odom, vector<Point2> &cone_obs,
                vector<Point2> &cone_obs_blue, vector<Point2> &cone_obs_yellow,
                vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity, double dt);

void print_update_poses(Pose2 &prev_pose, Pose2 &new_pose, Pose2 &odometry, Pose2 &imu_offset_global_odom, optional<rclcpp::Logger> logger);

void log_step_inputs(optional<rclcpp::Logger> logger, gtsam::Pose2 global_odom, vector<Point2> &cone_obs,
                vector<Point2> &cone_obs_blue, vector<Point2> &cone_obs_yellow,
                vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity, double dt);
