/**
 * @file slam_utils.cpp
 * @brief This file contains utility functions used for converting ROS messages
 * to appropriate data types and for any necessary calculations.
 *
 * Most functions in this value pass in the result container by reference.
 * This reference is modified to store the results. The result reference
 * is always the first parameter that is passed in.
 */
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "interfaces/msg/cone_array.hpp"
#include "interfaces/msg/cone_array_with_odom.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "eufs_msgs/msg/car_state.hpp"

#include <gtsam/nonlinear/ISAM2Params.h>

#include <boost/shared_ptr.hpp>
#include <vector>
#include <deque>
#include <cmath>
#include <chrono>
using namespace std;
using namespace gtsam;
using namespace Eigen;

/**
 * @brief Movella Xsens IMU uses y-left, x-forward axes. CMR DV uses y-forward
 * x-right axes. This function performs the conversion.
 */
void imu_axes_to_DV_axes(double &x, double &y) {
    double temp_x = x;
    x = -1 * y;
    y = temp_x;
}

void cone_msg_to_vectors(const interfaces::msg::ConeArray::ConstSharedPtr &cone_data,
                                            vector<Point2> &cones,
                                            vector<Point2> &blue_cones,
                                            vector<Point2> &yellow_cones,
                                            vector<Point2> &orange_cones) {
    // Trying to find a library that converts ros2 msg array to vector
    for (uint i = 0; i < cone_data->blue_cones.size(); i++) {
        Point2 to_add = Point2(cone_data->blue_cones.at(i).x,
                               cone_data->blue_cones.at(i).y);
        blue_cones.push_back(to_add);
        cones.push_back(to_add);
    }

    for (uint i = 0; i < cone_data->yellow_cones.size(); i++) {
        Point2 to_add = Point2(cone_data->yellow_cones.at(i).x,
                               cone_data->yellow_cones.at(i).y);
        yellow_cones.push_back(to_add);
        cones.push_back(to_add);
    }

    for (uint i = 0; i < cone_data->orange_cones.size(); i++) {
        Point2 to_add = Point2(cone_data->orange_cones.at(i).x,
                               cone_data->orange_cones.at(i).y);
        orange_cones.push_back(to_add);
        cones.push_back(to_add);
    }
}

void velocity_msg_to_point2(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data,
                            Point2 &init_velocity, Point2 &velocity) {
    double dx = vehicle_vel_data->twist.linear.x;
    double dy = vehicle_vel_data->twist.linear.y;
    imu_axes_to_DV_axes(dx, dy);
    velocity = Point2(dx, dy);
}

void quat_msg_to_yaw(
    const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data,
                        double &yaw, Pose2 &global_odom, rclcpp::Logger logger) {
    double qw = vehicle_angle_data->quaternion.w;
    double qx = vehicle_angle_data->quaternion.x;
    double qy = vehicle_angle_data->quaternion.y;
    double qz = vehicle_angle_data->quaternion.z;

    double imu_yaw = atan2(2 * (qz * qw + qx * qy),
                    -1 + 2 * (qw * qw + qx * qx));
    yaw = imu_yaw + (M_PI / 2.0);

    global_odom = Pose2(global_odom.x(), global_odom.y(), yaw);
}

void velocity_motion_model(Pose2 &new_pose, Pose2 &odometry, Point2 &velocity, double dt,
                    Pose2 &prev_pose, Pose2 global_odom) {
    
    new_pose = Pose2(prev_pose.x() + velocity.x() * dt,
                            prev_pose.y() + velocity.y() * dt,
                            global_odom.theta());

    odometry = Pose2(velocity.x() * dt,
                            velocity.y() * dt,
                            global_odom.theta() - prev_pose.theta());
}

void gps_motion_model(Pose2 &new_pose, Pose2 &odometry, Point2 &velocity, double dt,
                    Pose2 &prev_pose, Pose2 global_odom) {
    new_pose = Pose2(global_odom.x(), global_odom.y(), global_odom.theta());
    odometry = Pose2(new_pose.x() - prev_pose.x(),
                        new_pose.y() - prev_pose.y(),
                        global_odom.theta() - prev_pose.theta());
}

void gps_velocity_motion_model(Pose2 &new_pose, Pose2 &odometry, Point2 &velocity, double dt,
                    Pose2 &prev_pose, Pose2 global_odom) {
    /* new_pose is calculated using GPS. odometry uses velocity */
    new_pose = Pose2(global_odom.x(), global_odom.y(), global_odom.theta());
    odometry = Pose2(velocity.x() * dt, velocity.y() * dt, global_odom.theta() - prev_pose.theta());

}

double header_to_nanosec(const std_msgs::msg::Header &header) {
    return (header.stamp.sec * (double)1e9) + header.stamp.nanosec;
}

void header_to_dt(const optional<std_msgs::msg::Header> &prev, const optional<std_msgs::msg::Header> &cur, double &dt) {
    // dt units is meters per second
    dt = (header_to_nanosec(cur.value()) - header_to_nanosec(prev.value())) * 1e-9;
}

/* Vectorized functions */
void calc_cone_range_from_car(MatrixXd &range, vector<Point2> &cone_obs) {
    int num_obs = (int)cone_obs.size();

	for (int i = 0; i < num_obs; i++)
	{
	    range(i,0) = norm2(cone_obs.at(i));
	}
}

/** Bearing of cone from the car
 */
void calc_cone_bearing_from_car(MatrixXd &bearing, vector<Point2> &cone_obs) {
    int num_obs = (int)cone_obs.size();

    for (int i = 0; i < num_obs; i++)
    {
        bearing(i,0) = atan2(-cone_obs.at(i).x(), cone_obs.at(i).y());
    }
}

void cone_to_global_frame(MatrixXd &range, MatrixXd &bearing,
                            MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                            vector<Point2> &cone_obs, Pose2 &cur_pose) {

    MatrixXd global_bearing = bearing.array() + cur_pose.theta();
    global_cone_x = cur_pose.x() + range.array()*global_bearing.array().cos();
    global_cone_y = cur_pose.y() + range.array()*global_bearing.array().sin();

}

double degrees_to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

void vector3_msg_to_gps(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data,
                        Pose2 &global_odom, optional<Point2> &init_lon_lat, rclcpp::Logger logger) {
    /* Doesn't depend on imu axes. These are global coordinates */
    double latitude = vehicle_pos_data->vector.x;
    double longitude = vehicle_pos_data->vector.y;

    if (!(init_lon_lat.has_value())) {
        init_lon_lat.emplace(longitude, latitude);
    }

    longitude -= init_lon_lat.value().x();
    latitude -= init_lon_lat.value().y();
    /* Print statements for longitude and latitude debugging
    RCLCPP_INFO(logger, "init_lon_lat: %.10f | %.10f", init_lon_lat.value().x(),
                                                init_lon_lat.value().y());
    RCLCPP_INFO(logger, "cur lon_lat: %.10f | %.10f", longitude, latitude);
    RCLCPP_INFO(logger, "cur change in lon_lat: %.10f | %.10f",
                                            longitude, latitude); */

    double LAT_DEG_TO_METERS = 111111;

    /* Intuition: Find the radius of the circle at current latitude
     * Convert the change in longitude to radians
     * Calculate the distance in meters
     *
     * The range should be the earth's radius: 6378137 meters.
     * The radius of the circle at current latitude: 6378137 * cos(latitude_rads)
     * To get the longitude, we need to convert change in longitude to radians
     * - longitude_rads = longitude * (M_PI / 180.0)
     *
     * Observe: 111320 = 6378137 * M_PI / 180.0
     */

    /* Represents the radius used to multiply angle in radians */
    double LON_DEG_TO_METERS = 111319.5 * cos(degrees_to_radians(latitude));

    double x = LON_DEG_TO_METERS * longitude;
    double y = LAT_DEG_TO_METERS * latitude;
    imu_axes_to_DV_axes(x, y);



    global_odom = Pose2(x, y, global_odom.theta());

}
