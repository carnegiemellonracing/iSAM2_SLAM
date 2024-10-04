/**
 * @file slam_utils.cpp
 * @brief This file contains utility functions used for converting ROS messages
 * to appropriate data types and for any necessary calculations.
 */
#include <memory>

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
    y = x;
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
                            Point2 &velocity) {
    double dx = vehicle_vel_data->twist.linear.x;
    double dy = vehicle_vel_data->twist.linear.y;
    imu_axes_to_DV_axes(dx, dy);
    velocity = Point2(dx, dy);
}

void quat_msg_to_yaw(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data,
                        double &yaw) {
    double qw = vehicle_angle_data->quaternion.w;
    double qx = vehicle_angle_data->quaternion.x;
    double qy = vehicle_angle_data->quaternion.y;
    double qz = vehicle_angle_data->quaternion.z;
    imu_axes_to_DV_axes(qx, qy);

    yaw = atan2(2 * (qz * qw + qx * qy),
                             -1 + 2 * (qw * qw + qx * qx));
}

void cones_pos_to_global_frame(vector<Point2> &cone_obs, Pose2 &global_odom) {

        MatrixXd range(cone_obs.size(), 1);
	    MatrixXd bearing(cone_obs.size(), 1);

	    int num_obs = (int)cone_obs.size();

	    for (int i = 0; i < num_obs; i++)
	    {
	        range(i) = norm2(cone_obs.at(i));
	    }

        for (int i = 0; i < num_obs; i++)
	    {
	        bearing(i) = atan2(cone_obs.at(i).y(), cone_obs.at(i).x());
	    }

	    MatrixXd totalBearing = bearing.array() + global_odom.theta();
}

void motion_model(Pose2 &new_pose, Pose2 &odometry, Pose2 &velocity, double dt,
                    Pose2 &prev_pose, Pose2 global_odom, bool new_gps) {
    new_pose = Pose2(prev_pose.x() + velocity.x() * dt,
                            prev_pose.y() + velocity.y() * dt,
                            global_odom.theta());

    odometry = Pose2(velocity.x() * dt,
                            velocity.y() * dt,
                            global_odom.theta() - prev_pose.theta());
}


/* Vectorized functions */
void calc_cone_range(MatrixXd &range) {
    int num_obs = (int)cone_obs.size();

	for (int i = 0; i < num_obs; i++)
	{
	    range(i) = norm2(cone_obs.at(i));
	}
}

/** Bearing of cone from the car
 */
void calc_cone_bearing_from_car(MatrixXd &bearing, Pose2 &global_odom) {
    int num_obs = (int)cone_obs.size();

    for (int i = 0; i < num_obs; i++)
    {
        bearing(i) = atan2(cone_obs.at(i).y(), cone_obs.at(i).x());
    }

    /* Putting the bearing in global frame */
    bearing = bearing.array() + global_odom.theta();

}

void cone_car_to_global_frame(vector<Pose2> &cone_obs,
                            Pose2 &global_odom, Pose2 &prev_pose,
                            MatrixXd global_cone_x, MatrixXd global_cone_y) {

    MatrixXd range(cone_obs.size(), 1);
    MatrixXd bearing(cone_obs.size(), 1);


    calc_cone_range(range);
    calc_cone_bearing_from_car(bearing, global_odom);
    global_cone_x = prev_pose.x() + range.array()*bearing.array().cos();
    global_cone_y = prev_pose.y() + range.array()*bearing.array().sin();

}












