/**
 * @file slam_utils.cpp
 * @brief This file contains utility functions used for converting ROS messages
 * to appropriate data types and for any necessary calculations.
 */
using namespace std;
using namespace gtsam;
using namespace Eigen;

void cone_msg_to_vectors(interfaces::msg::ConeArray::SharedPtr cone_data,
                        vector<Point2> &blue_cones,
                        vector<Point2> &yellow_cones,
                        vector<Point2> &orange_cones) {
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

void velocity_msg_to_point2(geometry_msgs::msg::TwistStamped::SharedPtr vel_data,
                            Point2 &velocity) {
    velocity = Point2(vel_data.twist.linear.x, vel_data.twist.linear.y);
}

void quat_msg_to_yaw(geometry_msgs::msg::QuaternionStamped::SharedPtr ang_data,
                        int &yaw) {
    double qw = closest_angle->quaternion.w;
    double qx = closest_angle->quaternion.x;
    double qy = closest_angle->quaternion.y;
    double qz = closest_angle->quaternion.z;

    yaw = atan2(2 * (qz * qw + qx * qy),
                             -1 + 2 * (qw * qw + qx * qx));
}

void cones_pos_to_global_frame(vector<Point2> &cone_obs,) {

        Eigen::MatrixXd range(cone_obs.size(), 1);
	    Eigen::MatrixXd bearing(cone_obs.size(), 1);

	    int num_obs = (int)cone_obs.size();

	    for (int i = 0; i < num_obs; i++)
	    {
	        range(i) = norm2(cone_obs.at(i));
	    }

        for (int i = 0; i < num_obs; i++)
	    {
	        bearing(i) = atan2(cone_obs.at(i).y(), cone_obs.at(i).x());
	    }

	    Eigen::MatrixXd totalBearing = bearing.array()+ global_odom.theta();







