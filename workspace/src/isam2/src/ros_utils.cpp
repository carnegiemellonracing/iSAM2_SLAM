/**
 * @file ros_utils.cpp
 * @brief This file contains utility functions used for converting ROS messages
 * to appropriate data types and for any necessary calculations.
 *
 * Most functions in this value pass in the result container by reference.
 * This reference is modified to store the results. The result reference
 * is always the first parameter that is passed in.
 */
#include "ros_utils.hpp"

/**
 * @brief Movella Xsens IMU uses y-left, x-forward axes. CMR DV uses y-forward
 * x-right axes. This function performs the conversion.
 */
void imu_axes_to_DV_axes(double &x, double &y) {
    double temp_x = x;
    x = -1 * y;
    y = temp_x;
}

void vector3_msg_to_gps(Pose2 &global_odom, const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data,
                         optional<Point2> &init_lon_lat, rclcpp::Logger logger) {
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



    global_odom = Pose2(x, y, global_odom.theta());

}

void cone_msg_to_vectors(vector<Point2> &cones,vector<Point2> &blue_cones,vector<Point2> &yellow_cones,
                        vector<Point2> &orange_cones,const interfaces::msg::ConeArray::ConstSharedPtr &cone_data) {
    // Trying to find a library that converts ros2 msg array to vector
    for (std::size_t i = 0; i < cone_data->blue_cones.size(); i++) {
        Point2 to_add = Point2(cone_data->blue_cones.at(i).x,
                               cone_data->blue_cones.at(i).y);
        blue_cones.push_back(to_add);
        cones.push_back(to_add);
    }

    for (std::size_t i = 0; i < cone_data->yellow_cones.size(); i++) {
        Point2 to_add = Point2(cone_data->yellow_cones.at(i).x,
                               cone_data->yellow_cones.at(i).y);
        yellow_cones.push_back(to_add);
        cones.push_back(to_add);
    }

    for (std::size_t i = 0; i < cone_data->orange_cones.size(); i++) {
        Point2 to_add = Point2(cone_data->orange_cones.at(i).x,
                               cone_data->orange_cones.at(i).y);
        orange_cones.push_back(to_add);
        cones.push_back(to_add);
    }
}

void velocity_msg_to_pose2(Pose2 &velocity, const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data) {
    double dx = vehicle_vel_data->twist.linear.x;
    double dy = vehicle_vel_data->twist.linear.y;
    double dw = vehicle_vel_data->twist.angular.z;
    
    velocity = Pose2(dx, dy, dw);
}

void quat_msg_to_yaw(Pose2 &global_odom,
            const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data, rclcpp::Logger logger) {
    double qw = vehicle_angle_data->quaternion.w;
    double qx = vehicle_angle_data->quaternion.x;
    double qy = vehicle_angle_data->quaternion.y;
    double qz = vehicle_angle_data->quaternion.z;

    double yaw = atan2(2 * (qz * qw + qx * qy), -1 + 2 * (qw * qw + qx * qx));
    RCLCPP_INFO(logger, "yaw: %f", yaw);
    global_odom = Pose2(global_odom.x(), global_odom.y(), yaw);
}

/**
 * @brief Calculates the lateral velocity error as a result of turning.
 *        The IMU is offset from the center of the car. There's extra lateral
 *        velocity error due to the offset.
 */
void calc_lateral_velocity_error(double& dx_error, double& dy_error, 
                                double ang_velocity, double yaw) {
    /** Intuition: In the local frame, create a triangle T from the global bearing of the car
     *  
     * ang_velocity represents the magnitude of the angular velocity vector
     * - To get the x component (global frame x), 
     *      let the vertical side of the triangle be the radius
     * 
     * - To get the y component (global frame y),
     *      let the horizontal side of the triangle be the radius
     */
    dx_error = ang_velocity * (IMU_OFFSET * sin(yaw));
    dy_error = ang_velocity * (IMU_OFFSET * cos(yaw));
}

void velocity_motion_model(Pose2 &new_pose, Pose2 &odometry, Pose2 &velocity, 
                            double dt, Pose2 &prev_pose, Pose2 global_odom) {
    
    
    double dx_error = 0;
    double dy_error = 0;

    calc_lateral_velocity_error(dx_error, dy_error, velocity.theta(), global_odom.theta());
    
    double dx = velocity.x() - dx_error;
    double dy = velocity.y() - dy_error; 

    new_pose = Pose2(prev_pose.x() + dx * dt,
                    prev_pose.y() + dy * dt,
                    global_odom.theta());

    //TODO: how does change in theta results differ from dt * velocity.z()
    odometry = Pose2(dx * dt,
                    dy * dt,
                    global_odom.theta() - prev_pose.theta());
}

void calc_offset_imu_to_car_center(double& offset_x, double& offset_y, double yaw) {
    offset_x = IMU_OFFSET * cos(yaw);
    offset_y = IMU_OFFSET * sin(yaw);
}

void calc_offset_lidar_to_car_center(double& offset_x, double& offset_y, double yaw) {
    offset_x = LIDAR_OFFSET * cos(yaw);
    offset_y = LIDAR_OFFSET * sin(yaw);
}

void gps_motion_model(Pose2 &new_pose, Pose2 &odometry, Pose2 &velocity, double dt,
                    Pose2 &prev_pose, Pose2 global_odom) {
    double offset_x = 0;
    double offset_y = 0;
    calc_offset_imu_to_car_center(offset_x, offset_y, global_odom.theta());
    new_pose = Pose2(global_odom.x() - offset_x, global_odom.y() - offset_y, global_odom.theta());
    odometry = Pose2(new_pose.x() - prev_pose.x(),
                        new_pose.y() - prev_pose.y(),
                        global_odom.theta() - prev_pose.theta());
}

void determine_movement(bool &is_moving, bool &is_turning, Pose2 &velocity) {
    if (sqrt(pow(velocity.x(), 2) + pow(velocity.y(), 2)) > VELOCITY_MOVING_TH) {
        is_moving = true;
    } else {
        is_moving = false;
    }
    if (abs(velocity.theta()) > TURNING_TH) {
        is_turning = true;
    } else {
        is_turning = false;
    }
}

double header_to_nanosec(const std_msgs::msg::Header &header) {
    return (header.stamp.sec * (double)1e9) + header.stamp.nanosec;
}

void header_to_dt(const optional<std_msgs::msg::Header> &prev, const optional<std_msgs::msg::Header> &cur, double &dt) {
    // dt units is meters per second
    dt = (header_to_nanosec(cur.value()) - header_to_nanosec(prev.value())) * 1e-9;
}
/**
 * @brief Removes far away observed cones.Observed cones that are far away are more erroneous
 */
void remove_far_cones(vector<Point2> &cone_obs, double threshold) {
    for (std::size_t i = 0; i < cone_obs.size(); i++) {
        if (norm2(cone_obs.at(i)) > threshold) {
            cone_obs.erase(cone_obs.begin() + i);
            i--;
        }
    }
    
}

/* Vectorized functions */
void calc_cone_range_from_car(MatrixXd &range, vector<Point2> &cone_obs) {
    

	for (size_t i = 0; i < cone_obs.size(); i++)
	{
	    range(i,0) = norm2(cone_obs.at(i));
	}
}

/** Bearing of cone from the car
 */
void calc_cone_bearing_from_car(MatrixXd &bearing, vector<Point2> &cone_obs) {

    for (size_t i = 0; i < cone_obs.size(); i++)
    {   
        /**
         * Intuition:
         * Perceptions gives us cones in DV axes: y forwards and x right
         * 
         * Directly in front of us, the bearing is 0
         * - The bearing is the angle from the axis directly in front of us
         * - counter clockwise is positive bearing
         * - clockwise is negative bearing
         * 
         * Given the DV axes, to calculate the bearing is atan2(-x, y)
         * - If the bearing was to the left (counter clockwise from 0)
         * - The bearing would be a positive angle 
         * 
         */
        bearing(i,0) = atan2(-cone_obs.at(i).x(), cone_obs.at(i).y());
    }
}

void cone_to_global_frame(MatrixXd &range, MatrixXd &bearing,
                            MatrixXd &global_cone_x, MatrixXd &global_cone_y,
                            vector<Point2> &cone_obs, Pose2 &cur_pose) {
    double lidar_offset_x = 0;
    double lidar_offset_y = 0;
    calc_offset_lidar_to_car_center(lidar_offset_x, lidar_offset_y, cur_pose.theta());
    MatrixXd global_bearing = bearing.array() + cur_pose.theta();
    global_cone_x = cur_pose.x() + range.array()*global_bearing.array().cos();
    global_cone_x = global_cone_x.array() + lidar_offset_x;
    global_cone_y = cur_pose.y() + range.array()*global_bearing.array().sin();
    global_cone_y = global_cone_y.array() + lidar_offset_y;
    

}

double degrees_to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

void print_cone_obs(vector<Point2> &cone_obs, optional<rclcpp::Logger> logger) {
    for (std::size_t i = 0; i < cone_obs.size(); i++) {
        if (logger.has_value()) {
            RCLCPP_INFO(logger.value(), "cone_obs.at(%ld): %f %f", i, 
                                                    cone_obs.at(i).x(),
                                                    cone_obs.at(i).y());
        }
    }
}

void print_step_input(optional<rclcpp::Logger> logger, gtsam::Pose2 global_odom, vector<Point2> &cone_obs,
                vector<Point2> &cone_obs_blue, vector<Point2> &cone_obs_yellow,
                vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity, double dt) {
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "PRINTING STEP INPUTS");
        RCLCPP_INFO(logger.value(), "global_odom: %f %f %f", global_odom.x(), global_odom.y(), global_odom.theta());
        print_cone_obs(cone_obs, logger);

    // At the moment, cone_obs_blue and cone_obs_yellow are empty because we don't have coloring
    // Similarly for the orange_ref_cones
        RCLCPP_INFO(logger.value(), "velocity: %f %f %f", velocity.x(), velocity.y(), velocity.theta());
        RCLCPP_INFO(logger.value(), "dt: %f", dt);
    }
}
    
                        
void print_update_poses(Pose2 &prev_pose, Pose2 &new_pose, Pose2 &odometry, Pose2 &imu_offset_global_odom, optional<rclcpp::Logger> logger) {
    if (logger.has_value()) {
        RCLCPP_INFO(logger.value(), "\tLOGGING UPDATE_POSES POSE2 VARIABLES:");
        RCLCPP_INFO(logger.value(), "prev_pose: %f %f %f", prev_pose.x(), prev_pose.y(), prev_pose.theta());
        RCLCPP_INFO(logger.value(), "new_pose: %f %f %f", new_pose.x(), new_pose.y(), new_pose.theta());
        RCLCPP_INFO(logger.value(), "odometry: %f %f %f", odometry.x(), odometry.y(), odometry.theta());
        RCLCPP_INFO(logger.value(), "imu_offset_global_odom: %f %f %f", imu_offset_global_odom.x(), 
                                                                imu_offset_global_odom.y(), 
                                                                imu_offset_global_odom.theta());
        RCLCPP_INFO(logger.value(), "\n");
    }
}

void log_step_inputs(optional<rclcpp::Logger> logger, gtsam::Pose2 global_odom, vector<Point2> &cone_obs,
                vector<Point2> &cone_obs_blue, vector<Point2> &cone_obs_yellow,
                vector<Point2> &orange_ref_cones, gtsam::Pose2 velocity, double dt) {

    ofstream outfile;
    outfile.open(STEP_INPUT_FILE, ofstream::out | ios_base::app); 

    outfile << "global_odom: " << global_odom.x() << " " << global_odom.y() << " " << global_odom.theta() << endl;
    for (size_t i = 0; i < cone_obs.size(); i++) {
        outfile << "cone_obs: " << cone_obs.at(i).x() << " " << cone_obs.at(i).y() << endl;
    }
    outfile << "velocity: " << velocity.x() << " " << velocity.y() << " " << velocity.theta() << endl;
    outfile << "dt: " << dt << "\n" << endl;

    outfile.close();

}