#include "isam2_pkg.hpp"



class SLAMValidation : public rclcpp::Node {
public:
    SLAMValidation(): Node("slam_validation")
    {
        const rmw_qos_profile_t best_effort_profile = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            30,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };

        const rclcpp::QoS best_effort_qos = rclcpp::QoS(
           rclcpp::QoSInitialization(
               best_effort_profile.history,
               best_effort_profile.depth),
           best_effort_profile);

        cone_sub.subscribe(this, CONE_DATA_TOPIC, best_effort_profile);
        vehicle_pos_sub.subscribe(this, VEHICLE_POS_TOPIC, best_effort_profile);
        vehicle_angle_sub.subscribe(this, VEHICLE_ANGLE_TOPIC, best_effort_profile);
        vehicle_vel_sub.subscribe(this, VEHICLE_VEL_TOPIC, best_effort_profile);

        sync = std::make_shared<message_filters::Synchronizer<
                                    message_filters::sync_policies::ApproximateTime<
                                    interfaces::msg::ConeArray,
                                    geometry_msgs::msg::Vector3Stamped,
                                    geometry_msgs::msg::TwistStamped,
                                    geometry_msgs::msg::QuaternionStamped>>>(
                            message_filters::sync_policies::ApproximateTime<
                                    interfaces::msg::ConeArray,
                                    geometry_msgs::msg::Vector3Stamped,
                                    geometry_msgs::msg::TwistStamped,
                                    geometry_msgs::msg::QuaternionStamped>(30),
                                    cone_sub, vehicle_pos_sub, vehicle_vel_sub,vehicle_angle_sub);
        sync->setAgePenalty(0.09);
        sync->registerCallback(std::bind(&SLAMValidation::sync_callback, this, _1, _2, _3, _4));

        dt = .1;

        //TODO: std::optional where init is set to None
        init_lon_lat = std::nullopt;
        file_opened = true;

        prev_filter_time = std::nullopt;

        prev_sync_callback_time = std::nullopt;
    }

private:

    void sync_callback(const interfaces::msg::ConeArray::ConstSharedPtr &cone_data,
                    const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data,
                    const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data,
                    const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data) {
        RCLCPP_INFO(this->get_logger(), "\nSync Callback");
        
        /* Getting the time between sync callbacks */
        cur_sync_callback_time = high_resolution_clock::now();
        if (prev_sync_callback_time.has_value()) {
            auto time_betw_sync_callbacks = duration_cast<milliseconds>(cur_sync_callback_time - prev_sync_callback_time.value());
            RCLCPP_INFO(this->get_logger(), "\tTime between sync_callbacks: %ld", time_betw_sync_callbacks.count());
        }
        


        auto sync_data_start = high_resolution_clock::now();
        optional<std_msgs::msg::Header> cur_filter_time(vehicle_pos_data->header);
        if (!prev_filter_time.has_value()) {
            prev_filter_time.swap(cur_filter_time);
            return;
        }

        header_to_dt(prev_filter_time, cur_filter_time, dt);
        prev_filter_time.swap(cur_filter_time);

        /* Cone callback */
        cone_callback(cone_data);
        
        /* Vehicle position callback */
        vehicle_pos_callback(vehicle_pos_data);

        /* Vehicle velocity callback */
        vehicle_vel_callback(vehicle_vel_data);

        /* Vehicle angle callback */
        vehicle_angle_callback(vehicle_angle_data);
        

        auto sync_data_end = high_resolution_clock::now();
        auto sync_data_duration = duration_cast<milliseconds>(sync_data_end - sync_data_start);
        RCLCPP_INFO(this->get_logger(), "\tSync callback time: %ld \n", sync_data_duration.count());


        run_slam();

        prev_sync_callback_time.emplace(high_resolution_clock::now());
    }


    /* Whenever cone_callback is called, update car pose variables
     * by finding the ones that best match up to the time stamp of cone_data
     */
    void cone_callback(const interfaces::msg::ConeArray::ConstSharedPtr &cone_data)
    {
        // RCLCPP_INFO(this->get_logger(), "\t cone_callback!");
        auto cone_callback_start = high_resolution_clock::now();
        cones = {};

        /* issue these could clear before you're finished with data association. */
        blue_cones = {};
        yellow_cones = {};
        orange_cones = {};

        /* Process cones */
        cone_msg_to_vectors(cone_data, cones, blue_cones, yellow_cones, orange_cones);

        /* Timers */
        auto cone_callback_end = high_resolution_clock::now();
        auto cone_callback_duration = duration_cast<milliseconds>(cone_callback_end - cone_callback_start);
        RCLCPP_INFO(this->get_logger(), "\tCone callback time: %ld", cone_callback_duration.count());

    }

    void vehicle_pos_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data)
    {
        // RCLCPP_INFO(this->get_logger(), "\t vehicle position callback! | time: %d\n",
        //                                         vehicle_pos_data->header.stamp.sec);
        auto vehicle_pos_callback_start = high_resolution_clock::now();
        
        vector3_msg_to_gps(vehicle_pos_data, global_odom, init_lon_lat, this->get_logger());

        /* Timers*/
        auto vehicle_pos_callback_end = high_resolution_clock::now();
        auto vehicle_pos_callback_duration = duration_cast<milliseconds>(vehicle_pos_callback_end - vehicle_pos_callback_start);
        RCLCPP_INFO(this->get_logger(), "\tPosition callback time: %ld", vehicle_pos_callback_duration.count());
    }



    void vehicle_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data)
    {
        // RCLCPP_INFO(this->get_logger(), "\t vehicle velocity callback! | time: %d\n",
        //                                         vehicle_vel_data->header.stamp.sec);
        auto vehicle_vel_callback_start = high_resolution_clock::now();
        velocity_msg_to_pose2(vehicle_vel_data, velocity);

        /* Timers*/
        auto vehicle_vel_callback_end = high_resolution_clock::now();
        auto vehicle_vel_callback_duration = duration_cast<milliseconds>(vehicle_vel_callback_end - vehicle_vel_callback_start);
        RCLCPP_INFO(this->get_logger(), "\tVelocity callback time: %ld", vehicle_vel_callback_duration.count());
    }

    void vehicle_angle_callback(
        const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data)
    {
        // RCLCPP_INFO(this->get_logger(), "\t vehicle angle callback! | time: %d\n",
        //                                       vehicle_angle_data->header.stamp.sec);
        auto vehicle_angle_callback_start = high_resolution_clock::now();
        double yaw = 0;
        quat_msg_to_yaw(vehicle_angle_data, yaw, global_odom, this->get_logger());

        /* Timers*/
        auto vehicle_angle_callback_end = high_resolution_clock::now();
        auto vehicle_angle_callback_duration = duration_cast<milliseconds>(vehicle_angle_callback_end - vehicle_angle_callback_start);
        RCLCPP_INFO(this->get_logger(), "\tAngle callback time: %ld", vehicle_angle_callback_duration.count());
    }

    void run_slam()
    {
        RCLCPP_INFO(this->get_logger(), "Running SLAM");

       /* We should be passing in odometry info so that SLAM can do motion modeling.
	    * At each time stamp, we either:
	    * a.) Receive GPS message:
	    * When we do, we want to incorporate that into our motion modeling
	    *
	    * b.) Don't receive GPS message:
	    * When we don't we want to use velocity and our SLAM estimate
	    * to model our new pose */
        slam_instance.step(global_odom, cones, blue_cones,
                  yellow_cones, orange_cones, velocity, dt);


    }


    slamISAM slam_instance = slamISAM(this->get_logger());
    high_resolution_clock::time_point cur_sync_callback_time;
    optional<high_resolution_clock::time_point> prev_sync_callback_time;
    message_filters::Subscriber<interfaces::msg::ConeArray> cone_sub;
    message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> vehicle_pos_sub;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> vehicle_vel_sub;
    message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> vehicle_angle_sub;

    std::shared_ptr<message_filters::Synchronizer<
                            message_filters::sync_policies::ApproximateTime<
                                            interfaces::msg::ConeArray,
                                            geometry_msgs::msg::Vector3Stamped,
                                            geometry_msgs::msg::TwistStamped,
                                            geometry_msgs::msg::QuaternionStamped>>> sync;
    
    gtsam::Pose2 velocity;

    optional<gtsam::Point2> init_lon_lat; // local variable to load odom into SLAM instance
    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    gtsam::Pose2 prev_odom;
    vector<Point2> cones; // local variable to load cone observations into SLAM instance
    vector<Point2> orange_cones; // local variable to load cone observations into SLAM instance

    vector<Point2> blue_cones; //local variable to store the blue observed cones
    vector<Point2> yellow_cones; //local variable to store the yellow observed cones


    bool file_opened;

    rclcpp::TimerBase::SharedPtr timer;
    double dt;

    optional<std_msgs::msg::Header> prev_filter_time;

    //print files
    std::ofstream outfile;
    std::ofstream pose_cones;
};

int main(int argc, char * argv[]){
  std::ofstream out("squirrel.txt");
  std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
  std::cout.rdbuf(out.rdbuf());

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAMValidation>());
  rclcpp::shutdown();
  std::cout.rdbuf(coutbuf); //reset to standard output again

  return 0;
}