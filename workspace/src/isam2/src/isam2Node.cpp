#include "isam2_pkg.hpp"
using namespace std;
using namespace gtsam;
using namespace std::chrono;
using std::size_t;
using namespace rclcpp;

typedef tuple<Pose2, Pose2, std_msgs::msg::Header> synced_gps_msg_t;

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

        cone_sub = this->create_subscription<interfaces::msg::ConeArray>(CONE_DATA_TOPIC, 10, 
                                                std::bind(&SLAMValidation::cone_callback, this, _1));
        
        vehicle_pos_sub.subscribe(this, VEHICLE_POS_TOPIC, best_effort_profile);
        vehicle_angle_sub.subscribe(this, VEHICLE_ANGLE_TOPIC, best_effort_profile);
        vehicle_vel_sub.subscribe(this, VEHICLE_VEL_TOPIC, best_effort_profile);

        gps_sync = std::make_shared<message_filters::Synchronizer<
                                    message_filters::sync_policies::ApproximateTime<
                                    geometry_msgs::msg::Vector3Stamped,
                                    geometry_msgs::msg::TwistStamped,
                                    geometry_msgs::msg::QuaternionStamped>>>(
                            message_filters::sync_policies::ApproximateTime<
                                    geometry_msgs::msg::Vector3Stamped,
                                    geometry_msgs::msg::TwistStamped,
                                    geometry_msgs::msg::QuaternionStamped>(30),
                                    vehicle_pos_sub, vehicle_vel_sub,vehicle_angle_sub);
        gps_sync->setAgePenalty(0.09);
        gps_sync->registerCallback(std::bind(&SLAMValidation::gps_sync_callback, this, _1, _2, _3));


        dt = .1;

        //TODO: std::optional where init is set to None
        init_lon_lat = std::nullopt;
        file_opened = true;

        prev_filter_time = std::nullopt;

        prev_sync_callback_time = std::nullopt;
        first_cone_gps_sync = false;
        older_synced_gps_msg = nullopt;
        newer_synced_gps_msg = nullopt;
    }

private:
    void calc_dt_update_prev(std_msgs::msg::Header cur_time_header) {
        optional<std_msgs::msg::Header> cur_filter_time(cur_time_header);
        if (!prev_filter_time.has_value()) {
            prev_filter_time.swap(cur_filter_time);
            return;
        }

        header_to_dt(prev_filter_time, cur_filter_time, dt);
        prev_filter_time.swap(cur_filter_time);
    }

    void interpolate_betw_old_new(Pose2 interp_res, Pose2 old_pose, Pose2 new_pose, double scale) {
        double dx = new_pose.x() - old_pose.x();
        double dy = new_pose.y() - old_pose.y();
        double dtheta = new_pose.theta() - old_pose.theta();

        interp_res = Pose2(old_pose.x() + dx * scale, 
                            old_pose.y() + dy * scale,
                            old_pose.theta() + dtheta * scale);
    }

    void find_closest_synced_gps_msg(std_msgs::msg::Header cone_data_header,
                                    optional<synced_gps_msg_t> older_synced_gps_msg,
                                    optional<synced_gps_msg_t> newer_synced_gps_msg,
                                    synced_gps_msg_t &closest_synced_gps_msg) {
        if (!older_synced_gps_msg.has_value() && !newer_synced_gps_msg.has_value()) {
            return;
        }

        double diff_w_older = 0.0;
        header_to_dt(get<2>(older_synced_gps_msg.value()), cone_data_header, diff_w_older);

        double diff_w_newer = 0.0;
        header_to_dt(cone_data_header, get<2>(newer_synced_gps_msg.value()), diff_w_newer);

        if (diff_w_newer < MAX_DELAY_BETW_MSGS || diff_w_older < MAX_DELAY_BETW_MSGS) {
            if (diff_w_newer < diff_w_older) {
                closest_synced_gps_msg = newer_synced_gps_msg.value();
            } else {
                closest_synced_gps_msg = older_synced_gps_msg.value();
            }
        } else { /* interpolate */
            /* Scale interpolation based on where the cone_msg is wrt the sandwich*/
            double older_newer_diff = 0.0;
            header_to_dt(get<2>(older_synced_gps_msg.value()), 
                        get<2>(newer_synced_gps_msg.value()), 
                        older_newer_diff);
            
            double scale = diff_w_older / older_newer_diff;

            Pose2 older_odom = get<0>(older_synced_gps_msg.value());
            Pose2 newer_odom = get<0>(newer_synced_gps_msg.value());
            Pose2 interp_odom = Pose2(0.0, 0.0, 0.0);
            interpolate_betw_old_new(interp_odom, older_odom, newer_odom, scale);

            Pose2 older_velocity = get<1>(older_synced_gps_msg.value());
            Pose2 newer_velocity = get<1>(newer_synced_gps_msg.value());
            Pose2 interp_velocity = Pose2(0.0, 0.0, 0.0);
            interpolate_betw_old_new(interp_velocity, older_velocity, newer_velocity, scale);
            
            closest_synced_gps_msg = make_tuple(interp_odom, interp_velocity, cone_data_header);
        }
    }

    /* @brief Pairs a cone message with the closest matching synced gps message,
     *          or if there is no closest message within the allowed threshold,
     *          then we interpolate the odom message
     */
    void sync_cones_with_gps(synced_gps_msg_t closest_synced_gps_msg, const interfaces::msg::ConeArray::ConstSharedPtr &cone_data) {
        
        RCLCPP_INFO(this->get_logger(), "\tsync_cones_with_gps: finding closest_synced_gps_msg");
        

        if (!newer_synced_gps_msg.has_value() && !older_synced_gps_msg.has_value()) {
            return;
        }

        /* Strategy is to sandwich the current lidar message between 2 synced gps messages*/

        /* Find the synced gps data that best matches the cone_data */
        double diff_w_newer = 0.0;
        header_to_dt(cone_data->header, get<2>(newer_synced_gps_msg.value()), diff_w_newer); 
        /* diff = newer_synced_gps_msg - cone_data_header */

        if (diff_w_newer < 0.0) { /* gps synced message is less recent/older than the cone data */

            while (!gps_synced_msg_queue.empty() && diff_w_newer < 0.0 ) {
                header_to_dt(cone_data->header, get<2>(newer_synced_gps_msg.value()), diff_w_newer);
                older_synced_gps_msg = newer_synced_gps_msg.value();
                newer_synced_gps_msg = gps_synced_msg_queue.front();
                gps_synced_msg_queue.pop();
            }

        }

        /* 
         * If newer gps synced message is younger than the current cone data, then 
         * the older gps synced message must be older than the current cone data.
         * 
         * This is based on the fact that the odler gps synced message is was older
         * than the previous cone data (by IH)
         */

        find_closest_synced_gps_msg(cone_data->header, older_synced_gps_msg,
                                    newer_synced_gps_msg, closest_synced_gps_msg);

 
        //NOTE THIS ONLY HAPPENS WHEN THERE IS A MATCH
        /* Calculate dt */
        bool prev_filter_time_initialized = prev_filter_time.has_value();
        calc_dt_update_prev(cone_data->header);
        if (!prev_filter_time_initialized) {
            /* Fetch for another message so that we can calculate a dt */
            return;
        }

        
    }

    void gps_sync_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data,
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

        Pose2 cur_odom = Pose2(0.0, 0.0, 0.0);
        Pose2 cur_velocity = Pose2(0.0, 0.0, 0.0);
        std_msgs::msg::Header cur_header = vehicle_pos_data->header;

        ///* Vehicle position callback */
        vehicle_pos_callback(cur_odom, vehicle_pos_data);

        ///* Vehicle velocity callback */
        vehicle_vel_callback(cur_velocity, vehicle_vel_data);

        ///* Vehicle angle callback */
        vehicle_angle_callback(cur_odom, vehicle_angle_data);       

        /* gps_synced_msg_queue contains elements representing inputs to slamISAM::step */
        gps_synced_msg_queue.emplace(cur_odom, cur_velocity, cur_header);

        /* Ensuring base case sandwich is created before syncing cones with synced gps*/
        if (!older_synced_gps_msg.has_value()) {
            older_synced_gps_msg = gps_synced_msg_queue.front();

        } else if (older_synced_gps_msg.has_value() && 
                    !newer_synced_gps_msg.has_value() && 
                    gps_synced_msg_queue.size() >= 2) {
            gps_synced_msg_queue.pop();
            newer_synced_gps_msg = gps_synced_msg_queue.front();
        }

        auto sync_data_end = high_resolution_clock::now();
        auto sync_data_duration = duration_cast<milliseconds>(sync_data_end - sync_data_start);
        RCLCPP_INFO(this->get_logger(), "\tSync callback time: %ld \n", sync_data_duration.count());

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
        RCLCPP_INFO(this->get_logger(), "processing cone_data into cones");
        cone_msg_to_vectors(cones, blue_cones, yellow_cones, orange_cones, cone_data);        
        RCLCPP_INFO(this->get_logger(), "finished processing cone_data into cones");

        /* Timers */
        auto cone_callback_end = high_resolution_clock::now();
        auto cone_callback_duration = duration_cast<milliseconds>(cone_callback_end - cone_callback_start);
        RCLCPP_INFO(this->get_logger(), "\tCone callback time: %ld", cone_callback_duration.count());

        /* Sync with gps data */
        if (older_synced_gps_msg.has_value() && newer_synced_gps_msg.has_value()) {
            synced_gps_msg_t closest_synced_gps_msg;
            sync_cones_with_gps(closest_synced_gps_msg, cone_data);
            slam_instance.step(get<0>(closest_synced_gps_msg), cones, blue_cones, yellow_cones, orange_cones,
                                get<1>(closest_synced_gps_msg), dt);
        }

    }

    void vehicle_pos_callback(Pose2 &global_odom, const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data)
    {
        // RCLCPP_INFO(this->get_logger(), "\t vehicle position callback! | time: %d\n",
        //                                         vehicle_pos_data->header.stamp.sec);
        auto vehicle_pos_callback_start = high_resolution_clock::now();
        
        vector3_msg_to_gps(global_odom, vehicle_pos_data, init_lon_lat, this->get_logger());

        /* Timers*/
        auto vehicle_pos_callback_end = high_resolution_clock::now();
        auto vehicle_pos_callback_duration = duration_cast<milliseconds>(vehicle_pos_callback_end - vehicle_pos_callback_start);
        RCLCPP_INFO(this->get_logger(), "\tPosition callback time: %ld", vehicle_pos_callback_duration.count());
    }



    void vehicle_vel_callback(Pose2 &velocity, const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data)
    {
        // RCLCPP_INFO(this->get_logger(), "\t vehicle velocity callback! | time: %d\n",
        //                                         vehicle_vel_data->header.stamp.sec);
        auto vehicle_vel_callback_start = high_resolution_clock::now();
        velocity_msg_to_pose2(velocity, vehicle_vel_data);

        /* Timers*/
        auto vehicle_vel_callback_end = high_resolution_clock::now();
        auto vehicle_vel_callback_duration = duration_cast<milliseconds>(vehicle_vel_callback_end - vehicle_vel_callback_start);
        RCLCPP_INFO(this->get_logger(), "\tVelocity callback time: %ld", vehicle_vel_callback_duration.count());
    }

    void vehicle_angle_callback(Pose2 &global_odom, const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data)
    {
        // RCLCPP_INFO(this->get_logger(), "\t vehicle angle callback! | time: %d\n",
        //                                       vehicle_angle_data->header.stamp.sec);
        auto vehicle_angle_callback_start = high_resolution_clock::now();
        quat_msg_to_yaw(global_odom, vehicle_angle_data, this->get_logger());

        /* Timers*/
        auto vehicle_angle_callback_end = high_resolution_clock::now();
        auto vehicle_angle_callback_duration = duration_cast<milliseconds>(vehicle_angle_callback_end - vehicle_angle_callback_start);
        RCLCPP_INFO(this->get_logger(), "\tAngle callback time: %ld", vehicle_angle_callback_duration.count());
    }

    slamISAM slam_instance = slamISAM(this->get_logger());
    high_resolution_clock::time_point cur_sync_callback_time;
    optional<high_resolution_clock::time_point> prev_sync_callback_time;
    rclcpp::Subscription<interfaces::msg::ConeArray>::SharedPtr cone_sub;
    message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> vehicle_pos_sub;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> vehicle_vel_sub;
    message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> vehicle_angle_sub;

    std::shared_ptr<message_filters::Synchronizer<
                            message_filters::sync_policies::ApproximateTime<
                                            geometry_msgs::msg::Vector3Stamped,
                                            geometry_msgs::msg::TwistStamped,
                                            geometry_msgs::msg::QuaternionStamped>>> gps_sync;
    
    /* This is the queue that will be holding the most recent gps_synced messages */
    queue<synced_gps_msg_t> gps_synced_msg_queue;

    /* Tracks whether a cone message and synced gps message has been synced together yet 
     * Necessary for base case */
    bool first_cone_gps_sync;

    optional<synced_gps_msg_t> older_synced_gps_msg;
    optional<synced_gps_msg_t> newer_synced_gps_msg;

    optional<gtsam::Point2> init_lon_lat; // local variable to load odom into SLAM instance
    std::vector<Point2> cones; // local variable to load cone observations into SLAM instance
    std::vector<Point2> orange_cones; // local variable to load cone observations into SLAM instance

    std::vector<Point2> blue_cones; //local variable to store the blue observed cones
    std::vector<Point2> yellow_cones; //local variable to store the yellow observed cones


    bool file_opened;

    rclcpp::TimerBase::SharedPtr timer;
    double dt;

    optional<std_msgs::msg::Header> prev_filter_time;

    //print files
    std::ofstream outfile;
    std::ofstream pose_cones;
};

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAMValidation>());
  rclcpp::shutdown();

  return 0;
}
