#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "interfaces/msg/cone_array.hpp"
#include "interfaces/msg/cone_array_with_odom.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <gtsam/nonlinear/ISAM2Params.h>

#include "isam2.cpp"



#include <boost/shared_ptr.hpp>
#include <vector>
#include <deque>
#include <cmath>
#include <chrono>

//#define CONE_DATA_TOPIC "/ground_truth/cones"
//#define VEHICLE_DATA_TOPIC "/ground_truth/state"
//#define VEHICLE_VEL_TOPIC "/filter/velocity"
//#define VEHICLE_ANGLE_TOPIC "/filter/quaternion"


#define CONE_DATA_TOPIC "/perc_cones"
#define VEHICLE_POS_TOPIC "/filter/positionlla"
#define VEHICLE_ANGLE_TOPIC "/filter/quaternion"
#define VEHICLE_VEL_TOPIC "/filter/twist"

// static const long SLAM_DELAY_MICROSEC = 50000;

using namespace std;
using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

class SLAMValidation : public rclcpp::Node {
public:
    // builtin_interfaces::Time pos_time_stamp;
    // builtin_interfaces::Time obs_time_stamp;
    SLAMValidation(): Node("slam_validation")
    {
        const rmw_qos_profile_t best_effort_profile = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            20,
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

        vehicle_pos_sub.subscribe(this, VEHICLE_POS_TOPIC, best_effort_profile);
        vehicle_angle_sub.subscribe(this, VEHICLE_ANGLE_TOPIC, best_effort_profile);
        vehicle_vel_sub.subscribe(this, VEHICLE_VEL_TOPIC, best_effort_profile);

        cone_sub = std::make_shared<message_filters::Subscriber<interfaces::msg::ConeArray>>(this, CONE_DATA_TOPIC, best_effort_profile);
        cone_sub->registerCallback(std::bind(&SLAMValidation::cone_callback, this, _1));
        gps_sync = std::make_shared<message_filters::Synchronizer<
                                    message_filters::sync_policies::ApproximateTime<
                                    geometry_msgs::msg::Vector3Stamped,
                                    geometry_msgs::msg::TwistStamped,
                                    geometry_msgs::msg::QuaternionStamped>>>(
                            message_filters::sync_policies::ApproximateTime<
                                    geometry_msgs::msg::Vector3Stamped,
                                    geometry_msgs::msg::TwistStamped,
                                    geometry_msgs::msg::QuaternionStamped>(20),
                                    vehicle_pos_sub, vehicle_vel_sub,vehicle_angle_sub);
        gps_sync->setAgePenalty(0.09);
        gps_sync->registerCallback(std::bind(&SLAMValidation::sync_callback, this, _1, _2, _3));
        gps_queue = std::make_shared<std::deque<std::tuple<double, gtsam::Pose2, gtsam::Pose2, double>>>();

        dt = .1;
        gps_queue_mutex = new std::mutex();

        //TODO: std::optional where init is set to None
        init_lon_lat = std::nullopt;
        file_opened = true;

        prev_filter_time = std::nullopt;
    }

private:

    void sync_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data,
                    const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data,
                    const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data) {
        RCLCPP_INFO(this->get_logger(), "Sync Callback");

        optional<std_msgs::msg::Header> cur_filter_time(vehicle_pos_data->header);
        if (!prev_filter_time.has_value()) {
            prev_filter_time.swap(cur_filter_time);
            return;
        }

        header_to_dt(prev_filter_time, cur_filter_time, dt);
        prev_filter_time.swap(cur_filter_time);

        //cone_callback(cone_data);
        vehicle_pos_callback(vehicle_pos_data);
        vehicle_vel_callback(vehicle_vel_data);
        vehicle_angle_callback(vehicle_angle_data);
        gps_queue_mutex->lock();
        gps_queue->emplace_back(header_to_nanosec(cur_filter_time.value()), global_odom, velocity, dt);
        if(gps_queue->size() > MAX_QUEUE) {
            gps_queue->pop_front();
        }
        gps_queue_mutex->unlock();
        //run_slam();

        
    }


    /* Whenever cone_callback is called, update car pose variables
     * by finding the ones that best match up to the time stamp of cone_data
     */
    void cone_callback(const interfaces::msg::ConeArray::ConstSharedPtr &cone_data)
    {
        RCLCPP_INFO(this->get_logger(), "\t cone_callback!");

        optional<std_msgs::msg::Header> cur_filter_time(cone_data->header);
        double cur_time = header_to_nanosec(cur_filter_time.value());

        cones = {};

        /* issue these could clear before you're finished with data association. */
        blue_cones = {};
        yellow_cones = {};
        orange_cones = {};

        /* Process cones */
        cone_msg_to_vectors(cone_data, cones, blue_cones, yellow_cones, orange_cones);

        std::tuple<double, gtsam::Pose2, gtsam::Pose2, double> bestMsg; 
        double minTimeDiff = std::numeric_limits<double>::max();

        gps_queue_mutex->lock();
        for (const auto &element : *gps_queue) // Dereference the shared pointer
        {
            double time = std::get<0>(element);
            if(std::abs(time - cur_time) < minTimeDiff) {
                minTimeDiff = std::abs(time - cur_time);
                bestMsg = element;
            }
            if(time > cur_time) {
                break;
            }
        }
        global_odom = std::get<1>(bestMsg);
        velocity = std::get<2>(bestMsg);
        dt = std::get<3>(bestMsg);
        gps_queue_mutex->unlock();
        run_slam();
    }

    void vehicle_pos_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &vehicle_pos_data)
    {
        RCLCPP_INFO(this->get_logger(), "\t vehicle position callback! | time: %d\n",
                                                vehicle_pos_data->header.stamp.sec);
        vector3_msg_to_gps(vehicle_pos_data, global_odom, init_lon_lat, this->get_logger());
    }



    void vehicle_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data)
    {
        RCLCPP_INFO(this->get_logger(), "\t vehicle velocity callback! | time: %d\n",
                                                vehicle_vel_data->header.stamp.sec);
        velocity_msg_to_pose2(vehicle_vel_data, velocity);
    }

    void vehicle_angle_callback(
        const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data)
    {
        RCLCPP_INFO(this->get_logger(), "\t vehicle angle callback! | time: %d\n",
                  vehicle_angle_data->header.stamp.sec);
        double yaw = 0;
        quat_msg_to_yaw(vehicle_angle_data, yaw, global_odom, this->get_logger());
        //RCLCPP_INFO(this->get_logger(), "yaw: %.10f", yaw);
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
        slam_instance.step(this->get_logger(), global_odom, cones, blue_cones,
                  yellow_cones, orange_cones, velocity, dt);


    }


    slamISAM slam_instance = slamISAM(this->get_logger());

    message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> vehicle_pos_sub;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> vehicle_vel_sub;
    message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> vehicle_angle_sub;


    std::mutex *gps_queue_mutex;
    std::shared_ptr<std::deque<std::tuple<double, gtsam::Pose2, gtsam::Pose2, double>>> gps_queue;
    std::shared_ptr<message_filters::Subscriber<interfaces::msg::ConeArray>> cone_sub;
    std::shared_ptr<message_filters::Synchronizer<
                            message_filters::sync_policies::ApproximateTime<
                                            geometry_msgs::msg::Vector3Stamped,
                                            geometry_msgs::msg::TwistStamped,
                                            geometry_msgs::msg::QuaternionStamped>>> gps_sync;
    
    gtsam::Pose2 velocity;
    int MAX_QUEUE = 10;

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

  auto shared_ptr = std::make_shared<SLAMValidation>();
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(shared_ptr);
  rclcpp::spin(shared_ptr);
  executor.spin();
  std::cout.rdbuf(coutbuf); //reset to standard output again

  return 0;
}
