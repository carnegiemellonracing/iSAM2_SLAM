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
#define VEHICLE_POS_TOPIC "/gnss"
#define VEHICLE_ANGLE_TOPIC "/filter/quaternion"
#define VEHICLE_VEL_TOPIC "/filter/twist"
#define MSG_CACHE_SIZE 6
// #define VECHICLE_VEL_TOPIC ""

//test on hybrid-3 fourth run
// #define TURNING_CONSTANT_LOW 0.10
// #define TURNING_CONSTANT_HI 0.30
static const long QUEUE_SIZE = 20;
static const long SLAM_DELAY_MICROSEC = 50000;
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class SLAMValidation : public rclcpp::Node {
public:
    // builtin_interfaces::Time pos_time_stamp;
    // builtin_interfaces::Time obs_time_stamp;
    SLAMValidation(): Node("slam_validation")
    {
        const rmw_qos_profile_t best_effort_profile = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            10,
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

        //cone_sub = this->create_subscription<interfaces::msg::ConeArray>(
        //CONE_DATA_TOPIC,best_effort_qos,std::bind(&SLAMValidation::cone_callback,this,_1));

        //bring back///////////////
        //vehicle_pos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        //VEHICLE_POS_TOPIC, 10, std::bind(&SLAMValidation::vehicle_pos_callback, this, _1));

        //vehicle_angle_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        //VEHICLE_ANGLE_TOPIC, 10, std::bind(&SLAMValidation::vehicle_angle_callback, this, _1));

        //vehicle_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        //VEHICLE_VEL_TOPIC, 10, std::bind(&SLAMValidation::vehicle_vel_callback, this, _1));
        ////////////////////////
        cone_sub.subscribe(this, CONE_DATA_TOPIC, best_effort_profile);
        vehicle_angle_sub.subscribe(this, VEHICLE_ANGLE_TOPIC, best_effort_profile);
        vehicle_vel_sub.subscribe(this, VEHICLE_VEL_TOPIC, best_effort_profile);

        sync = std::make_shared<message_filters::Synchronizer<
                                    message_filters::sync_policies::ApproximateTime<
                                    interfaces::msg::ConeArray,
                                    geometry_msgs::msg::TwistStamped,
                                    geometry_msgs::msg::QuaternionStamped>>>(
                            message_filters::sync_policies::ApproximateTime<
                                    interfaces::msg::ConeArray,
                                    geometry_msgs::msg::TwistStamped,
                                    geometry_msgs::msg::QuaternionStamped>(10),
                                    cone_sub, vehicle_vel_sub,vehicle_angle_sub);
        sync->setAgePenalty(0.50);
        sync->registerCallback(std::bind(&SLAMValidation::sync_callback, this, _1, _2, _3));

        dt = .1;

        init_odom = gtsam::Pose2(-1,-1,-1);
        file_opened = true;

        prev_slam_time = high_resolution_clock::now();
        cur_slam_time = high_resolution_clock::now();

    }

private:

    void sync_callback(const interfaces::msg::ConeArray::ConstSharedPtr &cone_data,
                    const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data,
                    const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data) {
        RCLCPP_INFO(this->get_logger(), "Sync Callback");
        vehicle_vel_callback(vehicle_vel_data);
        vehicle_angle_callback(vehicle_angle_data);
        cone_callback(cone_data);

    }

    /* Whenever cone_callback is called, update car pose variables
     * by finding the ones that best match up to the time stamp of cone_data
     */
    void cone_callback(const interfaces::msg::ConeArray::ConstSharedPtr &cone_data)
    {
        RCLCPP_INFO(this->get_logger(), "\n \t cone_callback!");
        cones = {};

        /* issue these could clear before you're finished with data association. */
        blue_cones = {};
        yellow_cones = {};
        orange_cones = {};

        /* Process cones */
        cone_msg_to_vectors(cone_data, cones, blue_cones, yellow_cones, orange_cones);
        run_slam();
    }


    void vehicle_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &vehicle_vel_data)
    {
        RCLCPP_INFO(this->get_logger(), "\n \t vehicle velocity callback! | time: %d\n",
                                                vehicle_vel_data->header.stamp.sec);
        velocity_msg_to_point2(vehicle_vel_data, velocity);
    }

    void vehicle_angle_callback(
        const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr &vehicle_angle_data)
    {
        RCLCPP_INFO(this->get_logger(), "\n \t vehicle angle callback! | time: %d\n",
                  vehicle_angle_data->header.stamp.sec);
        double yaw = 0;
        quat_msg_to_yaw(vehicle_angle_data, yaw);

       /** At the very beginning, we take the current velocity time stamp.
        * dt = 0 but that's ok because we shouldn't be moving when SLAM
        * starts
        */
        if (init_odom.x() == -1 && init_odom.y() == -1 && init_odom.theta() == -1)
        {
            init_odom = gtsam::Pose2(-1, -1, yaw);
        }


        global_odom = Pose2(-1, -1, yaw - init_odom.theta());

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

    //Bring back/////////////////
    //rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr vehicle_pos_sub;
    //rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_vel_sub;
    //rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr vehicle_angle_sub;
    message_filters::Subscriber<interfaces::msg::ConeArray> cone_sub;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> vehicle_vel_sub;
    message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> vehicle_angle_sub;

    std::shared_ptr<message_filters::Synchronizer<
                            message_filters::sync_policies::ApproximateTime<
                                            interfaces::msg::ConeArray,
                                            geometry_msgs::msg::TwistStamped,
                                            geometry_msgs::msg::QuaternionStamped>>> sync;

    gtsam::Point2 velocity;
    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    gtsam::Pose2 prev_odom;
    vector<Point2> cones; // local variable to load cone observations into SLAM instance
    vector<Point2> orange_cones; // local variable to load cone observations into SLAM instance

    vector<Point2> blue_cones; //local variable to store the blue observed cones
    vector<Point2> yellow_cones; //local variable to store the yellow observed cones

    gtsam::Pose2 init_odom; // local variable to load odom into SLAM instance

    bool file_opened;

    rclcpp::TimerBase::SharedPtr timer;
    double dt;

    std::chrono::time_point<std::chrono::high_resolution_clock> prev_slam_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> cur_slam_time;

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
