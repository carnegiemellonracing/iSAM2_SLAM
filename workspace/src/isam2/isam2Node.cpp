#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
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

static const long SLAM_DELAY_MICROSEC = 50000;
using namespace std;
using namespace std::placeholders;


struct VehicleState{
  double x;
  double y;
  double yaw;
  double dx;
  double dy;
  double dyaw;
};

class SLAMValidation : public rclcpp::Node
{
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

        cone_sub = this->create_subscription<interfaces::msg::ConeArray>(
           CONE_DATA_TOPIC, best_effort_qos, std::bind(&SLAMValidation::cone_callback, this, _1));

        //bring back///////////////
        //vehicle_pos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        //VEHICLE_POS_TOPIC, 10, std::bind(&SLAMValidation::vehicle_pos_callback, this, _1));

        vehicle_angle_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        VEHICLE_ANGLE_TOPIC, 10, std::bind(&SLAMValidation::vehicle_angle_callback, this, _1));

        vehicle_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        VEHICLE_VEL_TOPIC, 10, std::bind(&SLAMValidation::vehicle_vel_callback, this, _1));
        ////////////////////////

        dt = .1;
        orangeNotSeen = 25;
        orangeNotSeenFlag = false;
        loopClosure = false;

        init_odom = gtsam::Pose2(-1,-1,-1);
        file_opened = true;

        slam_first_run = false;
        prev_slam_time = high_resolution_clock::now();
        cur_slam_time = high_resolution_clock::now();

        velocity_msg_cache = {};
        angle_msg_cache = {};
    }

private:

    /* Whenever cone_callback is called, update car pose variables
     * by finding the ones that best match up to the time stamp of cone_data
     */
    void cone_callback(const interfaces::msg::ConeArray::SharedPtr cone_data)
    {
        RCLCPP_INFO(this->get_logger(), "\n \t cone_callback!");
        cones.clear();

        blue_cones.clear();
        yellow_cones.clear();
        orangeCones.clear();

        /* Check if you have enough elements to sync */
        if (velocity_msg_cache.size() == 0 || angle_msg_cache.size() == 0)
        {
            return;
        }

        /* Grabbing the velocity and orientation message closest to the
        * current cone message in time */
        double cur_time = ((double)cone_data->orig_data_stamp.sec +
                        ((double)((int)(cone_data->orig_data_stamp.nanosec / 1000)) / 1000000));

        /* Find the closest velocity message */
        geometry_msgs::msg::TwistStamped::SharedPtr closest_velocity;
        double best_velocity_time = 0;
        double min_diff = (double)INT_MAX;
        for (int i = 0; i < velocity_msg_cache.size(); i++)
        {
            geometry_msgs::msg::TwistStamped::SharedPtr cur_msg = velocity_msg_cache.front();
            velocity_msg_cache.pop_front();
            int nano_to_micro = (int)(cur_msg->header.stamp.nanosec / 1000);
            double micro_double = ((double)(nano_to_micro)) / 1000000;
            double msg_time = (double)cur_msg->header.stamp.sec + micro_double;


            if (abs(cur_time - msg_time) < min_diff)
            {
                closest_velocity = cur_msg;
                best_velocity_time = msg_time;
                min_diff = abs(cur_time - msg_time);
            }

            velocity_msg_cache.push_back(cur_msg);
        }

        /* Find the closest quaternion message */
        geometry_msgs::msg::QuaternionStamped::SharedPtr closest_angle;
        double best_angle_time = 0;
        min_diff = (double)INT_MAX;
        for (int i = 0; i < angle_msg_cache.size(); i++)
        {
            geometry_msgs::msg::QuaternionStamped::SharedPtr cur_msg = angle_msg_cache.front();
            angle_msg_cache.pop_front();
            int nano_to_micro = (int)(cur_msg->header.stamp.nanosec / 1000);
            double micro_double = ((double)(nano_to_micro)) / 1000000;
            double msg_time = (double)cur_msg->header.stamp.sec + micro_double;

            if (abs(cur_time - msg_time) < min_diff)
            {
                closest_angle = cur_msg;
                best_angle_time = msg_time;
                min_diff = abs(cur_time - msg_time);
            }

            angle_msg_cache.push_back(cur_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Finished finding syncing data");

        /* Calculate variables about car pose after closest candidates */
        /* Motion model for car pose */
        velocity = gtsam::Point2(closest_velocity->twist.linear.y,
                                 closest_velocity->twist.linear.x);

        double qw = closest_angle->quaternion.w;
        double qx = closest_angle->quaternion.x;
        double qy = closest_angle->quaternion.y;
        double qz = closest_angle->quaternion.z;


        // rotate by 90 degrees? TODO: add to velocity
        double yaw = atan2(2 * (qz * qw + qx * qy),
                             -1 + 2 * (qw * qw + qx * qx));

        RCLCPP_INFO(this->get_logger(), "Calculated Yaw");
        /* At the very beginning, we take the current velocity time stamp.
        * dt = 0 but that's ok because we shouldn't be moving when SLAM
        * starts
        */
        if (init_odom.x() == -1 && init_odom.y() == -1 && init_odom.theta() == -1)
        {
            init_odom = gtsam::Pose2(-1, -1, yaw);
            prev_velocity_time = best_velocity_time;
        }


        dt = abs(prev_velocity_time - best_velocity_time);
        prev_velocity_time = best_velocity_time;


        global_odom = gtsam::Pose2(-1, -1, yaw);

        /* Process cones */
        auto b_cones = cone_data->blue_cones;
        for (uint i = 0; i < b_cones.size(); i++)
        {
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(b_cones[i].x,
                                                                 b_cones[i].y));
            cones.push_back(to_add);
            blue_cones.push_back(to_add);
        }

        auto y_cones = cone_data->yellow_cones;
        for (uint i = 0; i < y_cones.size(); i++)
        {
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(y_cones[i].x,
                                                                 y_cones[i].y));
            cones.push_back(to_add);
            yellow_cones.push_back(to_add);
        }

        auto o_cones = cone_data->big_orange_cones;
        for (uint i = 0; i < o_cones.size(); i++)
        {
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(o_cones[i].x,
                                                                 o_cones[i].y));
            cones.push_back(to_add);
            orangeCones.push_back(to_add);
        }

        ///////////////////////////////////////////////////////
        // LOOP CLOSURE should be done in iSAM2 //
        ///////////////////////////////////////////////////////

        // check to see if you've seen orange cones again
        // orangeNotSeenFlag set to true when the orange cones not seen for 25 frames
        /*
        if (orangeCones.size() == 0)
        {
            orangeNotSeen++;
            if (orangeNotSeen >= 25)
            {
                // std::cout <<"orange not seen flag true" << std::endl;
                orangeNotSeenFlag = true;
            }
        }
        else
        {
            orangeNotSeen = 0;
        }


        if (orangeCones.size() == 2)
        { // what if there's more than 2?
          // std::cout <<"Added two cones to orange cones"<<endl;
            bool left = false;
            bool right = false;
            vector<Point2> orangeLoopReferenceCones(2);
            for (uint i = 0; i < orangeCones.size(); i++)
            {
                if (left == false && orangeCones[i].y() < 0)
                {
                    // add the left cone here
                    left = true;
                    orangeLoopReferenceCones[0] = orangeCones[i];
                }
                if (right == false && orangeCones[i].y() > 0)
                {
                    // add the right cone here
                    right = true;
                    orangeLoopReferenceCones[1] = orangeCones[i];
                }
            }
            orangeCones = orangeLoopReferenceCones;
        }

        if (orangeNotSeenFlag == true && orangeCones.size() >= 2)
        {
           // std::cout<<"found loop closure" << std::endl;
           Point2 diff = Point2(init_odom.x() - global_odom.x(),
                           init_odom.y() - global_odom.y());
           float dist_from_start = norm2(diff);

           bool near_start = (dist_from_start <= (float)10);

           float bearing_diff = abs(init_odom.theta() - global_odom.theta());
           bool facing_start = bearing_diff <= (float)(M_PI / 2);

           RCLCPP_INFO(this->get_logger(), "BEARING DIFF: %lf | DIST FROM START: %lf",
                                   bearing_diff, dist_from_start);
           if (near_start && facing_start)
           {
               RCLCPP_INFO(this->get_logger(), "LOOP CLOSURE\n\n");
               loopClosure = true;
           }
        }
        */
        cur_slam_time = high_resolution_clock::now();
        long dur = duration_cast<microseconds>(cur_slam_time - prev_slam_time).count();
        if (slam_first_run == false || dur > SLAM_DELAY_MICROSEC)
        {
            prev_slam_time = cur_slam_time;
            slam_first_run = true;
            run_slam();
        }
    }

    void vehicle_angle_callback(
        const geometry_msgs::msg::QuaternionStamped::SharedPtr vehicle_angle_data)
    {
        RCLCPP_INFO(this->get_logger(), "\n \t vehicle angle callback! | time: %d\n",
                  vehicle_angle_data->header.stamp.sec);

        angle_msg_cache.push_back(vehicle_angle_data);
        if (angle_msg_cache.size() > MSG_CACHE_SIZE)
        {
            angle_msg_cache.pop_front();
        }

    }

    void vehicle_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr vehicle_vel_data)
    {
        RCLCPP_INFO(this->get_logger(), "\n \t vehicle velocity callback! | time: %d\n",
                                                vehicle_vel_data->header.stamp.sec);
        velocity_msg_cache.push_back(vehicle_vel_data);
        if (velocity_msg_cache.size() > MSG_CACHE_SIZE)
        {
            velocity_msg_cache.pop_front();
        }

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
                  yellow_cones, orangeCones, velocity, dt, loopClosure);


    }


    slamISAM slam_instance = slamISAM(this->get_logger());
    rclcpp::Subscription<interfaces::msg::ConeArray>::SharedPtr cone_sub;
    std::chrono::time_point<std::chrono::high_resolution_clock> prev_slam_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> cur_slam_time;

    //Bring back/////////////////
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr vehicle_pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr vehicle_angle_sub;

    gtsam::Point2 velocity;
    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    gtsam::Pose2 prev_odom;
    vector<Point2> cones; // local variable to load cone observations into SLAM instance
    vector<Point2> orangeCones; // local variable to load cone observations into SLAM instance

    vector<Point2> blue_cones; //local variable to store the blue observed cones
    vector<Point2> yellow_cones; //local variable to store the yellow observed cones

    gtsam::Pose2 init_odom; // local variable to load odom into SLAM instance

    bool file_opened;

    rclcpp::TimerBase::SharedPtr timer;
    double dt;
    double prev_velocity_time;
    int orangeNotSeen;
    bool orangeNotSeenFlag;
    bool loopClosure;
    bool slam_first_run;

    deque<interfaces::msg::ConeArray::SharedPtr> cone_msg_cache;
    deque<geometry_msgs::msg::TwistStamped::SharedPtr> velocity_msg_cache;
    deque<geometry_msgs::msg::QuaternionStamped::SharedPtr> angle_msg_cache;


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
