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

        //cone_sub = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
        //    CONE_DATA_TOPIC, best_effort_qos, std::bind(&SLAMValidation::cone_callback, this, _1));
        cone_sub = this->create_subscription<interfaces::msg::ConeArray>(
           CONE_DATA_TOPIC, best_effort_qos, std::bind(&SLAMValidation::cone_callback, this, _1));


        //vehicle_state_sub = this->create_subscription<eufs_msgs::msg::CarState>(
        //VEHICLE_DATA_TOPIC, 10, std::bind(&SLAMValidation::vehicle_state_callback, this, _1));

        //bring back///////////////
        //vehicle_pos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        //VEHICLE_POS_TOPIC, 10, std::bind(&SLAMValidation::vehicle_pos_callback, this, _1));

        vehicle_angle_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        VEHICLE_ANGLE_TOPIC, 10, std::bind(&SLAMValidation::vehicle_angle_callback, this, _1));

        vehicle_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        VEHICLE_VEL_TOPIC, 10, std::bind(&SLAMValidation::vehicle_vel_callback, this, _1));
        ////////////////////////

        /*
        vehicle_data_sub = this->create_subscription<interfaces::msg::ConeArrayWithOdom>(
                              "/perc_cones", 10, std::bind(&SLAMValidation::vehicle_data_callback,
                                  this, _1));
                                  */
        timer = this->create_wall_timer(100ms, std::bind(&SLAMValidation::timer_callback, this));
        //TODO: need to initalize robot state?????
        dt = .1;
        orangeNotSeen = 25;
        orangeNotSeenFlag = false;
        loopClosure = false;

        init_odom = gtsam::Pose2(-1,-1,-1);
        file_opened = true;
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
	
        /* Grabbing the velocity and orientation message closest to the 
        * current cone message in time */
        double cur_time = ((double)cone_data->orig_data_stamp.sec +
                        ((double)((int)(cone_data->orig_data_stamp.nanosec / 1000)) / 1000000));

        /* Find the closest velocity message */
        geometry_msgs::msg::TwistStamped::SharedPtr closest_velocity;
        double best_velocity_time = 0;
        double min_diff = (double)INT_MAX;
        for (int i = 0; i < 6; i++)
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
        for (int i = 0; i < 6; i++)
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

        /* Calculate variables about car pose after closest candidates */
        /* Motion model for car pose */
        veh_state.dx = closest_velocity->twist.linear.x;
        veh_state.dy = closest_velocity->twist.linear.y;
        veh_state.dyaw = closest_velocity->twist.angular.z;
        velocity = gtsam::Point2(veh_state.dx, veh_state.dy, veh_state.dyaw);

        /* At the very beginning, we take the current velocity time stamp. 
        * dt = 0 but that's ok because we shouldn't be moving when SLAM 
        * starts
        */
        if (init_odom.x() == -1 && init_odom.y() == -1 && init_odom.theta() == -1)
        {
            prev_velocity_time = best_velocity_time;
        }


        dt = abs(prev_velocity_time - best_velocity_time);
        prev_velocity_time = best_velocity_time;
        

        double q0 = closest_angle->quaternion.w;
        double q1 = closest_angle->quaternion.x;
        double q2 = closest_angle->quaternion.y;
        double q3 = closest_angle->quaternion.z;

        //veh_state.yaw = atan2(2 * (q0 * q3 + q1 * q2),
        //        pow(q0, 2) + pow(q1, 2) - pow(q2, 2) - pow(q3, 2));

        // rotate by 90 degrees? TODO: add to velocity
        veh_state.yaw = atan2(2 * (q3 * q0 + q1 * q2),
                             -1 + 2 * (q0 * q0 + q1 * q1));

        
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
        // LOOP CLOSURE //
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
           // TODO: does not account for when there is only a single frame that it sees orange cones
        }
        */
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
        //if (global_odom.x() == 0 || global_odom.y() == 0 || global_odom.theta() == 0 || init_odom.theta() == -1)
        //{
        //  RCLCPP_INFO(this->get_logger(), "fucked pose: (%f,%f,%f)", global_odom.x(), global_odom.y(), global_odom.theta());
        //  return;
        //}
        RCLCPP_INFO(this->get_logger(), "Car Pose: (%f, %f, %f)", global_odom.x(),
                                                                  global_odom.y(),
                                                                  global_odom.theta());
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


        /* Why are we resetting the global pose? 
         * We may not get GPS at every time step.
         * Instead, use motion model to predict new pose.
         */
         //global_odom = gtsam::Pose2(-1, -1, -1);
    }

    void timer_callback()
    {
        global_odom = gtsam::Pose2(veh_state.x, veh_state.y, veh_state.yaw);
        if (init_odom.x() == -1 && init_odom.y() == -1 && init_odom.theta() == -1)
        {
            init_odom = global_odom;
        }

        velocity = gtsam::Point2(veh_state.dx, veh_state.dy);


        /* Determining which dyaw indicates turning */
        // if (abs(veh_state.dyaw) < TURNING_CONSTANT)
        // {
        //   run_slam();
        // }
        run_slam();




        prev_veh_state = veh_state;
    }

    slamISAM slam_instance = slamISAM(this->get_logger());
    rclcpp::Subscription<interfaces::msg::ConeArray>::SharedPtr cone_sub;
    //rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_sub;
    //rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr vehicle_state_sub;


    //Bring back/////////////////
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr vehicle_pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr vehicle_angle_sub;
    ///////////////////////////////
    //rclcpp::Subscription<interfaces::msg::ConeArrayWithOdom>::SharedPtr vehicle_data_sub;

    //gtsam::Pose2 velocity;  // local variable to load velocity into SLAM instance
    gtsam::Point2 velocity;
    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    gtsam::Pose2 prev_odom;
    vector<Point2> cones; // local variable to load cone observations into SLAM instance
    vector<Point2> orangeCones; // local variable to load cone observations into SLAM instance

    vector<Point2> blue_cones; //local variable to store the blue observed cones
    vector<Point2> yellow_cones; //local variable to store the yellow observed cones

    gtsam::Pose2 init_odom; // local variable to load odom into SLAM instance
    VehicleState prev_veh_state = VehicleState();
    VehicleState veh_state = VehicleState();

    bool file_opened;

    rclcpp::TimerBase::SharedPtr timer;
    double dt;
    //double time_ns;
    double prev_velocity_time;
    int orangeNotSeen;
    bool orangeNotSeenFlag;
    bool loopClosure;

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
