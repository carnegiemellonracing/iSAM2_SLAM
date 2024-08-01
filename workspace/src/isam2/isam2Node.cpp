#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "interfaces/msg/cone_array.hpp"

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
#include <cmath>
#include <chrono>

// #define CONE_DATA_TOPIC "/ground_truth/cones"
// #define VEHICLE_DATA_TOPIC "/ground_truth/state"


#define CONE_DATA_TOPIC "/perc_cones"
#define VEHICLE_POS_TOPIC "/gnss"
#define VEHICLE_ANGLE_TOPIC "/filter/quaternion"
#define VEHICLE_VEL_TOPIC "/filter/twist"
// #define VECHICLE_VEL_TOPIC ""

//test on hybrid-3 fourth run
// #define TURNING_CONSTANT_LOW 0.10
// #define TURNING_CONSTANT_HI 0.30
// #define TURNING_CONSTANT 0.1

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
    SLAMValidation(): Node("slam_validation"){
      const rmw_qos_profile_t best_effort_profile = {
          RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          10,
          RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
          RMW_QOS_POLICY_DURABILITY_VOLATILE,
          RMW_QOS_DEADLINE_DEFAULT,
          RMW_QOS_LIFESPAN_DEFAULT,
          RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
          RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
          false};

      const rclcpp::QoS best_effort_qos = rclcpp::QoS(
          rclcpp::QoSInitialization(
              best_effort_profile.history,
              best_effort_profile.depth),
          best_effort_profile);

      cone_sub = this->create_subscription<interfaces::msg::ConeArray>(
          CONE_DATA_TOPIC, best_effort_qos, std::bind(&SLAMValidation::cone_callback, this, _1));
      

      // For sim
      // vehicle_state_sub = this->create_subscription<interfaces::msg::CarState>(
      // VEHICLE_DATA_TOPIC, 10, std::bind(&SLAMValidation::vehicle_state_callback, this, _1));

      //bring back///////////////
      vehicle_pos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      VEHICLE_POS_TOPIC, 10, std::bind(&SLAMValidation::vehicle_pos_callback, this, _1));

      vehicle_angle_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      VEHICLE_ANGLE_TOPIC, 10, std::bind(&SLAMValidation::vehicle_angle_callback, this, _1));

      vehicle_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      VEHICLE_VEL_TOPIC, 10, std::bind(&SLAMValidation::vehicle_vel_callback, this, _1));
      ////////////////////////

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

    // positive y forward
    // positive x right
    void cone_callback(const interfaces::msg::ConeArray::SharedPtr cone_data)
    {
      RCLCPP_INFO(this->get_logger(), "\n \t cone_callback! | time: %d\n", 
              cone_data->orig_data_stamp.sec);
      // obs_time_stamp = cone_data->orig_data_stamp;
      cones.clear();

      blue_cones.clear();
      yellow_cones.clear();
      orangeCones.clear();


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
      
      // check to see if you've seen orange cones again
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
        loopClosure = true; // TODO: does not account for when there is only a single frame that it sees orange cones
      }
      



    }


    void vehicle_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr vehicle_pos_data)
    {
      RCLCPP_INFO(this->get_logger(), "vehicle_pos_callback!");

      double LAT_TO_M = 111320;
      double LON_TO_M = 40075000;
      double x_meters;
      double y_meters;
      // TODO: filter

      // True measurement conversion for gps
      //  x_meters = LAT_TO_M*vehicle_pos_data->longitude;
      //  y_meters = LON_TO_M*cos(vehicle_pos_data->latitude)/360;

      x_meters = -(LAT_TO_M * vehicle_pos_data->latitude);
      y_meters = LAT_TO_M * vehicle_pos_data->longitude;

      if (init_odom.x() == -1 && init_odom.y() == -1)
      {
        init_odom = gtsam::Pose2(x_meters, y_meters, init_odom.theta());
      }

      veh_state.x = x_meters - init_odom.x();
      veh_state.y = y_meters - init_odom.y();
    }

    void vehicle_angle_callback(
        const geometry_msgs::msg::QuaternionStamped::SharedPtr vehicle_angle_data)
    {
      RCLCPP_INFO(this->get_logger(), "\n \t vehicle angle callback! | time: %d\n", 
                vehicle_angle_data->header.stamp.sec);

      double q0 = vehicle_angle_data->quaternion.w;
      double q1 = vehicle_angle_data->quaternion.x;
      double q2 = vehicle_angle_data->quaternion.y;
      double q3 = vehicle_angle_data->quaternion.z;
      // double yaw = atan2(2 * (q0 * q3 + q1 * q2),
      //             pow(q0, 2) + pow(q1, 2) - pow(q2, 2) - pow(q3, 2));
      double yaw = atan2(2 * (q3 * q0 + q1 * q2), -1 + 2 * (q0 * q0 + q1 * q1)); // rotate by 90 degrees? TODO: add to velocity

      if (init_odom.theta() == -1)
      {
        init_odom = gtsam::Pose2(init_odom.x(), init_odom.y(), yaw);
      }
      RCLCPP_INFO(this->get_logger(), "global_odom.theta() = %lf", yaw);
      veh_state.yaw = yaw;
    }

    void vehicle_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr vehicle_vel_data)
    {
      RCLCPP_INFO(this->get_logger(), "\n \t vehicle velocity callback!\n");
      
      veh_state.dx = vehicle_vel_data->twist.linear.x;
      veh_state.dy = vehicle_vel_data->twist.linear.y;
      veh_state.dyaw = vehicle_vel_data->twist.angular.z;
    }

    // void vehicle_state_callback(const sensor_msgs::msg::NavSatFix::SharedPtr pose_data)
    // {
    //     double pos_x = pose_data->pose.pose.position.x;
    //     double pos_y = pose_data->pose.pose.position.y;

    //     double q0 = pose_data->pose.pose.orientation.w;
    //     double q1 = pose_data->pose.pose.orientation.x;
    //     double q2 = pose_data->pose.pose.orientation.y;
    //     double q3 = pose_data->pose.pose.orientation.z;

    //     double yaw = atan2(2 * (q0 * q3 + q1 * q2),
    //             pow(q0, 2) + pow(q1, 2) - pow(q2, 2) - pow(q3, 2));


    //     global_odom = Pose2(pos_x, pos_y, yaw);
    //     double dx = pose_data->twist.twist.linear.x;
    //     double dy = pose_data->twist.twist.linear.y;
    //     velocity = gtsam::Point2(dx, dy);

    //     long cur_time = pose_data->header.stamp.sec * SEC_TO_NANOSEC + pose_data->header.stamp.nanosec;
    //     dt = time_ns - cur_time;
    //     time_ns = cur_time;


    // }




    void run_slam(){
      if (global_odom.x() == 0 || global_odom.y() == 0 || global_odom.theta() == 0 || init_odom.theta() == -1)
      {
        RCLCPP_INFO(this->get_logger(), "fucked pose: (%f,%f,%f)", global_odom.x(), global_odom.y(), global_odom.theta());
        return;
      }

      // print pose and cones
      std::ofstream ofs;
      std::ofstream out("urmom.txt");
      std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
      std::cout.rdbuf(out.rdbuf());

      if (!file_opened)
      {

          //RCLCPP_INFO(this->get_logger(), "running SLAM; new\n");
          ofs.open("urmom.txt", std::ofstream::out | std::ofstream::trunc);
          file_opened = true;
      } else if (file_opened) {

          //RCLCPP_INFO(this->get_logger(), "running SLAM\n");
          ofs.open("urmom.txt", std::ofstream::out | std::ofstream::app);
      }

      ofs << "(" << global_odom.x() << ","<< global_odom.y() << ","<< global_odom.theta()<< ")\n";
      for(auto cone: cones){
          //CHANGED BY MELINDA: yeah so cones look like they're flipped about the car y axis for some reason ?
          //RCLCPP_INFO(this->get_logger(), "reading cones\n");
          double range = norm2(cone);

          double bearing = std::atan2(cone.y(), cone.x());//+ global_odom.theta();
          double global_cone_x = global_odom.x() + range*cos(bearing+global_odom.theta());
          double global_cone_y = global_odom.y() + range*sin(bearing+global_odom.theta());
          ofs << "(" << global_cone_x << ","<< global_cone_y << ")\n"; //original
      }
      ofs.close();
      std::cout.rdbuf(coutbuf); //reset to standard output again
      
      slam_instance.step(this->get_logger(), global_odom, cones, blue_cones,
                  yellow_cones,orangeCones, velocity, dt, loopClosure);
      // RCLCPP_INFO(this->get_logger(), "NUM_LANDMARKS: %i\n", (slam_instance.n_landmarks));
    }

    void timer_callback()
    {
        global_odom = gtsam::Pose2(veh_state.x, veh_state.y, veh_state.yaw);
        velocity = gtsam::Pose2(veh_state.dx, veh_state.dy, veh_state.dyaw);


        /* Determining which dyaw indicates turning */
        // if (abs(veh_state.dyaw) < TURNING_CONSTANT)
        // {
        //   run_slam();
        // } 
        run_slam();
        
       
        prev_veh_state = veh_state;
    }

    slamISAM slam_instance = slamISAM(this->get_logger());
    // rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_sub;
    rclcpp::Subscription<interfaces::msg::ConeArray>::SharedPtr cone_sub;
    // rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr vehicle_state_sub;


    //Bring back/////////////////
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr vehicle_pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr vehicle_angle_sub;
    ///////////////////////////////

    gtsam::Pose2 velocity;  // local variable to load velocity into SLAM instance
    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    vector<Point2> cones; // local variable to load cone observations into SLAM instance
    vector<Point2> orangeCones; // local variable to load cone observations into SLAM instance

    vector<Point2> blue_cones; //local variable to store the blue observed cones
    vector<Point2> yellow_cones; //local variable to store the yellow observed cones

    gtsam::Pose2 init_odom; // local variable to load odom into SLAM instance
    VehicleState prev_veh_state = VehicleState();
    VehicleState veh_state = VehicleState();

    bool file_opened;

    rclcpp::TimerBase::SharedPtr timer;
    long dt;
    long time_ns;
    int orangeNotSeen;
    bool orangeNotSeenFlag;
    bool loopClosure;

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
