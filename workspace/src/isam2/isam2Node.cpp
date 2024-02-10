#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "eufs_msgs/msg/cone_array.hpp"
#include "eufs_msgs/msg/car_state.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


#include "sensor_msgs/msg/nav_sat_fix.hpp"


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
#define VEHICLE_VEL_TOPIC "/filter/velocity"
// #define VECHICLE_VEL_TOPIC ""

using namespace std;
using std::placeholders::_1;


struct VehiclePosition{
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
    SLAMValidation(): Node("slam_validation"){
      // cone_sub = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
      rmw_qos_profile_t best_effort_qos = rmw_qos_profile_default;
      cone_sub = this->create_subscription<eufs_msgs::msg::ConeArray>(
      CONE_DATA_TOPIC, 10, std::bind(&SLAMValidation::cone_callback, this, _1));

      // vehicle_state_sub = this->create_subscription<eufs_msgs::msg::CarState>(
      // VEHICLE_DATA_TOPIC, 10, std::bind(&SLAMValidation::vehicle_state_callback, this, _1));

      vehicle_pos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      VEHICLE_POS_TOPIC, 10, std::bind(&SLAMValidation::vehicle_pos_callback, this, _1));

      vehicle_angle_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      VEHICLE_ANGLE_TOPIC, 10, std::bind(&SLAMValidation::vehicle_angle_callback, this, _1));

      vehicle_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      VEHICLE_VEL_TOPIC, 10, std::bind(&SLAMValidation::vehicle_vel_callback, this, _1));

      //TODO: need to initalize robot state?????
    
      timer = this->create_wall_timer(100ms, std::bind(&SLAMValidation::timer_callback, this));
      time_ns = 0; //Initialize time and dt
      dt = .1;
      orangeNotSeen = 25;
      orangeNotSeenFlag = false;
      loopClosure = false;


      init_odom = gtsam::Pose2(-1,-1,-1);
    }
  private:
    // void cone_callback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr cone_data){
    void cone_callback(const eufs_msgs::msg::ConeArray::SharedPtr cone_data){
        RCLCPP_INFO(this->get_logger(), "CONECALLBACK: B: %i| Y: %i| O: %i", cone_data->blue_cones.size(), cone_data->yellow_cones.size(), cone_data->orange_cones.size());
        cones.clear();
        orangeCones.clear();
        for(uint i = 0; i < cone_data->blue_cones.size(); i++){
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->blue_cones[i].x, cone_data->blue_cones[i].y));
            cones.push_back(to_add);
        }
        for(uint i = 0; i < cone_data->yellow_cones.size(); i++){
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->yellow_cones[i].x, cone_data->yellow_cones[i].y));
            cones.push_back(to_add);
        }
        for(uint i = 0; i < cone_data->big_orange_cones.size(); i++){
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->big_orange_cones[i].x, cone_data->big_orange_cones[i].y));
            cones.push_back(to_add);
            orangeCones.push_back(to_add);
        }   

        //check to see if you've seen orange cones again
        if(orangeCones.size() == 0){
          orangeNotSeen++;
          if(orangeNotSeen >= 25){
            // std::cout <<"orange not seen flag true" << std::endl;
            orangeNotSeenFlag = true;
          }
        }
        else{
          orangeNotSeen = 0;
        }

        if(orangeCones.size() == 2){ //what if there's more than 2?
          // std::cout <<"Added two cones to orange cones"<<endl;
          bool left = false;
          bool right = false;
          vector<Point2> orangeLoopReferenceCones(2);
          for(uint i = 0; i <  orangeCones.size(); i++){
            if(left == false && orangeCones[i].y() < 0 ){
              //add the left cone here
              left = true;
              orangeLoopReferenceCones[0] = orangeCones[i];
            }
            if(right == false && orangeCones[i].y() > 0 ){
              //add the left cone here
              right = true;
              orangeLoopReferenceCones[1] = orangeCones[i];
            }
          }
          orangeCones = orangeLoopReferenceCones;
        }

        if(orangeNotSeenFlag == true && orangeCones.size() >= 2){
          // std::cout<<"found loop closure" << std::endl;
          loopClosure = true; //TODO: does not account for when there is only a single frame that it sees orange cones
        }

        //run every time you get a measurement
        // slam_instance.step(global_odom, cones);
        // RCLCPP_INFO(this->get_logger(), "NUM_LANDMARKS: %i\n", (slam_instance.n_landmarks));
    }
    // void vehicle_state_callback(const eufs_msgs::msg::CarState::SharedPtr vehicle_state_data){
    void vehicle_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr vehicle_pos_data){
        double LAT_TO_M = 111320;
        double LON_TO_M = 40075000;
        double x_meters;
        double y_meters;
        //TODO: filter
        x_meters = LAT_TO_M*vehicle_pos_data->latitude;
        y_meters = LON_TO_M*cos(vehicle_pos_data->longitude)/360; 

        if(init_odom.x() == -1 && init_odom.y() == -1){
          init_odom = gtsam::Pose2(x_meters,y_meters,init_odom.theta());
        }

        veh_state.x = x_meters-init_odom.x();
        veh_state.y = y_meters-init_odom.y();

        // RCLCPP_INFO(this->get_logger(), "vehicle gps:(%f,%f)",vehicle_pos_data->latitude,vehicle_pos_data->longitude);
        // RCLCPP_INFO(this->get_logger(), "vehicle meters:(%cone_callbackf,%f)\n",veh_state.x,veh_state.y);
    }

    void vehicle_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr vehicle_vel_data){
        // RCLCPP_INFO(this->get_logger(), "vehicle vel:(%f,%f)\n",vehicle_vel_data->twist.linear.x,vehicle_vel_data->twist.linear.y,vehicle_vel_data->twist.linear.z);
        veh_state.dx = vehicle_vel_data->twist.linear.x;
        veh_state.dy = vehicle_vel_data->twist.linear.y;
        veh_state.dyaw = vehicle_vel_data->twist.angular.z;
    }

    //Takes Quaternion and translates to angle
    void vehicle_angle_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr vehicle_angle_data){
        // RCLCPP_INFO(this->get_logger(), "vehicle angle:(%f,%f,%f,%f)\n",vehicle_angle_data->quaternion.x,vehicle_angle_data->quaternion.y,vehicle_angle_data->quaternion.z,vehicle_angle_data->quaternion.w);
        double q1 = vehicle_angle_data->quaternion.x;
        double q2 = vehicle_angle_data->quaternion.y;
        double q3 = vehicle_angle_data->quaternion.z;
        double q0 = vehicle_angle_data->quaternion.w;
        double yaw = atan2(2*(q0*q3+q1*q2), pow(q0, 2)+pow(q1, 2)-pow(q2, 2)-pow(q3, 2));
        // RCLCPP_INFO(this->get_logger(), "vehicle yaw:%f",veh_state.yaw);

        if(init_odom.theta() == -1){
          init_odom = gtsam::Pose2(init_odom.x(),init_odom.y(),yaw);
        }

        veh_state.yaw = yaw-init_odom.theta(); 
    }

 
    void timer_callback(){
      // RCLCPP_INFO(this->get_logger(), "vehicle callback:\n");
      global_odom = gtsam::Pose2(veh_state.x, veh_state.y, veh_state.yaw);
      velocity = gtsam::Point2(veh_state.dx,veh_state.dy);
      dt = 69;
      RCLCPP_INFO(this->get_logger(), "Global Odom:(%f,%f,%f)",veh_state.x,veh_state.y,veh_state.yaw);

      // global_odom = gtsam::Pose2(vehicle_state_data>pose.pose.position.x, vehicle_state_data->pose.pose.position.y, yaw);
      // velocity = gtsam::Point2(vehicle_state_data->twist.twist.linear.x,vehicle_state_data->twist.twist.linear.y);

      // long tempTime = vehicle_state_data->header.stamp.sec*SEC_TO_NANOSEC + vehicle_state_data->header.stamp.nanosec;
      // dt = time_ns - tempTime;
      // time_ns = tempTime;  
      run_slam();
    }
 
    void run_slam(){
        slam_instance.step(this->get_logger(), global_odom, cones,orangeCones, velocity, dt, loopClosure);
        // RCLCPP_INFO(this->get_logger(), "NUM_LANDMARKS: %i\n", (slam_instance.n_landmarks));
    }
    // ISAM2Params parameters;
    // parameters.RelinearizationThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    slamISAM slam_instance = slamISAM();
    // rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_sub;
    rclcpp::Subscription<eufs_msgs::msg::ConeArray>::SharedPtr cone_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr vehicle_pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr vehicle_angle_sub;

    gtsam::Point2 velocity;  // local variable to load velocity into SLAM instance
    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    vector<Point2> cones; // local variable to load cone observations into SLAM instance
    vector<Point2> orangeCones; // local variable to load cone observations into SLAM instance


    gtsam::Pose2 init_odom; // local variable to load odom into SLAM instance

    VehiclePosition veh_state = VehiclePosition();

    rclcpp::TimerBase::SharedPtr timer;
    long dt;
    long time_ns;
    int orangeNotSeen;
    bool orangeNotSeenFlag;
    bool loopClosure;
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