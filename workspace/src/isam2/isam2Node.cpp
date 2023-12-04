#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "eufs_msgs/msg/car_state.hpp"

#include <gtsam/nonlinear/ISAM2Params.h>

#include "isam2.cpp"

#include <boost/shared_ptr.hpp>
#include <vector>
#include <cmath>
#include <chrono>

#define CONE_DATA_TOPIC "/ground_truth/cones"
#define VEHICLE_DATA_TOPIC "/ground_truth/state"

using namespace std;
using std::placeholders::_1;

struct Cone{
  double x;
  double y;

};

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
      cone_sub = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
      CONE_DATA_TOPIC, 10, std::bind(&SLAMValidation::cone_callback, this, _1));
      vehicle_state_sub = this->create_subscription<eufs_msgs::msg::CarState>(
      VEHICLE_DATA_TOPIC, 10, std::bind(&SLAMValidation::vehicle_state_callback, this, _1));
      timer = this->create_wall_timer(100ms, std::bind(&SLAMValidation::timer_callback, this));
    }
  private:
    void cone_callback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr cone_data){
        RCLCPP_INFO(this->get_logger(), "CONECALLBACK: B: %i| Y: %i| O: %i", cone_data->blue_cones.size(), cone_data->yellow_cones.size(), cone_data->orange_cones.size());
        cones.clear();
        for(uint i = 0; i < cone_data->blue_cones.size(); i++){
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->blue_cones[i].point.x, cone_data->blue_cones[i].point.y));
            cones.push_back(to_add);
        }
        for(uint i = 0; i < cone_data->yellow_cones.size(); i++){
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->yellow_cones[i].point.x, cone_data->yellow_cones[i].point.y));
            cones.push_back(to_add);
        }
        for(uint i = 0; i < cone_data->orange_cones.size(); i++){
            gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->orange_cones[i].point.x, cone_data->orange_cones[i].point.y));
            cones.push_back(to_add);
        }
    }
    void vehicle_state_callback(const eufs_msgs::msg::CarState::SharedPtr vehicle_state_data){
        RCLCPP_INFO(this->get_logger(), "vehicle state\n");
        double q1 = vehicle_state_data->pose.pose.orientation.x;
        double q2 = vehicle_state_data->pose.pose.orientation.y;
        double q3 = vehicle_state_data->pose.pose.orientation.z;
        double q0 = vehicle_state_data->pose.pose.orientation.w;
        double yaw = atan2(2*(q0*q3+q1*q2), pow(q0, 2)+pow(q1, 2)-pow(q2, 2)-pow(q3, 2));

        global_odom = gtsam::Pose2(vehicle_state_data->pose.pose.position.x, vehicle_state_data->pose.pose.position.y, yaw);
    }
    void timer_callback(){
      run_slam();
    }

    void run_slam(){
        slam_instance.step(this->get_logger(),global_odom, cones);
        RCLCPP_INFO(this->get_logger(), "got output\n");
        RCLCPP_INFO(this->get_logger(), "NUM_LANDMARKS: %i\n", (slam_instance.n_landmarks));

    }
    // ISAM2Params parameters;
    // parameters.RelinearizationThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    slamISAM slam_instance = slamISAM();
    rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_sub;
    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr vehicle_state_sub;

    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    vector<Point2> cones; // local variable to load cone observations into SLAM instance

    rclcpp::TimerBase::SharedPtr timer;
    double dt;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAMValidation>());
  rclcpp::shutdown();
  return 0;
}