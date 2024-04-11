#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "interfaces/msg/cone_array.hpp"

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
    SLAMValidation(): Node("slam_validation"){
      // cone_sub = this->create_subscription<interfaces::msg::ConeArrayWithCovariance>(
      rmw_qos_profile_t best_effort_qos = rmw_qos_profile_default;
      cone_sub = this->create_subscription<interfaces::msg::ConeArray>(
          CONE_DATA_TOPIC, 10, std::bind(&SLAMValidation::cone_callback, this, _1));

      // For sim
      // vehicle_state_sub = this->create_subscription<interfaces::msg::CarState>(
      // VEHICLE_DATA_TOPIC, 10, std::bind(&SLAMValidation::vehicle_state_callback, this, _1));

      //bring back///////////////
      // vehicle_pos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      // VEHICLE_POS_TOPIC, 10, std::bind(&SLAMValidation::vehicle_pos_callback, this, _1));

      // vehicle_angle_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      // VEHICLE_ANGLE_TOPIC, 10, std::bind(&SLAMValidation::vehicle_angle_callback, this, _1));

      // vehicle_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      // VEHICLE_VEL_TOPIC, 10, std::bind(&SLAMValidation::vehicle_vel_callback, this, _1));
      ////////////////////////

      //TODO: need to initalize robot state?????
      // timer = this->create_wall_timer(100ms, std::bind(&SLAMValidation::timer_callback, this));
      time_ns = 0; //Initialize time and dt
      dt = .1;
      orangeNotSeen = 25;
      orangeNotSeenFlag = false;
      loopClosure = false;


      init_odom = gtsam::Pose2(-1,-1,-1);

    }
  private:
    // positive y forward
    // positive x right
    void cone_callback(const interfaces::msg::ConeArray::SharedPtr cone_data)
    {
      // RCLCPP_INFO(this->get_logger(), "CONECALLBACK: B: %i| Y: %i| O: %i", cone_data->blue_cones.size(), cone_data->yellow_cones.size(), cone_data->orange_cones.size());
      // return;
      cones.clear();
      orangeCones.clear();
      for (uint i = 0; i < cone_data->blue_cones.size(); i++)
      {
        gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->blue_cones[i].x, cone_data->blue_cones[i].y));
        cones.push_back(to_add);
      }
      for (uint i = 0; i < cone_data->yellow_cones.size(); i++)
      {
        gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->yellow_cones[i].x, cone_data->yellow_cones[i].y));
        cones.push_back(to_add);
      }
      for (uint i = 0; i < cone_data->big_orange_cones.size(); i++)
      {
        gtsam::Point2 to_add = gtsam::Point2(Eigen::Vector2d(cone_data->big_orange_cones[i].x, cone_data->big_orange_cones[i].y));
        cones.push_back(to_add);
        orangeCones.push_back(to_add);
      }

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


      //process pose
      if (init_odom.x() == -1 && init_odom.y() == -1)
      {
        init_odom = gtsam::Pose2(cone_data->pose.x, cone_data->pose.y, cone_data->pose.theta);
      }

      else{

        global_odom = gtsam::Pose2(cone_data->pose.x - init_odom.x(), cone_data->pose.y - init_odom.y(), cone_data->pose.theta - init_odom.theta());
        RCLCPP_INFO(this->get_logger(), "definitely fucked pose: (%f,%f,%f)", global_odom.x(), global_odom.y(), global_odom.theta());

        run_slam();
      }


    }

    void run_slam(){

      if(global_odom.x() == 0 || global_odom.y() == 0 || global_odom.theta() == 0){
        RCLCPP_INFO(this->get_logger(), "fucked pose: (%f,%f,%f)", global_odom.x(), global_odom.y(), global_odom.theta());
        return;
      }

      // print pose and cones
      std::ofstream ofs;
      std::ofstream out("urmom.txt");
      std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
      std::cout.rdbuf(out.rdbuf());
      ofs.open("urmom.txt", std::ofstream::out | std::ofstream::trunc);
      ofs << "(" << veh_state.x << ","<< veh_state.y << ","<< veh_state.yaw<< ")\n";
      for(auto cone: cones){
          //CHANGED BY MELINDA: yeah so cones look like they're flipped about the car y axis for some reason ?
          double range = norm2(cone);

          double bearing = std::atan2(cone.y(), cone.x());//+ global_odom.theta();
          double global_cone_x = global_odom.x() + range*cos(bearing+global_odom.theta());
          double global_cone_y = global_odom.y() + range*sin(bearing+global_odom.theta());
          ofs << "(" << global_cone_x << ","<< global_cone_y << ")\n"; //original
      }
      ofs.close();
      std::cout.rdbuf(coutbuf); //reset to standard output again
      slam_instance.step(this->get_logger(), global_odom, cones,orangeCones, velocity, dt, loopClosure);
      // RCLCPP_INFO(this->get_logger(), "NUM_LANDMARKS: %i\n", (slam_instance.n_landmarks));
    }

    slamISAM slam_instance = slamISAM();
    rclcpp::Subscription<interfaces::msg::ConeArray>::SharedPtr cone_sub;
    //Bring back/////////////////
    // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr vehicle_pos_sub;
    // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_vel_sub;
    // rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr vehicle_angle_sub;
    ///////////////////////////////

    gtsam::Point2 velocity;  // local variable to load velocity into SLAM instance
    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    vector<Point2> cones; // local variable to load cone observations into SLAM instance
    vector<Point2> orangeCones; // local variable to load cone observations into SLAM instance

    gtsam::Pose2 init_odom; // local variable to load odom into SLAM instance
    VehicleState prev_veh_state = VehicleState();
    VehicleState veh_state = VehicleState();

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