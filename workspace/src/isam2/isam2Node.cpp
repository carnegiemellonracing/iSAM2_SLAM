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
      // cone_sub = this->create_subscription<interfaces::msg::ConeArrayWithCovariance>(
      rmw_qos_profile_t best_effort_qos = rmw_qos_profile_default;
      cone_sub = this->create_subscription<interfaces::msg::ConeArray>(
          CONE_DATA_TOPIC, 10, std::bind(&SLAMValidation::cone_callback, this, _1));

      // vehicle_state_sub = this->create_subscription<interfaces::msg::CarState>(
      // VEHICLE_DATA_TOPIC, 10, std::bind(&SLAMValidation::vehicle_state_callback, this, _1));

      vehicle_pos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      VEHICLE_POS_TOPIC, 10, std::bind(&SLAMValidation::vehicle_pos_callback, this, _1));

      vehicle_angle_sub = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      VEHICLE_ANGLE_TOPIC, 10, std::bind(&SLAMValidation::vehicle_angle_callback, this, _1));

      vehicle_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      VEHICLE_VEL_TOPIC, 10, std::bind(&SLAMValidation::vehicle_vel_callback, this, _1));

      //TODO: need to initalize robot state?????
      timer = this->create_wall_timer(20ms, std::bind(&SLAMValidation::timer_callback, this));
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
      RCLCPP_INFO(this->get_logger(), "CONECALLBACK: B: %i| Y: %i| O: %i", cone_data->blue_cones.size(), cone_data->yellow_cones.size(), cone_data->orange_cones.size());
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
            // add the left cone here
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

      // run every time you get a measurement
      //  slam_instance.step(global_odom, cones);
      //  RCLCPP_INFO(this->get_logger(), "NUM_LANDMARKS: %i\n", (slam_instance.n_landmarks));
    }
    // void vehicle_state_callback(const interfaces::msg::CarState::SharedPtr vehicle_state_data){
    void vehicle_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr vehicle_pos_data){
        double LAT_TO_M = 111320;
        double LON_TO_M = 40075000;
        double x_meters;
        double y_meters;
        //TODO: filter
        // x_meters = LAT_TO_M*vehicle_pos_data->longitude;
        // y_meters = LON_TO_M*cos(vehicle_pos_data->latitude)/360;

        x_meters = -(LAT_TO_M * vehicle_pos_data->latitude);
        y_meters = LAT_TO_M * vehicle_pos_data->longitude;

        if(init_odom.x() == -1 && init_odom.y() == -1){
          init_odom = gtsam::Pose2(x_meters,y_meters,init_odom.theta());
        }

        veh_state.x = x_meters-init_odom.x();
        veh_state.y = y_meters-init_odom.y();

        veh_state.yaw = atan2(veh_state.y - prev_veh_state.y, veh_state.x - prev_veh_state.x) - (3.1415926535 / 2);

        if (init_odom.theta() == -1)
        {
          init_odom = gtsam::Pose2(init_odom.x(), init_odom.y(), veh_state.yaw);
        }

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
        double q0 = vehicle_angle_data->quaternion.w;
        double q1 = vehicle_angle_data->quaternion.x;
        double q2 = vehicle_angle_data->quaternion.y;
        double q3 = vehicle_angle_data->quaternion.z;

        double yaw   = atan2(2*(q3*q0 +q1*q2) , -1+2*(q0*q0 +q1*q1)) ; //rotate by 90 degrees? TODO: add to velocity

        // RCLCPP_INFO(this->get_logger(), "vehicle yaw:%f",veh_state.yaw);
        // if(init_odom.theta() == -1){
        //   init_odom = gtsam::Pose2(init_odom.x(),init_odom.y(),yaw);
        //   veh_state.yaw = yaw; //-init_odom.theta(); 
        // }

    }

    void timer_callback(){
      // RCLCPP_INFO(this->get_logger(), "vehicle callback:\n");
      global_odom = gtsam::Pose2(veh_state.x, veh_state.y, veh_state.yaw);
      RCLCPP_INFO(this->get_logger(), "(%f,%f,%f)",veh_state.x,veh_state.y,veh_state.yaw);

      //PRINT POSES
      // outfile.open("chipmunk.txt", std::ios_base::app);
      // outfile << "(" << veh_state.x << ","<< veh_state.y << ","<< veh_state.yaw<< ")\n";
      // outfile.close();
      ////////////////////////////

      velocity = gtsam::Point2(veh_state.dx,veh_state.dy);
      dt = 69;
      // RCLCPP_INFO(this->get_logger(), "Global Odom:(%f,%f,%f)",veh_state.x,veh_state.y,veh_state.yaw);
      // global_odom = gtsam::Pose2(vehicle_state_data>pose.pose.position.x, vehicle_state_data->pose.pose.position.y, yaw);
      // velocity = gtsam::Point2(vehicle_state_data->twist.twist.linear.x,vehicle_state_data->twist.twist.linear.y);

      // long tempTime = vehicle_state_data->header.stamp.sec*SEC_TO_NANOSEC + vehicle_state_data->header.stamp.nanosec;
      // dt = time_ns - tempTime;
      // time_ns = tempTime;  
      run_slam();
      prev_veh_state = veh_state;
    }

    bool almost_equal(Pose2 a, Pose2 b) {
      double threshold = 0.05;
      return std::fabs(a.x() - b.x()) < threshold && std::fabs(a.y() - b.y()) < threshold;
    }
                                                                                                                                               
    std::vector<Pose2> xTruth;
    //building xTruth
    std::vector<int>append_cones() {
      std::vector<int> annot_obs_cones;
      bool new_cone = true;
      int cone_id = -1;
      for(Point2 cone: cones) {
        new_cone = true;
        double range = std::sqrt(cone.x() * cone.x() + cone.y() * cone.y());
        double bearing = std::atan2(cone.y(), cone.x());
                                                                                                                                               
        double global_cone_x = global_odom.x() + range * cos(bearing+global_odom.theta());
        double global_cone_y = global_odom.y() + range * sin(bearing+global_odom.theta());
        
        Pose2 global_coords(global_cone_x, global_cone_y, -1);
        for(long unsigned int i = 0; i < xTruth.size(); i++) {
          Pose2 other_cones = xTruth[i];
          if(almost_equal(global_coords, other_cones)) {
            new_cone = false;
            cone_id = i;
            break;
          }
        }
                                                                                                                                               
                                                                                                                                                   
        if(new_cone) {
          global_coords = gtsam::Pose2(global_cone_x, global_cone_y, xTruth.size());
          xTruth.push_back(global_coords);
          RCLCPP_INFO(logger, "Size of xTruth: %d \t | \t Number of data_association_errors: %d\n\n", xTruth.size(), data_association_errors);
        }
        //std::tuple<int> tp(cone_id);
        annot_obs_cones.push_back(cone_id);
        
      }
                                                                                                                                               
                                                                                                                                               
      return annot_obs_cones;
    }








    void run_slam(){
      vector<int> annot_obs_cones = append_cones();

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
          // ofs << "(" << cone.x() << "," << cone.y() << ")\n"; // local cones

          // ofs << "global:(" << global_cone_x << ","<< global_cone_y << ")\n";
          // ofs << "local:(" << cone.x() << "," << cone.y() << ")\n";
      }
      ofs.close();
      std::cout.rdbuf(coutbuf); //reset to standard output again
      // pose_cones.open("pose_cones.txt", std::ios_base::app);
      
      // pose_cones.close();
      ///////////

      slam_instance.step(this->get_logger(), global_odom, cones,orangeCones, velocity, dt, loopClosure);
      // RCLCPP_INFO(this->get_logger(), "NUM_LANDMARKS: %i\n", (slam_instance.n_landmarks));
    }
    // ISAM2Params parameters;
    // parameters.RelinearizationThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    slamISAM slam_instance = slamISAM();
    // rclcpp::Subscription<interfaces::msg::ConeArrayWithCovariance>::SharedPtr cone_sub;
    rclcpp::Subscription<interfaces::msg::ConeArray>::SharedPtr cone_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr vehicle_pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr vehicle_angle_sub;

    gtsam::Point2 velocity;  // local variable to load velocity into SLAM instance
    gtsam::Pose2 global_odom; // local variable to load odom into SLAM instance
    vector<Point2> cones; // local variable to load cone observations into SLAM instance
    vector<Point2> orangeCones; // local variable to load cone observations into SLAM instance


    gtsam::Pose2 init_odom; // local variable to load odom into SLAM instance
    VehiclePosition prev_veh_state = VehiclePosition();
    VehiclePosition veh_state = VehiclePosition();

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
