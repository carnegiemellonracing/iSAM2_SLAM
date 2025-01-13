#include "ros_utils.hpp"

const double DIST_FROM_START_LC_TH = 5; //meters; distance from the start for loop closure detection
bool start_pose_in_front(Pose2 &cur_pose, Pose2 &first_pose, optional<rclcpp::Logger> logger);

bool detect_loop_closure(Pose2 &cur_pose,Pose2 &first_pose, int pose_num, optional<rclcpp::Logger> logger);