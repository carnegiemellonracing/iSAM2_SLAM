#include "ros_utils.hpp"

bool start_pose_in_front(Pose2 &cur_pose, Pose2 &first_pose, optional<rclcpp::Logger> logger);

bool detect_loop_closure(Pose2 &cur_pose,Pose2 &first_pose, int pose_num, optional<rclcpp::Logger> logger);