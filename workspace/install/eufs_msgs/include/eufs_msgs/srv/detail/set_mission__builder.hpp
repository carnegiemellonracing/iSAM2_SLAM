// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eufs_msgs:srv/SetMission.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__SRV__DETAIL__SET_MISSION__BUILDER_HPP_
#define EUFS_MSGS__SRV__DETAIL__SET_MISSION__BUILDER_HPP_

#include "eufs_msgs/srv/detail/set_mission__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace eufs_msgs
{

namespace srv
{

namespace builder
{

class Init_SetMission_Request_mission
{
public:
  Init_SetMission_Request_mission()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::eufs_msgs::srv::SetMission_Request mission(::eufs_msgs::srv::SetMission_Request::_mission_type arg)
  {
    msg_.mission = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::srv::SetMission_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::srv::SetMission_Request>()
{
  return eufs_msgs::srv::builder::Init_SetMission_Request_mission();
}

}  // namespace eufs_msgs


namespace eufs_msgs
{

namespace srv
{

namespace builder
{

class Init_SetMission_Response_message
{
public:
  explicit Init_SetMission_Response_message(::eufs_msgs::srv::SetMission_Response & msg)
  : msg_(msg)
  {}
  ::eufs_msgs::srv::SetMission_Response message(::eufs_msgs::srv::SetMission_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::srv::SetMission_Response msg_;
};

class Init_SetMission_Response_success
{
public:
  Init_SetMission_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMission_Response_message success(::eufs_msgs::srv::SetMission_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetMission_Response_message(msg_);
  }

private:
  ::eufs_msgs::srv::SetMission_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::srv::SetMission_Response>()
{
  return eufs_msgs::srv::builder::Init_SetMission_Response_success();
}

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__SRV__DETAIL__SET_MISSION__BUILDER_HPP_
