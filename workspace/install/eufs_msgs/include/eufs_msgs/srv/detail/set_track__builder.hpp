// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eufs_msgs:srv/SetTrack.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__SRV__DETAIL__SET_TRACK__BUILDER_HPP_
#define EUFS_MSGS__SRV__DETAIL__SET_TRACK__BUILDER_HPP_

#include "eufs_msgs/srv/detail/set_track__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace eufs_msgs
{

namespace srv
{

namespace builder
{

class Init_SetTrack_Request_data
{
public:
  Init_SetTrack_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::eufs_msgs::srv::SetTrack_Request data(::eufs_msgs::srv::SetTrack_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::srv::SetTrack_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::srv::SetTrack_Request>()
{
  return eufs_msgs::srv::builder::Init_SetTrack_Request_data();
}

}  // namespace eufs_msgs


namespace eufs_msgs
{

namespace srv
{

namespace builder
{

class Init_SetTrack_Response_message
{
public:
  explicit Init_SetTrack_Response_message(::eufs_msgs::srv::SetTrack_Response & msg)
  : msg_(msg)
  {}
  ::eufs_msgs::srv::SetTrack_Response message(::eufs_msgs::srv::SetTrack_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::srv::SetTrack_Response msg_;
};

class Init_SetTrack_Response_success
{
public:
  Init_SetTrack_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTrack_Response_message success(::eufs_msgs::srv::SetTrack_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetTrack_Response_message(msg_);
  }

private:
  ::eufs_msgs::srv::SetTrack_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::srv::SetTrack_Response>()
{
  return eufs_msgs::srv::builder::Init_SetTrack_Response_success();
}

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__SRV__DETAIL__SET_TRACK__BUILDER_HPP_
