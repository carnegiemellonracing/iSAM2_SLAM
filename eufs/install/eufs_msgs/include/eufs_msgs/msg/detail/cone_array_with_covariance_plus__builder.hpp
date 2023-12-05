// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eufs_msgs:msg/ConeArrayWithCovariancePlus.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_ARRAY_WITH_COVARIANCE_PLUS__BUILDER_HPP_
#define EUFS_MSGS__MSG__DETAIL__CONE_ARRAY_WITH_COVARIANCE_PLUS__BUILDER_HPP_

#include "eufs_msgs/msg/detail/cone_array_with_covariance_plus__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace eufs_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeArrayWithCovariancePlus_cones
{
public:
  explicit Init_ConeArrayWithCovariancePlus_cones(::eufs_msgs::msg::ConeArrayWithCovariancePlus & msg)
  : msg_(msg)
  {}
  ::eufs_msgs::msg::ConeArrayWithCovariancePlus cones(::eufs_msgs::msg::ConeArrayWithCovariancePlus::_cones_type arg)
  {
    msg_.cones = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::msg::ConeArrayWithCovariancePlus msg_;
};

class Init_ConeArrayWithCovariancePlus_header
{
public:
  Init_ConeArrayWithCovariancePlus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeArrayWithCovariancePlus_cones header(::eufs_msgs::msg::ConeArrayWithCovariancePlus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ConeArrayWithCovariancePlus_cones(msg_);
  }

private:
  ::eufs_msgs::msg::ConeArrayWithCovariancePlus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::msg::ConeArrayWithCovariancePlus>()
{
  return eufs_msgs::msg::builder::Init_ConeArrayWithCovariancePlus_header();
}

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_ARRAY_WITH_COVARIANCE_PLUS__BUILDER_HPP_
