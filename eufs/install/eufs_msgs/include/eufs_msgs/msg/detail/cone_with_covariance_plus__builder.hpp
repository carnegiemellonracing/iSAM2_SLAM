// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eufs_msgs:msg/ConeWithCovariancePlus.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__BUILDER_HPP_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__BUILDER_HPP_

#include "eufs_msgs/msg/detail/cone_with_covariance_plus__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace eufs_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeWithCovariancePlus_covariance
{
public:
  explicit Init_ConeWithCovariancePlus_covariance(::eufs_msgs::msg::ConeWithCovariancePlus & msg)
  : msg_(msg)
  {}
  ::eufs_msgs::msg::ConeWithCovariancePlus covariance(::eufs_msgs::msg::ConeWithCovariancePlus::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithCovariancePlus msg_;
};

class Init_ConeWithCovariancePlus_point
{
public:
  explicit Init_ConeWithCovariancePlus_point(::eufs_msgs::msg::ConeWithCovariancePlus & msg)
  : msg_(msg)
  {}
  Init_ConeWithCovariancePlus_covariance point(::eufs_msgs::msg::ConeWithCovariancePlus::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_ConeWithCovariancePlus_covariance(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithCovariancePlus msg_;
};

class Init_ConeWithCovariancePlus_unknown_prob
{
public:
  explicit Init_ConeWithCovariancePlus_unknown_prob(::eufs_msgs::msg::ConeWithCovariancePlus & msg)
  : msg_(msg)
  {}
  Init_ConeWithCovariancePlus_point unknown_prob(::eufs_msgs::msg::ConeWithCovariancePlus::_unknown_prob_type arg)
  {
    msg_.unknown_prob = std::move(arg);
    return Init_ConeWithCovariancePlus_point(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithCovariancePlus msg_;
};

class Init_ConeWithCovariancePlus_big_orange_prob
{
public:
  explicit Init_ConeWithCovariancePlus_big_orange_prob(::eufs_msgs::msg::ConeWithCovariancePlus & msg)
  : msg_(msg)
  {}
  Init_ConeWithCovariancePlus_unknown_prob big_orange_prob(::eufs_msgs::msg::ConeWithCovariancePlus::_big_orange_prob_type arg)
  {
    msg_.big_orange_prob = std::move(arg);
    return Init_ConeWithCovariancePlus_unknown_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithCovariancePlus msg_;
};

class Init_ConeWithCovariancePlus_orange_prob
{
public:
  explicit Init_ConeWithCovariancePlus_orange_prob(::eufs_msgs::msg::ConeWithCovariancePlus & msg)
  : msg_(msg)
  {}
  Init_ConeWithCovariancePlus_big_orange_prob orange_prob(::eufs_msgs::msg::ConeWithCovariancePlus::_orange_prob_type arg)
  {
    msg_.orange_prob = std::move(arg);
    return Init_ConeWithCovariancePlus_big_orange_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithCovariancePlus msg_;
};

class Init_ConeWithCovariancePlus_yellow_prob
{
public:
  explicit Init_ConeWithCovariancePlus_yellow_prob(::eufs_msgs::msg::ConeWithCovariancePlus & msg)
  : msg_(msg)
  {}
  Init_ConeWithCovariancePlus_orange_prob yellow_prob(::eufs_msgs::msg::ConeWithCovariancePlus::_yellow_prob_type arg)
  {
    msg_.yellow_prob = std::move(arg);
    return Init_ConeWithCovariancePlus_orange_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithCovariancePlus msg_;
};

class Init_ConeWithCovariancePlus_blue_prob
{
public:
  explicit Init_ConeWithCovariancePlus_blue_prob(::eufs_msgs::msg::ConeWithCovariancePlus & msg)
  : msg_(msg)
  {}
  Init_ConeWithCovariancePlus_yellow_prob blue_prob(::eufs_msgs::msg::ConeWithCovariancePlus::_blue_prob_type arg)
  {
    msg_.blue_prob = std::move(arg);
    return Init_ConeWithCovariancePlus_yellow_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithCovariancePlus msg_;
};

class Init_ConeWithCovariancePlus_id
{
public:
  Init_ConeWithCovariancePlus_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeWithCovariancePlus_blue_prob id(::eufs_msgs::msg::ConeWithCovariancePlus::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_ConeWithCovariancePlus_blue_prob(msg_);
  }

private:
  ::eufs_msgs::msg::ConeWithCovariancePlus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eufs_msgs::msg::ConeWithCovariancePlus>()
{
  return eufs_msgs::msg::builder::Init_ConeWithCovariancePlus_id();
}

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__BUILDER_HPP_
