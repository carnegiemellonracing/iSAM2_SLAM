// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eufs_msgs:msg/ConeWithCovariancePlus.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__TRAITS_HPP_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__TRAITS_HPP_

#include "eufs_msgs/msg/detail/cone_with_covariance_plus__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<eufs_msgs::msg::ConeWithCovariancePlus>()
{
  return "eufs_msgs::msg::ConeWithCovariancePlus";
}

template<>
inline const char * name<eufs_msgs::msg::ConeWithCovariancePlus>()
{
  return "eufs_msgs/msg/ConeWithCovariancePlus";
}

template<>
struct has_fixed_size<eufs_msgs::msg::ConeWithCovariancePlus>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<eufs_msgs::msg::ConeWithCovariancePlus>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<eufs_msgs::msg::ConeWithCovariancePlus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__TRAITS_HPP_
