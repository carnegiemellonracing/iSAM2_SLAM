// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eufs_msgs:msg/ConeWithCovariancePlus.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__STRUCT_HPP_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__eufs_msgs__msg__ConeWithCovariancePlus __attribute__((deprecated))
#else
# define DEPRECATED__eufs_msgs__msg__ConeWithCovariancePlus __declspec(deprecated)
#endif

namespace eufs_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ConeWithCovariancePlus_
{
  using Type = ConeWithCovariancePlus_<ContainerAllocator>;

  explicit ConeWithCovariancePlus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->blue_prob = 0.0;
      this->yellow_prob = 0.0;
      this->orange_prob = 0.0;
      this->big_orange_prob = 0.0;
      this->unknown_prob = 0.0;
      std::fill<typename std::array<double, 4>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  explicit ConeWithCovariancePlus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point(_alloc, _init),
    covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->blue_prob = 0.0;
      this->yellow_prob = 0.0;
      this->orange_prob = 0.0;
      this->big_orange_prob = 0.0;
      this->unknown_prob = 0.0;
      std::fill<typename std::array<double, 4>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _blue_prob_type =
    double;
  _blue_prob_type blue_prob;
  using _yellow_prob_type =
    double;
  _yellow_prob_type yellow_prob;
  using _orange_prob_type =
    double;
  _orange_prob_type orange_prob;
  using _big_orange_prob_type =
    double;
  _big_orange_prob_type big_orange_prob;
  using _unknown_prob_type =
    double;
  _unknown_prob_type unknown_prob;
  using _point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _point_type point;
  using _covariance_type =
    std::array<double, 4>;
  _covariance_type covariance;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__blue_prob(
    const double & _arg)
  {
    this->blue_prob = _arg;
    return *this;
  }
  Type & set__yellow_prob(
    const double & _arg)
  {
    this->yellow_prob = _arg;
    return *this;
  }
  Type & set__orange_prob(
    const double & _arg)
  {
    this->orange_prob = _arg;
    return *this;
  }
  Type & set__big_orange_prob(
    const double & _arg)
  {
    this->big_orange_prob = _arg;
    return *this;
  }
  Type & set__unknown_prob(
    const double & _arg)
  {
    this->unknown_prob = _arg;
    return *this;
  }
  Type & set__point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->point = _arg;
    return *this;
  }
  Type & set__covariance(
    const std::array<double, 4> & _arg)
  {
    this->covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator> *;
  using ConstRawPtr =
    const eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eufs_msgs__msg__ConeWithCovariancePlus
    std::shared_ptr<eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eufs_msgs__msg__ConeWithCovariancePlus
    std::shared_ptr<eufs_msgs::msg::ConeWithCovariancePlus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConeWithCovariancePlus_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->blue_prob != other.blue_prob) {
      return false;
    }
    if (this->yellow_prob != other.yellow_prob) {
      return false;
    }
    if (this->orange_prob != other.orange_prob) {
      return false;
    }
    if (this->big_orange_prob != other.big_orange_prob) {
      return false;
    }
    if (this->unknown_prob != other.unknown_prob) {
      return false;
    }
    if (this->point != other.point) {
      return false;
    }
    if (this->covariance != other.covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConeWithCovariancePlus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConeWithCovariancePlus_

// alias to use template instance with default allocator
using ConeWithCovariancePlus =
  eufs_msgs::msg::ConeWithCovariancePlus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__STRUCT_HPP_
