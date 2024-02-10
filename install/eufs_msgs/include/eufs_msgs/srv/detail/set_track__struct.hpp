// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eufs_msgs:srv/SetTrack.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__SRV__DETAIL__SET_TRACK__STRUCT_HPP_
#define EUFS_MSGS__SRV__DETAIL__SET_TRACK__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'data'
#include "eufs_msgs/msg/detail/cone_array_with_covariance__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__eufs_msgs__srv__SetTrack_Request __attribute__((deprecated))
#else
# define DEPRECATED__eufs_msgs__srv__SetTrack_Request __declspec(deprecated)
#endif

namespace eufs_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTrack_Request_
{
  using Type = SetTrack_Request_<ContainerAllocator>;

  explicit SetTrack_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_init)
  {
    (void)_init;
  }

  explicit SetTrack_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _data_type =
    eufs_msgs::msg::ConeArrayWithCovariance_<ContainerAllocator>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const eufs_msgs::msg::ConeArrayWithCovariance_<ContainerAllocator> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eufs_msgs::srv::SetTrack_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const eufs_msgs::srv::SetTrack_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eufs_msgs::srv::SetTrack_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eufs_msgs::srv::SetTrack_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::srv::SetTrack_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::srv::SetTrack_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::srv::SetTrack_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::srv::SetTrack_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eufs_msgs::srv::SetTrack_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eufs_msgs::srv::SetTrack_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eufs_msgs__srv__SetTrack_Request
    std::shared_ptr<eufs_msgs::srv::SetTrack_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eufs_msgs__srv__SetTrack_Request
    std::shared_ptr<eufs_msgs::srv::SetTrack_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTrack_Request_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTrack_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTrack_Request_

// alias to use template instance with default allocator
using SetTrack_Request =
  eufs_msgs::srv::SetTrack_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace eufs_msgs


#ifndef _WIN32
# define DEPRECATED__eufs_msgs__srv__SetTrack_Response __attribute__((deprecated))
#else
# define DEPRECATED__eufs_msgs__srv__SetTrack_Response __declspec(deprecated)
#endif

namespace eufs_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTrack_Response_
{
  using Type = SetTrack_Response_<ContainerAllocator>;

  explicit SetTrack_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetTrack_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eufs_msgs::srv::SetTrack_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const eufs_msgs::srv::SetTrack_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eufs_msgs::srv::SetTrack_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eufs_msgs::srv::SetTrack_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::srv::SetTrack_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::srv::SetTrack_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::srv::SetTrack_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::srv::SetTrack_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eufs_msgs::srv::SetTrack_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eufs_msgs::srv::SetTrack_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eufs_msgs__srv__SetTrack_Response
    std::shared_ptr<eufs_msgs::srv::SetTrack_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eufs_msgs__srv__SetTrack_Response
    std::shared_ptr<eufs_msgs::srv::SetTrack_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTrack_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTrack_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTrack_Response_

// alias to use template instance with default allocator
using SetTrack_Response =
  eufs_msgs::srv::SetTrack_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace eufs_msgs

namespace eufs_msgs
{

namespace srv
{

struct SetTrack
{
  using Request = eufs_msgs::srv::SetTrack_Request;
  using Response = eufs_msgs::srv::SetTrack_Response;
};

}  // namespace srv

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__SRV__DETAIL__SET_TRACK__STRUCT_HPP_
