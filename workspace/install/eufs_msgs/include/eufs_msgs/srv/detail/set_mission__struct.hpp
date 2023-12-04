// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eufs_msgs:srv/SetMission.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_HPP_
#define EUFS_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__eufs_msgs__srv__SetMission_Request __attribute__((deprecated))
#else
# define DEPRECATED__eufs_msgs__srv__SetMission_Request __declspec(deprecated)
#endif

namespace eufs_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMission_Request_
{
  using Type = SetMission_Request_<ContainerAllocator>;

  explicit SetMission_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission = 0;
    }
  }

  explicit SetMission_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission = 0;
    }
  }

  // field types and members
  using _mission_type =
    int16_t;
  _mission_type mission;

  // setters for named parameter idiom
  Type & set__mission(
    const int16_t & _arg)
  {
    this->mission = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int16_t NOT_SELECTED =
    0;
  static constexpr int16_t ACCELERATION =
    1;
  static constexpr int16_t SKIDPAD =
    2;
  static constexpr int16_t AUTOCROSS =
    3;
  static constexpr int16_t TRACK_DRIVE =
    4;
  static constexpr int16_t MANUAL_DRIVING =
    5;
  static constexpr int16_t ADS_EBS_TEST =
    6;
  static constexpr int16_t ADS_INSPECTION =
    7;
  static constexpr int16_t DDT_INSPECTION_A =
    8;
  static constexpr int16_t DDT_INSPECTION_B =
    9;
  static constexpr int16_t DDT_AUTONOMOUS_DEMO =
    10;

  // pointer types
  using RawPtr =
    eufs_msgs::srv::SetMission_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const eufs_msgs::srv::SetMission_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eufs_msgs::srv::SetMission_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eufs_msgs::srv::SetMission_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::srv::SetMission_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::srv::SetMission_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::srv::SetMission_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::srv::SetMission_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eufs_msgs::srv::SetMission_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eufs_msgs::srv::SetMission_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eufs_msgs__srv__SetMission_Request
    std::shared_ptr<eufs_msgs::srv::SetMission_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eufs_msgs__srv__SetMission_Request
    std::shared_ptr<eufs_msgs::srv::SetMission_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMission_Request_ & other) const
  {
    if (this->mission != other.mission) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMission_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMission_Request_

// alias to use template instance with default allocator
using SetMission_Request =
  eufs_msgs::srv::SetMission_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::NOT_SELECTED;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::ACCELERATION;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::SKIDPAD;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::AUTOCROSS;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::TRACK_DRIVE;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::MANUAL_DRIVING;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::ADS_EBS_TEST;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::ADS_INSPECTION;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::DDT_INSPECTION_A;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::DDT_INSPECTION_B;
template<typename ContainerAllocator>
constexpr int16_t SetMission_Request_<ContainerAllocator>::DDT_AUTONOMOUS_DEMO;

}  // namespace srv

}  // namespace eufs_msgs


#ifndef _WIN32
# define DEPRECATED__eufs_msgs__srv__SetMission_Response __attribute__((deprecated))
#else
# define DEPRECATED__eufs_msgs__srv__SetMission_Response __declspec(deprecated)
#endif

namespace eufs_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMission_Response_
{
  using Type = SetMission_Response_<ContainerAllocator>;

  explicit SetMission_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetMission_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    eufs_msgs::srv::SetMission_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const eufs_msgs::srv::SetMission_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eufs_msgs::srv::SetMission_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eufs_msgs::srv::SetMission_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::srv::SetMission_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::srv::SetMission_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eufs_msgs::srv::SetMission_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eufs_msgs::srv::SetMission_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eufs_msgs::srv::SetMission_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eufs_msgs::srv::SetMission_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eufs_msgs__srv__SetMission_Response
    std::shared_ptr<eufs_msgs::srv::SetMission_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eufs_msgs__srv__SetMission_Response
    std::shared_ptr<eufs_msgs::srv::SetMission_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMission_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMission_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMission_Response_

// alias to use template instance with default allocator
using SetMission_Response =
  eufs_msgs::srv::SetMission_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace eufs_msgs

namespace eufs_msgs
{

namespace srv
{

struct SetMission
{
  using Request = eufs_msgs::srv::SetMission_Request;
  using Response = eufs_msgs::srv::SetMission_Response;
};

}  // namespace srv

}  // namespace eufs_msgs

#endif  // EUFS_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_HPP_
