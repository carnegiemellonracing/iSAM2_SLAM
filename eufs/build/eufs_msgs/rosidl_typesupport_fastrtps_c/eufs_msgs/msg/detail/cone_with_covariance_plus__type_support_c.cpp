// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from eufs_msgs:msg/ConeWithCovariancePlus.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/msg/detail/cone_with_covariance_plus__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "eufs_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "eufs_msgs/msg/detail/cone_with_covariance_plus__struct.h"
#include "eufs_msgs/msg/detail/cone_with_covariance_plus__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/point__functions.h"  // point

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_eufs_msgs
size_t get_serialized_size_geometry_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_eufs_msgs
size_t max_serialized_size_geometry_msgs__msg__Point(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_eufs_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point)();


using _ConeWithCovariancePlus__ros_msg_type = eufs_msgs__msg__ConeWithCovariancePlus;

static bool _ConeWithCovariancePlus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ConeWithCovariancePlus__ros_msg_type * ros_message = static_cast<const _ConeWithCovariancePlus__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: blue_prob
  {
    cdr << ros_message->blue_prob;
  }

  // Field name: yellow_prob
  {
    cdr << ros_message->yellow_prob;
  }

  // Field name: orange_prob
  {
    cdr << ros_message->orange_prob;
  }

  // Field name: big_orange_prob
  {
    cdr << ros_message->big_orange_prob;
  }

  // Field name: unknown_prob
  {
    cdr << ros_message->unknown_prob;
  }

  // Field name: point
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->point, cdr))
    {
      return false;
    }
  }

  // Field name: covariance
  {
    size_t size = 4;
    auto array_ptr = ros_message->covariance;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _ConeWithCovariancePlus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ConeWithCovariancePlus__ros_msg_type * ros_message = static_cast<_ConeWithCovariancePlus__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: blue_prob
  {
    cdr >> ros_message->blue_prob;
  }

  // Field name: yellow_prob
  {
    cdr >> ros_message->yellow_prob;
  }

  // Field name: orange_prob
  {
    cdr >> ros_message->orange_prob;
  }

  // Field name: big_orange_prob
  {
    cdr >> ros_message->big_orange_prob;
  }

  // Field name: unknown_prob
  {
    cdr >> ros_message->unknown_prob;
  }

  // Field name: point
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->point))
    {
      return false;
    }
  }

  // Field name: covariance
  {
    size_t size = 4;
    auto array_ptr = ros_message->covariance;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_eufs_msgs
size_t get_serialized_size_eufs_msgs__msg__ConeWithCovariancePlus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ConeWithCovariancePlus__ros_msg_type * ros_message = static_cast<const _ConeWithCovariancePlus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name blue_prob
  {
    size_t item_size = sizeof(ros_message->blue_prob);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name yellow_prob
  {
    size_t item_size = sizeof(ros_message->yellow_prob);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name orange_prob
  {
    size_t item_size = sizeof(ros_message->orange_prob);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name big_orange_prob
  {
    size_t item_size = sizeof(ros_message->big_orange_prob);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name unknown_prob
  {
    size_t item_size = sizeof(ros_message->unknown_prob);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name point

  current_alignment += get_serialized_size_geometry_msgs__msg__Point(
    &(ros_message->point), current_alignment);
  // field.name covariance
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->covariance;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ConeWithCovariancePlus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_eufs_msgs__msg__ConeWithCovariancePlus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_eufs_msgs
size_t max_serialized_size_eufs_msgs__msg__ConeWithCovariancePlus(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: blue_prob
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: yellow_prob
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: orange_prob
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: big_orange_prob
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: unknown_prob
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: point
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Point(
        full_bounded, current_alignment);
    }
  }
  // member: covariance
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _ConeWithCovariancePlus__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_eufs_msgs__msg__ConeWithCovariancePlus(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_ConeWithCovariancePlus = {
  "eufs_msgs::msg",
  "ConeWithCovariancePlus",
  _ConeWithCovariancePlus__cdr_serialize,
  _ConeWithCovariancePlus__cdr_deserialize,
  _ConeWithCovariancePlus__get_serialized_size,
  _ConeWithCovariancePlus__max_serialized_size
};

static rosidl_message_type_support_t _ConeWithCovariancePlus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ConeWithCovariancePlus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, eufs_msgs, msg, ConeWithCovariancePlus)() {
  return &_ConeWithCovariancePlus__type_support;
}

#if defined(__cplusplus)
}
#endif
