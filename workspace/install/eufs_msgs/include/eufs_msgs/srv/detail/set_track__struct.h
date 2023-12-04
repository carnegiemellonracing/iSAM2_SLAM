// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eufs_msgs:srv/SetTrack.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__SRV__DETAIL__SET_TRACK__STRUCT_H_
#define EUFS_MSGS__SRV__DETAIL__SET_TRACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "eufs_msgs/msg/detail/cone_array_with_covariance__struct.h"

// Struct defined in srv/SetTrack in the package eufs_msgs.
typedef struct eufs_msgs__srv__SetTrack_Request
{
  eufs_msgs__msg__ConeArrayWithCovariance data;
} eufs_msgs__srv__SetTrack_Request;

// Struct for a sequence of eufs_msgs__srv__SetTrack_Request.
typedef struct eufs_msgs__srv__SetTrack_Request__Sequence
{
  eufs_msgs__srv__SetTrack_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eufs_msgs__srv__SetTrack_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetTrack in the package eufs_msgs.
typedef struct eufs_msgs__srv__SetTrack_Response
{
  bool success;
  rosidl_runtime_c__String message;
} eufs_msgs__srv__SetTrack_Response;

// Struct for a sequence of eufs_msgs__srv__SetTrack_Response.
typedef struct eufs_msgs__srv__SetTrack_Response__Sequence
{
  eufs_msgs__srv__SetTrack_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eufs_msgs__srv__SetTrack_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EUFS_MSGS__SRV__DETAIL__SET_TRACK__STRUCT_H_
