// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eufs_msgs:msg/ConeArrayWithCovariancePlus.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_ARRAY_WITH_COVARIANCE_PLUS__STRUCT_H_
#define EUFS_MSGS__MSG__DETAIL__CONE_ARRAY_WITH_COVARIANCE_PLUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'cones'
#include "eufs_msgs/msg/detail/cone_with_covariance_plus__struct.h"

// Struct defined in msg/ConeArrayWithCovariancePlus in the package eufs_msgs.
typedef struct eufs_msgs__msg__ConeArrayWithCovariancePlus
{
  std_msgs__msg__Header header;
  eufs_msgs__msg__ConeWithCovariancePlus__Sequence cones;
} eufs_msgs__msg__ConeArrayWithCovariancePlus;

// Struct for a sequence of eufs_msgs__msg__ConeArrayWithCovariancePlus.
typedef struct eufs_msgs__msg__ConeArrayWithCovariancePlus__Sequence
{
  eufs_msgs__msg__ConeArrayWithCovariancePlus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eufs_msgs__msg__ConeArrayWithCovariancePlus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_ARRAY_WITH_COVARIANCE_PLUS__STRUCT_H_
