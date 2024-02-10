// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eufs_msgs:msg/ConeWithCovariancePlus.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__STRUCT_H_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/ConeWithCovariancePlus in the package eufs_msgs.
typedef struct eufs_msgs__msg__ConeWithCovariancePlus
{
  int32_t id;
  double blue_prob;
  double yellow_prob;
  double orange_prob;
  double big_orange_prob;
  geometry_msgs__msg__Point point;
  double covariance[4];
} eufs_msgs__msg__ConeWithCovariancePlus;

// Struct for a sequence of eufs_msgs__msg__ConeWithCovariancePlus.
typedef struct eufs_msgs__msg__ConeWithCovariancePlus__Sequence
{
  eufs_msgs__msg__ConeWithCovariancePlus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eufs_msgs__msg__ConeWithCovariancePlus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__STRUCT_H_
