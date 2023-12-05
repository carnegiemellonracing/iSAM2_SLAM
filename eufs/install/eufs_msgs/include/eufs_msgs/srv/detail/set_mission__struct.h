// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eufs_msgs:srv/SetMission.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_H_
#define EUFS_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NOT_SELECTED'.
enum
{
  eufs_msgs__srv__SetMission_Request__NOT_SELECTED = 0
};

/// Constant 'ACCELERATION'.
enum
{
  eufs_msgs__srv__SetMission_Request__ACCELERATION = 1
};

/// Constant 'SKIDPAD'.
enum
{
  eufs_msgs__srv__SetMission_Request__SKIDPAD = 2
};

/// Constant 'AUTOCROSS'.
enum
{
  eufs_msgs__srv__SetMission_Request__AUTOCROSS = 3
};

/// Constant 'TRACK_DRIVE'.
enum
{
  eufs_msgs__srv__SetMission_Request__TRACK_DRIVE = 4
};

/// Constant 'MANUAL_DRIVING'.
enum
{
  eufs_msgs__srv__SetMission_Request__MANUAL_DRIVING = 5
};

/// Constant 'ADS_EBS_TEST'.
enum
{
  eufs_msgs__srv__SetMission_Request__ADS_EBS_TEST = 6
};

/// Constant 'ADS_INSPECTION'.
enum
{
  eufs_msgs__srv__SetMission_Request__ADS_INSPECTION = 7
};

/// Constant 'DDT_INSPECTION_A'.
enum
{
  eufs_msgs__srv__SetMission_Request__DDT_INSPECTION_A = 8
};

/// Constant 'DDT_INSPECTION_B'.
enum
{
  eufs_msgs__srv__SetMission_Request__DDT_INSPECTION_B = 9
};

/// Constant 'DDT_AUTONOMOUS_DEMO'.
enum
{
  eufs_msgs__srv__SetMission_Request__DDT_AUTONOMOUS_DEMO = 10
};

// Struct defined in srv/SetMission in the package eufs_msgs.
typedef struct eufs_msgs__srv__SetMission_Request
{
  int16_t mission;
} eufs_msgs__srv__SetMission_Request;

// Struct for a sequence of eufs_msgs__srv__SetMission_Request.
typedef struct eufs_msgs__srv__SetMission_Request__Sequence
{
  eufs_msgs__srv__SetMission_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eufs_msgs__srv__SetMission_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetMission in the package eufs_msgs.
typedef struct eufs_msgs__srv__SetMission_Response
{
  bool success;
  rosidl_runtime_c__String message;
} eufs_msgs__srv__SetMission_Response;

// Struct for a sequence of eufs_msgs__srv__SetMission_Response.
typedef struct eufs_msgs__srv__SetMission_Response__Sequence
{
  eufs_msgs__srv__SetMission_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eufs_msgs__srv__SetMission_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EUFS_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_H_
