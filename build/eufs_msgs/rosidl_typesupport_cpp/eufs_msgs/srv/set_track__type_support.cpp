// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from eufs_msgs:srv/SetTrack.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "eufs_msgs/srv/detail/set_track__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace eufs_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetTrack_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetTrack_Request_type_support_ids_t;

static const _SetTrack_Request_type_support_ids_t _SetTrack_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetTrack_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetTrack_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetTrack_Request_type_support_symbol_names_t _SetTrack_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, eufs_msgs, srv, SetTrack_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, eufs_msgs, srv, SetTrack_Request)),
  }
};

typedef struct _SetTrack_Request_type_support_data_t
{
  void * data[2];
} _SetTrack_Request_type_support_data_t;

static _SetTrack_Request_type_support_data_t _SetTrack_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetTrack_Request_message_typesupport_map = {
  2,
  "eufs_msgs",
  &_SetTrack_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetTrack_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetTrack_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetTrack_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetTrack_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace eufs_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<eufs_msgs::srv::SetTrack_Request>()
{
  return &::eufs_msgs::srv::rosidl_typesupport_cpp::SetTrack_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, eufs_msgs, srv, SetTrack_Request)() {
  return get_message_type_support_handle<eufs_msgs::srv::SetTrack_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "eufs_msgs/srv/detail/set_track__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace eufs_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetTrack_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetTrack_Response_type_support_ids_t;

static const _SetTrack_Response_type_support_ids_t _SetTrack_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetTrack_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetTrack_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetTrack_Response_type_support_symbol_names_t _SetTrack_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, eufs_msgs, srv, SetTrack_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, eufs_msgs, srv, SetTrack_Response)),
  }
};

typedef struct _SetTrack_Response_type_support_data_t
{
  void * data[2];
} _SetTrack_Response_type_support_data_t;

static _SetTrack_Response_type_support_data_t _SetTrack_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetTrack_Response_message_typesupport_map = {
  2,
  "eufs_msgs",
  &_SetTrack_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetTrack_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetTrack_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetTrack_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetTrack_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace eufs_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<eufs_msgs::srv::SetTrack_Response>()
{
  return &::eufs_msgs::srv::rosidl_typesupport_cpp::SetTrack_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, eufs_msgs, srv, SetTrack_Response)() {
  return get_message_type_support_handle<eufs_msgs::srv::SetTrack_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "eufs_msgs/srv/detail/set_track__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace eufs_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetTrack_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetTrack_type_support_ids_t;

static const _SetTrack_type_support_ids_t _SetTrack_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetTrack_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetTrack_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetTrack_type_support_symbol_names_t _SetTrack_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, eufs_msgs, srv, SetTrack)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, eufs_msgs, srv, SetTrack)),
  }
};

typedef struct _SetTrack_type_support_data_t
{
  void * data[2];
} _SetTrack_type_support_data_t;

static _SetTrack_type_support_data_t _SetTrack_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetTrack_service_typesupport_map = {
  2,
  "eufs_msgs",
  &_SetTrack_service_typesupport_ids.typesupport_identifier[0],
  &_SetTrack_service_typesupport_symbol_names.symbol_name[0],
  &_SetTrack_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetTrack_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetTrack_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace eufs_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<eufs_msgs::srv::SetTrack>()
{
  return &::eufs_msgs::srv::rosidl_typesupport_cpp::SetTrack_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, eufs_msgs, srv, SetTrack)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<eufs_msgs::srv::SetTrack>();
}

#ifdef __cplusplus
}
#endif
