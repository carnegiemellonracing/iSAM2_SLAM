// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eufs_msgs:srv/SetTrack.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/srv/detail/set_track__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `data`
#include "eufs_msgs/msg/detail/cone_array_with_covariance__functions.h"

bool
eufs_msgs__srv__SetTrack_Request__init(eufs_msgs__srv__SetTrack_Request * msg)
{
  if (!msg) {
    return false;
  }
  // data
  if (!eufs_msgs__msg__ConeArrayWithCovariance__init(&msg->data)) {
    eufs_msgs__srv__SetTrack_Request__fini(msg);
    return false;
  }
  return true;
}

void
eufs_msgs__srv__SetTrack_Request__fini(eufs_msgs__srv__SetTrack_Request * msg)
{
  if (!msg) {
    return;
  }
  // data
  eufs_msgs__msg__ConeArrayWithCovariance__fini(&msg->data);
}

bool
eufs_msgs__srv__SetTrack_Request__are_equal(const eufs_msgs__srv__SetTrack_Request * lhs, const eufs_msgs__srv__SetTrack_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (!eufs_msgs__msg__ConeArrayWithCovariance__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
eufs_msgs__srv__SetTrack_Request__copy(
  const eufs_msgs__srv__SetTrack_Request * input,
  eufs_msgs__srv__SetTrack_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  if (!eufs_msgs__msg__ConeArrayWithCovariance__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

eufs_msgs__srv__SetTrack_Request *
eufs_msgs__srv__SetTrack_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__srv__SetTrack_Request * msg = (eufs_msgs__srv__SetTrack_Request *)allocator.allocate(sizeof(eufs_msgs__srv__SetTrack_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__srv__SetTrack_Request));
  bool success = eufs_msgs__srv__SetTrack_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__srv__SetTrack_Request__destroy(eufs_msgs__srv__SetTrack_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    eufs_msgs__srv__SetTrack_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
eufs_msgs__srv__SetTrack_Request__Sequence__init(eufs_msgs__srv__SetTrack_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__srv__SetTrack_Request * data = NULL;

  if (size) {
    data = (eufs_msgs__srv__SetTrack_Request *)allocator.zero_allocate(size, sizeof(eufs_msgs__srv__SetTrack_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__srv__SetTrack_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__srv__SetTrack_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
eufs_msgs__srv__SetTrack_Request__Sequence__fini(eufs_msgs__srv__SetTrack_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      eufs_msgs__srv__SetTrack_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

eufs_msgs__srv__SetTrack_Request__Sequence *
eufs_msgs__srv__SetTrack_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__srv__SetTrack_Request__Sequence * array = (eufs_msgs__srv__SetTrack_Request__Sequence *)allocator.allocate(sizeof(eufs_msgs__srv__SetTrack_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__srv__SetTrack_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
eufs_msgs__srv__SetTrack_Request__Sequence__destroy(eufs_msgs__srv__SetTrack_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    eufs_msgs__srv__SetTrack_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
eufs_msgs__srv__SetTrack_Request__Sequence__are_equal(const eufs_msgs__srv__SetTrack_Request__Sequence * lhs, const eufs_msgs__srv__SetTrack_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__srv__SetTrack_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__srv__SetTrack_Request__Sequence__copy(
  const eufs_msgs__srv__SetTrack_Request__Sequence * input,
  eufs_msgs__srv__SetTrack_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__srv__SetTrack_Request);
    eufs_msgs__srv__SetTrack_Request * data =
      (eufs_msgs__srv__SetTrack_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__srv__SetTrack_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__srv__SetTrack_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!eufs_msgs__srv__SetTrack_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
eufs_msgs__srv__SetTrack_Response__init(eufs_msgs__srv__SetTrack_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    eufs_msgs__srv__SetTrack_Response__fini(msg);
    return false;
  }
  return true;
}

void
eufs_msgs__srv__SetTrack_Response__fini(eufs_msgs__srv__SetTrack_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
eufs_msgs__srv__SetTrack_Response__are_equal(const eufs_msgs__srv__SetTrack_Response * lhs, const eufs_msgs__srv__SetTrack_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
eufs_msgs__srv__SetTrack_Response__copy(
  const eufs_msgs__srv__SetTrack_Response * input,
  eufs_msgs__srv__SetTrack_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

eufs_msgs__srv__SetTrack_Response *
eufs_msgs__srv__SetTrack_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__srv__SetTrack_Response * msg = (eufs_msgs__srv__SetTrack_Response *)allocator.allocate(sizeof(eufs_msgs__srv__SetTrack_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__srv__SetTrack_Response));
  bool success = eufs_msgs__srv__SetTrack_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__srv__SetTrack_Response__destroy(eufs_msgs__srv__SetTrack_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    eufs_msgs__srv__SetTrack_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
eufs_msgs__srv__SetTrack_Response__Sequence__init(eufs_msgs__srv__SetTrack_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__srv__SetTrack_Response * data = NULL;

  if (size) {
    data = (eufs_msgs__srv__SetTrack_Response *)allocator.zero_allocate(size, sizeof(eufs_msgs__srv__SetTrack_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__srv__SetTrack_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__srv__SetTrack_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
eufs_msgs__srv__SetTrack_Response__Sequence__fini(eufs_msgs__srv__SetTrack_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      eufs_msgs__srv__SetTrack_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

eufs_msgs__srv__SetTrack_Response__Sequence *
eufs_msgs__srv__SetTrack_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__srv__SetTrack_Response__Sequence * array = (eufs_msgs__srv__SetTrack_Response__Sequence *)allocator.allocate(sizeof(eufs_msgs__srv__SetTrack_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__srv__SetTrack_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
eufs_msgs__srv__SetTrack_Response__Sequence__destroy(eufs_msgs__srv__SetTrack_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    eufs_msgs__srv__SetTrack_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
eufs_msgs__srv__SetTrack_Response__Sequence__are_equal(const eufs_msgs__srv__SetTrack_Response__Sequence * lhs, const eufs_msgs__srv__SetTrack_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__srv__SetTrack_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__srv__SetTrack_Response__Sequence__copy(
  const eufs_msgs__srv__SetTrack_Response__Sequence * input,
  eufs_msgs__srv__SetTrack_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__srv__SetTrack_Response);
    eufs_msgs__srv__SetTrack_Response * data =
      (eufs_msgs__srv__SetTrack_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__srv__SetTrack_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__srv__SetTrack_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!eufs_msgs__srv__SetTrack_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
