// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eufs_msgs:msg/ConeWithCovariancePlus.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/msg/detail/cone_with_covariance_plus__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `point`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
eufs_msgs__msg__ConeWithCovariancePlus__init(eufs_msgs__msg__ConeWithCovariancePlus * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // blue_prob
  // yellow_prob
  // orange_prob
  // big_orange_prob
  // unknown_prob
  // point
  if (!geometry_msgs__msg__Point__init(&msg->point)) {
    eufs_msgs__msg__ConeWithCovariancePlus__fini(msg);
    return false;
  }
  // covariance
  return true;
}

void
eufs_msgs__msg__ConeWithCovariancePlus__fini(eufs_msgs__msg__ConeWithCovariancePlus * msg)
{
  if (!msg) {
    return;
  }
  // id
  // blue_prob
  // yellow_prob
  // orange_prob
  // big_orange_prob
  // unknown_prob
  // point
  geometry_msgs__msg__Point__fini(&msg->point);
  // covariance
}

bool
eufs_msgs__msg__ConeWithCovariancePlus__are_equal(const eufs_msgs__msg__ConeWithCovariancePlus * lhs, const eufs_msgs__msg__ConeWithCovariancePlus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // blue_prob
  if (lhs->blue_prob != rhs->blue_prob) {
    return false;
  }
  // yellow_prob
  if (lhs->yellow_prob != rhs->yellow_prob) {
    return false;
  }
  // orange_prob
  if (lhs->orange_prob != rhs->orange_prob) {
    return false;
  }
  // big_orange_prob
  if (lhs->big_orange_prob != rhs->big_orange_prob) {
    return false;
  }
  // unknown_prob
  if (lhs->unknown_prob != rhs->unknown_prob) {
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->point), &(rhs->point)))
  {
    return false;
  }
  // covariance
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->covariance[i] != rhs->covariance[i]) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__msg__ConeWithCovariancePlus__copy(
  const eufs_msgs__msg__ConeWithCovariancePlus * input,
  eufs_msgs__msg__ConeWithCovariancePlus * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // blue_prob
  output->blue_prob = input->blue_prob;
  // yellow_prob
  output->yellow_prob = input->yellow_prob;
  // orange_prob
  output->orange_prob = input->orange_prob;
  // big_orange_prob
  output->big_orange_prob = input->big_orange_prob;
  // unknown_prob
  output->unknown_prob = input->unknown_prob;
  // point
  if (!geometry_msgs__msg__Point__copy(
      &(input->point), &(output->point)))
  {
    return false;
  }
  // covariance
  for (size_t i = 0; i < 4; ++i) {
    output->covariance[i] = input->covariance[i];
  }
  return true;
}

eufs_msgs__msg__ConeWithCovariancePlus *
eufs_msgs__msg__ConeWithCovariancePlus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__msg__ConeWithCovariancePlus * msg = (eufs_msgs__msg__ConeWithCovariancePlus *)allocator.allocate(sizeof(eufs_msgs__msg__ConeWithCovariancePlus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__msg__ConeWithCovariancePlus));
  bool success = eufs_msgs__msg__ConeWithCovariancePlus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__msg__ConeWithCovariancePlus__destroy(eufs_msgs__msg__ConeWithCovariancePlus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    eufs_msgs__msg__ConeWithCovariancePlus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
eufs_msgs__msg__ConeWithCovariancePlus__Sequence__init(eufs_msgs__msg__ConeWithCovariancePlus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__msg__ConeWithCovariancePlus * data = NULL;

  if (size) {
    data = (eufs_msgs__msg__ConeWithCovariancePlus *)allocator.zero_allocate(size, sizeof(eufs_msgs__msg__ConeWithCovariancePlus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__msg__ConeWithCovariancePlus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__msg__ConeWithCovariancePlus__fini(&data[i - 1]);
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
eufs_msgs__msg__ConeWithCovariancePlus__Sequence__fini(eufs_msgs__msg__ConeWithCovariancePlus__Sequence * array)
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
      eufs_msgs__msg__ConeWithCovariancePlus__fini(&array->data[i]);
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

eufs_msgs__msg__ConeWithCovariancePlus__Sequence *
eufs_msgs__msg__ConeWithCovariancePlus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eufs_msgs__msg__ConeWithCovariancePlus__Sequence * array = (eufs_msgs__msg__ConeWithCovariancePlus__Sequence *)allocator.allocate(sizeof(eufs_msgs__msg__ConeWithCovariancePlus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__msg__ConeWithCovariancePlus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
eufs_msgs__msg__ConeWithCovariancePlus__Sequence__destroy(eufs_msgs__msg__ConeWithCovariancePlus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    eufs_msgs__msg__ConeWithCovariancePlus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
eufs_msgs__msg__ConeWithCovariancePlus__Sequence__are_equal(const eufs_msgs__msg__ConeWithCovariancePlus__Sequence * lhs, const eufs_msgs__msg__ConeWithCovariancePlus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__msg__ConeWithCovariancePlus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__msg__ConeWithCovariancePlus__Sequence__copy(
  const eufs_msgs__msg__ConeWithCovariancePlus__Sequence * input,
  eufs_msgs__msg__ConeWithCovariancePlus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__msg__ConeWithCovariancePlus);
    eufs_msgs__msg__ConeWithCovariancePlus * data =
      (eufs_msgs__msg__ConeWithCovariancePlus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__msg__ConeWithCovariancePlus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__msg__ConeWithCovariancePlus__fini(&data[i]);
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
    if (!eufs_msgs__msg__ConeWithCovariancePlus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
