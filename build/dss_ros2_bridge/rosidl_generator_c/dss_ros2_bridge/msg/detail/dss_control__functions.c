// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dss_ros2_bridge:msg/DssControl.idl
// generated code does not contain a copyright notice
#include "dss_ros2_bridge/msg/detail/dss_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
dss_ros2_bridge__msg__DssControl__init(dss_ros2_bridge__msg__DssControl * msg)
{
  if (!msg) {
    return false;
  }
  // steer
  // throttle
  // brake
  return true;
}

void
dss_ros2_bridge__msg__DssControl__fini(dss_ros2_bridge__msg__DssControl * msg)
{
  if (!msg) {
    return;
  }
  // steer
  // throttle
  // brake
}

bool
dss_ros2_bridge__msg__DssControl__are_equal(const dss_ros2_bridge__msg__DssControl * lhs, const dss_ros2_bridge__msg__DssControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // steer
  if (lhs->steer != rhs->steer) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // brake
  if (lhs->brake != rhs->brake) {
    return false;
  }
  return true;
}

bool
dss_ros2_bridge__msg__DssControl__copy(
  const dss_ros2_bridge__msg__DssControl * input,
  dss_ros2_bridge__msg__DssControl * output)
{
  if (!input || !output) {
    return false;
  }
  // steer
  output->steer = input->steer;
  // throttle
  output->throttle = input->throttle;
  // brake
  output->brake = input->brake;
  return true;
}

dss_ros2_bridge__msg__DssControl *
dss_ros2_bridge__msg__DssControl__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_ros2_bridge__msg__DssControl * msg = (dss_ros2_bridge__msg__DssControl *)allocator.allocate(sizeof(dss_ros2_bridge__msg__DssControl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dss_ros2_bridge__msg__DssControl));
  bool success = dss_ros2_bridge__msg__DssControl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dss_ros2_bridge__msg__DssControl__destroy(dss_ros2_bridge__msg__DssControl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dss_ros2_bridge__msg__DssControl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dss_ros2_bridge__msg__DssControl__Sequence__init(dss_ros2_bridge__msg__DssControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_ros2_bridge__msg__DssControl * data = NULL;

  if (size) {
    data = (dss_ros2_bridge__msg__DssControl *)allocator.zero_allocate(size, sizeof(dss_ros2_bridge__msg__DssControl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dss_ros2_bridge__msg__DssControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dss_ros2_bridge__msg__DssControl__fini(&data[i - 1]);
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
dss_ros2_bridge__msg__DssControl__Sequence__fini(dss_ros2_bridge__msg__DssControl__Sequence * array)
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
      dss_ros2_bridge__msg__DssControl__fini(&array->data[i]);
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

dss_ros2_bridge__msg__DssControl__Sequence *
dss_ros2_bridge__msg__DssControl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dss_ros2_bridge__msg__DssControl__Sequence * array = (dss_ros2_bridge__msg__DssControl__Sequence *)allocator.allocate(sizeof(dss_ros2_bridge__msg__DssControl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dss_ros2_bridge__msg__DssControl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dss_ros2_bridge__msg__DssControl__Sequence__destroy(dss_ros2_bridge__msg__DssControl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dss_ros2_bridge__msg__DssControl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dss_ros2_bridge__msg__DssControl__Sequence__are_equal(const dss_ros2_bridge__msg__DssControl__Sequence * lhs, const dss_ros2_bridge__msg__DssControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dss_ros2_bridge__msg__DssControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dss_ros2_bridge__msg__DssControl__Sequence__copy(
  const dss_ros2_bridge__msg__DssControl__Sequence * input,
  dss_ros2_bridge__msg__DssControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dss_ros2_bridge__msg__DssControl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dss_ros2_bridge__msg__DssControl * data =
      (dss_ros2_bridge__msg__DssControl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dss_ros2_bridge__msg__DssControl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dss_ros2_bridge__msg__DssControl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dss_ros2_bridge__msg__DssControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
