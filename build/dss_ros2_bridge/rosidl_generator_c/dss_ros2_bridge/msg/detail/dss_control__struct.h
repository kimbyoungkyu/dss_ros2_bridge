// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dss_ros2_bridge:msg/DssControl.idl
// generated code does not contain a copyright notice

#ifndef DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__STRUCT_H_
#define DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DssControl in the package dss_ros2_bridge.
typedef struct dss_ros2_bridge__msg__DssControl
{
  float steer;
  float throttle;
  float brake;
} dss_ros2_bridge__msg__DssControl;

// Struct for a sequence of dss_ros2_bridge__msg__DssControl.
typedef struct dss_ros2_bridge__msg__DssControl__Sequence
{
  dss_ros2_bridge__msg__DssControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dss_ros2_bridge__msg__DssControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__STRUCT_H_
