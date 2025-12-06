// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from dss_ros2_bridge:msg/DssControl.idl
// generated code does not contain a copyright notice

#ifndef DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dss_ros2_bridge/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "dss_ros2_bridge/msg/detail/dss_control__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace dss_ros2_bridge
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_ros2_bridge
cdr_serialize(
  const dss_ros2_bridge::msg::DssControl & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_ros2_bridge
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dss_ros2_bridge::msg::DssControl & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_ros2_bridge
get_serialized_size(
  const dss_ros2_bridge::msg::DssControl & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_ros2_bridge
max_serialized_size_DssControl(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dss_ros2_bridge

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_ros2_bridge
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dss_ros2_bridge, msg, DssControl)();

#ifdef __cplusplus
}
#endif

#endif  // DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
