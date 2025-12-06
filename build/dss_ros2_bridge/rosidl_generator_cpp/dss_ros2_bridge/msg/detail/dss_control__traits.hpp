// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dss_ros2_bridge:msg/DssControl.idl
// generated code does not contain a copyright notice

#ifndef DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__TRAITS_HPP_
#define DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dss_ros2_bridge/msg/detail/dss_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dss_ros2_bridge
{

namespace msg
{

inline void to_flow_style_yaml(
  const DssControl & msg,
  std::ostream & out)
{
  out << "{";
  // member: steer
  {
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << ", ";
  }

  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: brake
  {
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DssControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: steer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << "\n";
  }

  // member: throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << "\n";
  }

  // member: brake
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DssControl & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dss_ros2_bridge

namespace rosidl_generator_traits
{

[[deprecated("use dss_ros2_bridge::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dss_ros2_bridge::msg::DssControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  dss_ros2_bridge::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dss_ros2_bridge::msg::to_yaml() instead")]]
inline std::string to_yaml(const dss_ros2_bridge::msg::DssControl & msg)
{
  return dss_ros2_bridge::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dss_ros2_bridge::msg::DssControl>()
{
  return "dss_ros2_bridge::msg::DssControl";
}

template<>
inline const char * name<dss_ros2_bridge::msg::DssControl>()
{
  return "dss_ros2_bridge/msg/DssControl";
}

template<>
struct has_fixed_size<dss_ros2_bridge::msg::DssControl>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dss_ros2_bridge::msg::DssControl>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dss_ros2_bridge::msg::DssControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__TRAITS_HPP_
