// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dss_ros2_bridge:msg/DssControl.idl
// generated code does not contain a copyright notice

#ifndef DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__BUILDER_HPP_
#define DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dss_ros2_bridge/msg/detail/dss_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dss_ros2_bridge
{

namespace msg
{

namespace builder
{

class Init_DssControl_brake
{
public:
  explicit Init_DssControl_brake(::dss_ros2_bridge::msg::DssControl & msg)
  : msg_(msg)
  {}
  ::dss_ros2_bridge::msg::DssControl brake(::dss_ros2_bridge::msg::DssControl::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dss_ros2_bridge::msg::DssControl msg_;
};

class Init_DssControl_throttle
{
public:
  explicit Init_DssControl_throttle(::dss_ros2_bridge::msg::DssControl & msg)
  : msg_(msg)
  {}
  Init_DssControl_brake throttle(::dss_ros2_bridge::msg::DssControl::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_DssControl_brake(msg_);
  }

private:
  ::dss_ros2_bridge::msg::DssControl msg_;
};

class Init_DssControl_steer
{
public:
  Init_DssControl_steer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DssControl_throttle steer(::dss_ros2_bridge::msg::DssControl::_steer_type arg)
  {
    msg_.steer = std::move(arg);
    return Init_DssControl_throttle(msg_);
  }

private:
  ::dss_ros2_bridge::msg::DssControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dss_ros2_bridge::msg::DssControl>()
{
  return dss_ros2_bridge::msg::builder::Init_DssControl_steer();
}

}  // namespace dss_ros2_bridge

#endif  // DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__BUILDER_HPP_
