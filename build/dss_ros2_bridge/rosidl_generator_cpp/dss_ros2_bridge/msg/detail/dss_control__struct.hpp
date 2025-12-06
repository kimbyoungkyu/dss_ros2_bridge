// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dss_ros2_bridge:msg/DssControl.idl
// generated code does not contain a copyright notice

#ifndef DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__STRUCT_HPP_
#define DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dss_ros2_bridge__msg__DssControl __attribute__((deprecated))
#else
# define DEPRECATED__dss_ros2_bridge__msg__DssControl __declspec(deprecated)
#endif

namespace dss_ros2_bridge
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DssControl_
{
  using Type = DssControl_<ContainerAllocator>;

  explicit DssControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steer = 0.0f;
      this->throttle = 0.0f;
      this->brake = 0.0f;
    }
  }

  explicit DssControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steer = 0.0f;
      this->throttle = 0.0f;
      this->brake = 0.0f;
    }
  }

  // field types and members
  using _steer_type =
    float;
  _steer_type steer;
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _brake_type =
    float;
  _brake_type brake;

  // setters for named parameter idiom
  Type & set__steer(
    const float & _arg)
  {
    this->steer = _arg;
    return *this;
  }
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__brake(
    const float & _arg)
  {
    this->brake = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dss_ros2_bridge::msg::DssControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const dss_ros2_bridge::msg::DssControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dss_ros2_bridge::msg::DssControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dss_ros2_bridge::msg::DssControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dss_ros2_bridge::msg::DssControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dss_ros2_bridge::msg::DssControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dss_ros2_bridge::msg::DssControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dss_ros2_bridge::msg::DssControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dss_ros2_bridge::msg::DssControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dss_ros2_bridge::msg::DssControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dss_ros2_bridge__msg__DssControl
    std::shared_ptr<dss_ros2_bridge::msg::DssControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dss_ros2_bridge__msg__DssControl
    std::shared_ptr<dss_ros2_bridge::msg::DssControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DssControl_ & other) const
  {
    if (this->steer != other.steer) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->brake != other.brake) {
      return false;
    }
    return true;
  }
  bool operator!=(const DssControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DssControl_

// alias to use template instance with default allocator
using DssControl =
  dss_ros2_bridge::msg::DssControl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dss_ros2_bridge

#endif  // DSS_ROS2_BRIDGE__MSG__DETAIL__DSS_CONTROL__STRUCT_HPP_
