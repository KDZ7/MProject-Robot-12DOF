// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from space_interfaces:msg/Position.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "space_interfaces/msg/position.hpp"


#ifndef SPACE_INTERFACES__MSG__DETAIL__POSITION__BUILDER_HPP_
#define SPACE_INTERFACES__MSG__DETAIL__POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "space_interfaces/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace space_interfaces
{

namespace msg
{

namespace builder
{

class Init_Position_yaw
{
public:
  explicit Init_Position_yaw(::space_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  ::space_interfaces::msg::Position yaw(::space_interfaces::msg::Position::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::space_interfaces::msg::Position msg_;
};

class Init_Position_pitch
{
public:
  explicit Init_Position_pitch(::space_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_yaw pitch(::space_interfaces::msg::Position::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Position_yaw(msg_);
  }

private:
  ::space_interfaces::msg::Position msg_;
};

class Init_Position_roll
{
public:
  explicit Init_Position_roll(::space_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_pitch roll(::space_interfaces::msg::Position::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_Position_pitch(msg_);
  }

private:
  ::space_interfaces::msg::Position msg_;
};

class Init_Position_z
{
public:
  explicit Init_Position_z(::space_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_roll z(::space_interfaces::msg::Position::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Position_roll(msg_);
  }

private:
  ::space_interfaces::msg::Position msg_;
};

class Init_Position_y
{
public:
  explicit Init_Position_y(::space_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_z y(::space_interfaces::msg::Position::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Position_z(msg_);
  }

private:
  ::space_interfaces::msg::Position msg_;
};

class Init_Position_x
{
public:
  explicit Init_Position_x(::space_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_y x(::space_interfaces::msg::Position::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Position_y(msg_);
  }

private:
  ::space_interfaces::msg::Position msg_;
};

class Init_Position_target
{
public:
  explicit Init_Position_target(::space_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_x target(::space_interfaces::msg::Position::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_Position_x(msg_);
  }

private:
  ::space_interfaces::msg::Position msg_;
};

class Init_Position_source
{
public:
  Init_Position_source()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_target source(::space_interfaces::msg::Position::_source_type arg)
  {
    msg_.source = std::move(arg);
    return Init_Position_target(msg_);
  }

private:
  ::space_interfaces::msg::Position msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::space_interfaces::msg::Position>()
{
  return space_interfaces::msg::builder::Init_Position_source();
}

}  // namespace space_interfaces

#endif  // SPACE_INTERFACES__MSG__DETAIL__POSITION__BUILDER_HPP_
