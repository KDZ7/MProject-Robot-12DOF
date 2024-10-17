// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from space_interfaces:msg/Position.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "space_interfaces/msg/position.h"


#ifndef SPACE_INTERFACES__MSG__DETAIL__POSITION__STRUCT_H_
#define SPACE_INTERFACES__MSG__DETAIL__POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'source'
// Member 'target'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Position in the package space_interfaces.
/**
  * Position.msg
 */
typedef struct space_interfaces__msg__Position
{
  /// Represents the position and orientation of an object relative to a reference frame
  /// Reference frames for the position and orientation
  /// Source <--- Target
  rosidl_runtime_c__String source;
  rosidl_runtime_c__String target;
  /// Position coordinates in 3D space
  double x;
  double y;
  double z;
  /// Orientation coordinates in 3D space
  double roll;
  double pitch;
  double yaw;
} space_interfaces__msg__Position;

// Struct for a sequence of space_interfaces__msg__Position.
typedef struct space_interfaces__msg__Position__Sequence
{
  space_interfaces__msg__Position * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} space_interfaces__msg__Position__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPACE_INTERFACES__MSG__DETAIL__POSITION__STRUCT_H_
