// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from urc_msgs:msg/ArucoDetection.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__ARUCO_DETECTION__STRUCT_H_
#define URC_MSGS__MSG__DETAIL__ARUCO_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ArucoDetection in the package urc_msgs.
typedef struct urc_msgs__msg__ArucoDetection
{
  std_msgs__msg__Header header;
  double x_angle;
  double y_angle;
  double z_angle;
  double distance;
  uint64_t id;
} urc_msgs__msg__ArucoDetection;

// Struct for a sequence of urc_msgs__msg__ArucoDetection.
typedef struct urc_msgs__msg__ArucoDetection__Sequence
{
  urc_msgs__msg__ArucoDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} urc_msgs__msg__ArucoDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // URC_MSGS__MSG__DETAIL__ARUCO_DETECTION__STRUCT_H_
