// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from micro_ros_msgs:msg/Entity.idl
// generated code does not contain a copyright notice
#ifndef MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "micro_ros_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "micro_ros_msgs/msg/detail/entity__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_micro_ros_msgs
bool cdr_serialize_micro_ros_msgs__msg__Entity(
  const micro_ros_msgs__msg__Entity * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_micro_ros_msgs
bool cdr_deserialize_micro_ros_msgs__msg__Entity(
  eprosima::fastcdr::Cdr &,
  micro_ros_msgs__msg__Entity * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_micro_ros_msgs
size_t get_serialized_size_micro_ros_msgs__msg__Entity(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_micro_ros_msgs
size_t max_serialized_size_micro_ros_msgs__msg__Entity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_micro_ros_msgs
bool cdr_serialize_key_micro_ros_msgs__msg__Entity(
  const micro_ros_msgs__msg__Entity * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_micro_ros_msgs
size_t get_serialized_size_key_micro_ros_msgs__msg__Entity(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_micro_ros_msgs
size_t max_serialized_size_key_micro_ros_msgs__msg__Entity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_micro_ros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, micro_ros_msgs, msg, Entity)();

#ifdef __cplusplus
}
#endif

#endif  // MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
