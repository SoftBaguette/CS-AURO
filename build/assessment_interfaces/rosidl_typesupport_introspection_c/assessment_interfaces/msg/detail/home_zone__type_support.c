// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from assessment_interfaces:msg/HomeZone.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "assessment_interfaces/msg/detail/home_zone__rosidl_typesupport_introspection_c.h"
#include "assessment_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "assessment_interfaces/msg/detail/home_zone__functions.h"
#include "assessment_interfaces/msg/detail/home_zone__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  assessment_interfaces__msg__HomeZone__init(message_memory);
}

void assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_fini_function(void * message_memory)
{
  assessment_interfaces__msg__HomeZone__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_message_member_array[4] = {
  {
    "visible",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(assessment_interfaces__msg__HomeZone, visible),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(assessment_interfaces__msg__HomeZone, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(assessment_interfaces__msg__HomeZone, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(assessment_interfaces__msg__HomeZone, size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_message_members = {
  "assessment_interfaces__msg",  // message namespace
  "HomeZone",  // message name
  4,  // number of fields
  sizeof(assessment_interfaces__msg__HomeZone),
  assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_message_member_array,  // message members
  assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_init_function,  // function to initialize message memory (memory has to be allocated)
  assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_message_type_support_handle = {
  0,
  &assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_assessment_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, assessment_interfaces, msg, HomeZone)() {
  if (!assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_message_type_support_handle.typesupport_identifier) {
    assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &assessment_interfaces__msg__HomeZone__rosidl_typesupport_introspection_c__HomeZone_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
