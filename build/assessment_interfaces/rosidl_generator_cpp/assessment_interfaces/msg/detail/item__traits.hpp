// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from assessment_interfaces:msg/Item.idl
// generated code does not contain a copyright notice

#ifndef ASSESSMENT_INTERFACES__MSG__DETAIL__ITEM__TRAITS_HPP_
#define ASSESSMENT_INTERFACES__MSG__DETAIL__ITEM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "assessment_interfaces/msg/detail/item__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace assessment_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Item & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: diameter
  {
    out << "diameter: ";
    rosidl_generator_traits::value_to_yaml(msg.diameter, out);
    out << ", ";
  }

  // member: colour
  {
    out << "colour: ";
    rosidl_generator_traits::value_to_yaml(msg.colour, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Item & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: diameter
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diameter: ";
    rosidl_generator_traits::value_to_yaml(msg.diameter, out);
    out << "\n";
  }

  // member: colour
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "colour: ";
    rosidl_generator_traits::value_to_yaml(msg.colour, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Item & msg, bool use_flow_style = false)
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

}  // namespace assessment_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use assessment_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const assessment_interfaces::msg::Item & msg,
  std::ostream & out, size_t indentation = 0)
{
  assessment_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use assessment_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const assessment_interfaces::msg::Item & msg)
{
  return assessment_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<assessment_interfaces::msg::Item>()
{
  return "assessment_interfaces::msg::Item";
}

template<>
inline const char * name<assessment_interfaces::msg::Item>()
{
  return "assessment_interfaces/msg/Item";
}

template<>
struct has_fixed_size<assessment_interfaces::msg::Item>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<assessment_interfaces::msg::Item>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<assessment_interfaces::msg::Item>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASSESSMENT_INTERFACES__MSG__DETAIL__ITEM__TRAITS_HPP_
