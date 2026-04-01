#ifndef ESVO2_CORE_PARAMS_HELPER_H
#define ESVO2_CORE_PARAMS_HELPER_H

#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace esvo2_core
{
namespace tools
{

template<typename T>
T param(rclcpp::Node* node, const std::string &name, const T &defaultValue)
{
  // Only declare if not already declared
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, defaultValue);
  }
  T v;
  if (node->get_parameter(name, v))
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
    return v;
  }
  RCLCPP_WARN_STREAM(node->get_logger(), "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  return defaultValue;
}
} // tools
} // esvo2_core

#endif //ESVO2_CORE_PARAMS_HELPER_H
