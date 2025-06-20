// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/qos.hpp"

namespace qml_ros2_plugin
{
QoSWrapper::QoSWrapper() : qos_( 1 )
{
  qos_.best_effort();
  qos_.durability_volatile();
}

QoSWrapper::QoSWrapper( rclcpp::QoS qos ) : qos_( qos ) { }

QoSWrapper QoSWrapper::reliable()
{
  qos_.reliable();
  return *this;
}

QoSWrapper QoSWrapper::best_effort()
{
  qos_.best_effort();
  return *this;
}

QoSWrapper QoSWrapper::durability_volatile()
{
  qos_.durability_volatile();
  return *this;
}

QoSWrapper QoSWrapper::transient_local()
{
  qos_.transient_local();
  return *this;
}

QoSWrapper QoSWrapper::keep_all()
{
  qos_.keep_all();
  return *this;
}

qml_ros2_plugin::QoSWrapper QoSWrapper::keep_last( int depth )
{
  qos_.keep_last( depth );
  return *this;
}

int QoSWrapper::depth() const { return qos_.depth(); }

namespace
{
std::string to_string( const rclcpp::ReliabilityPolicy &reliability )
{
  switch ( reliability ) {
  case rclcpp::ReliabilityPolicy::BestEffort:
    return "BestEffort";
  case rclcpp::ReliabilityPolicy::Reliable:
    return "Reliable";
  case rclcpp::ReliabilityPolicy::SystemDefault:
    return "SystemDefault";
  case rclcpp::ReliabilityPolicy::BestAvailable:
    return "BestAvailable";
  case rclcpp::ReliabilityPolicy::Unknown:
    return "Unknown";
  }
  return "Invalid";
}

std::string to_string( const rclcpp::DurabilityPolicy &durability )
{
  switch ( durability ) {
  case rclcpp::DurabilityPolicy::Volatile:
    return "Volatile";
  case rclcpp::DurabilityPolicy::TransientLocal:
    return "TransientLocal";
  case rclcpp::DurabilityPolicy::SystemDefault:
    return "SystemDefault";
  case rclcpp::DurabilityPolicy::Unknown:
    return "Unknown";
  case rclcpp::DurabilityPolicy::BestAvailable:
    return "BestAvailable";
  }
  return "Invalid";
}

std::string to_string( const rclcpp::HistoryPolicy &history )
{
  switch ( history ) {
  case rclcpp::HistoryPolicy::KeepLast:
    return "KeepLast";
  case rclcpp::HistoryPolicy::KeepAll:
    return "KeepAll";
  case rclcpp::HistoryPolicy::SystemDefault:
    return "SystemDefault";
  case rclcpp::HistoryPolicy::Unknown:
    return "Unknown";
  }
  return "Invalid";
}
} // namespace

std::string QoSWrapper::toString() const
{
  std::string result = "QoS(";
  result += "reliability: " + to_string( qos_.reliability() ) + ", ";
  result += "durability: " + to_string( qos_.durability() ) + ", ";
  result += "history: " + to_string( qos_.history() ) + ", ";
  result += "depth: " + std::to_string( qos_.depth() ) + ")";
  return result;
}
} // namespace qml_ros2_plugin
