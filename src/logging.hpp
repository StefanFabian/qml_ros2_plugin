// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_LOGGING_HPP
#define QML_ROS2_PLUGIN_LOGGING_HPP

#include <rclcpp/logging.hpp>

#define QML_ROS2_PLUGIN_DEBUG( ... )                                                               \
  RCLCPP_DEBUG( rclcpp::get_logger( "qml_ros2_plugin" ), __VA_ARGS__ )
#define QML_ROS2_PLUGIN_INFO( ... )                                                                \
  RCLCPP_INFO( rclcpp::get_logger( "qml_ros2_plugin" ), __VA_ARGS__ )
#define QML_ROS2_PLUGIN_WARN( ... )                                                                \
  RCLCPP_WARN( rclcpp::get_logger( "qml_ros2_plugin" ), __VA_ARGS__ )
#define QML_ROS2_PLUGIN_ERROR( ... )                                                               \
  RCLCPP_ERROR( rclcpp::get_logger( "qml_ros2_plugin" ), __VA_ARGS__ )

#endif // QML_ROS2_PLUGIN_LOGGING_HPP
