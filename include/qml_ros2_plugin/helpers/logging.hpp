/*
 * Copyright (C) 2021  Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
