// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_QML_ROS_CONVERSION_HPP
#define QML_ROS2_PLUGIN_QML_ROS_CONVERSION_HPP

#include <QDateTime>
#include <cmath>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace qml_ros2_plugin
{

inline int64_t milliseconds_to_nanoseconds( double milliseconds )
{

  auto ms = static_cast<int64_t>( milliseconds );
  double fraction =
      milliseconds - ( milliseconds < 0 ? std::ceil( milliseconds ) : std::floor( milliseconds ) );
  return ms * static_cast<int64_t>( 1E6 ) + static_cast<int64_t>( fraction * 1E6 );
}

inline rclcpp::Duration qmlToRos2Duration( double milliseconds )
{
  return rclcpp::Duration( std::chrono::nanoseconds( milliseconds_to_nanoseconds( milliseconds ) ) );
}

inline rclcpp::Time qmlToRos2Time( double milliseconds_since_epoch )
{
  return rclcpp::Time( milliseconds_to_nanoseconds( milliseconds_since_epoch ) );
}

inline rclcpp::Time qmlToRos2Time( const QDateTime &time )
{
  if ( !time.isValid() )
    return rclcpp::Time();
  qint64 msecs = time.toMSecsSinceEpoch();
  return rclcpp::Time( msecs * static_cast<int64_t>( 1E6 ) );
}

inline double rosToQmlDuration( const rclcpp::Duration &duration )
{
  return duration.seconds() * 1E3;
}

inline QDateTime rosToQmlTime( const rclcpp::Time &time )
{
  // Always round down because otherwise high precision stuff like tf might fail due to, e.g., look up into future
  return QDateTime::fromMSecsSinceEpoch( time.nanoseconds() / static_cast<qint64>( 1E6 ) );
}
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_QML_ROS_CONVERSION_HPP
