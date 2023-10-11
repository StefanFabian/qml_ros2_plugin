// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_TIME_HPP
#define QML_ROS2_PLUGIN_TIME_HPP

#include "qml_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml_ros2_plugin/qobject_ros2.hpp"

#include <QVariant>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

namespace qml_ros2_plugin
{
namespace ros_clock_types
{
Q_NAMESPACE

enum Ros2ClockTypes {
  //! Uninitialized
  Uninitialized = RCL_CLOCK_UNINITIALIZED,
  //! ROS time
  Ros = RCL_ROS_TIME,
  //! System time
  System = RCL_SYSTEM_TIME,
  //! Steady clock time
  Steady = RCL_STEADY_TIME
};

Q_ENUM_NS( Ros2ClockTypes )
} // namespace ros_clock_types

/*!
 * Represents a point in time for a time source.
 *
 * Properties:
 *   - seconds: Floating point value containing the seconds passed since epoch. Depending on the
 * size of double, this may have significant precision loss. Use nanoseconds for precision.
 *   - nanoseconds: unsigned integer containing the nanoseconds since epoch.
 *   - clockType: The type of the time source. See Ros2ClockTypes
 */
class Time
{
  Q_GADGET
  //! The time in seconds (since 1970) as a decimal value. (Possible loss in precision)
  Q_PROPERTY( double seconds READ seconds CONSTANT )
  //! The time in nanoseconds (since 1970) as an unsigned integer.
  Q_PROPERTY( quint64 nanoseconds READ nanoseconds CONSTANT )
  //! The clock type of this time point. See Ros2ClockTypes
  Q_PROPERTY( quint32 clockType READ clockType CONSTANT )
public:
  explicit Time( const rclcpp::Time &time = rclcpp::Time() ) : time_( time ) { }

  double seconds() const { return time_.seconds(); }

  quint64 nanoseconds() const { return time_.nanoseconds(); }

  //! Whether the time represented by this instance is zero.
  Q_INVOKABLE bool isZero() const { return nanoseconds() == 0; }

  //!  A JS Date representing the value stored in this instance.
  //!  Since JS Dates only have millisecond accuracy, information about microseconds and nanoseconds
  //!  are lost. The time is always rounded down to prevent the JS Date from being in the future.
  Q_INVOKABLE QDateTime toJSDate() const { return rosToQmlTime( time_ ); }

  quint32 clockType() const { return time_.get_clock_type(); }

  rclcpp::Time getTime() const { return time_; }

private:
  rclcpp::Time time_;
};

/*!
 * Represents a duration for a time source.
 *
 * Properties:
 *   - seconds: Floating point value containing the seconds passed between two time points. Can be
 *     negative. Depending on the size of double, may have significant precision loss. Use
 *     nanoseconds if you need a precise measurement.
 *   - nanoseconds: Signed integer containing the nanoseconds that have passed between two timepoints.
 */
class Duration
{
  Q_GADGET
  //! The time in seconds that has passed between two timepoints. (Possible loss in precision)
  Q_PROPERTY( double seconds READ seconds CONSTANT )
  //! The duration in nanoseconds
  Q_PROPERTY( qint64 nanoseconds READ nanoseconds CONSTANT )
public:
  explicit Duration(
      const rclcpp::Duration &duration = rclcpp::Duration( std::chrono::nanoseconds( 0 ) ) )
      : duration_( duration )
  {
  }

  double seconds() const { return duration_.seconds(); }

  qint64 nanoseconds() const { return duration_.nanoseconds(); }

  //! Whether the duration represented by this instance is zero.
  Q_INVOKABLE bool isZero() const { return nanoseconds() == 0; }

  //! A JS duration representing the value stored in this instance.
  //! JS measures differences between two Dates as a (floating point) number of milliseconds.
  Q_INVOKABLE double toJSDuration() const { return rosToQmlDuration( duration_ ); }

  rclcpp::Duration getDuration() const { return duration_; }

private:
  rclcpp::Duration duration_;
};
} // namespace qml_ros2_plugin

// Register Time types
Q_DECLARE_METATYPE( qml_ros2_plugin::Time )

Q_DECLARE_METATYPE( qml_ros2_plugin::Duration )

#endif // QML_ROS2_PLUGIN_TIME_HPP
