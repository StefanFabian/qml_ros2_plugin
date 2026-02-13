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
 * Please note that this is a Q_GADGET and not a QObject which means there is no signal emitted
 * when properties change.
 */
class Time
{
  Q_GADGET
  //! The time in seconds (since 1970) as an integer.
  //! Use seconds() for floating point value including sub-seconds.
  Q_PROPERTY( qint32 sec READ sec WRITE setSec )
  //! The time in nanoseconds relative to the current second (as unsigned integer).
  //! Use nanoseconds() for the full time in nanoseconds since epoch.
  Q_PROPERTY( quint32 nanosec READ nanosec WRITE setNanosec )
public:
  explicit Time( const rclcpp::Time &time = rclcpp::Time() ) : time_( time ) { }

  //! Get the seconds since epoch as floating point including sub-seconds.
  Q_INVOKABLE double seconds() const { return time_.seconds(); }

  //! Get the nanoseconds since epoch.
  Q_INVOKABLE qint64 nanoseconds() const { return time_.nanoseconds(); }

  //! Whether the time represented by this instance is zero.
  Q_INVOKABLE bool isZero() const { return nanoseconds() == 0; }

  //!  A JS Date representing the value stored in this instance.
  //!  Since JS Dates only have millisecond accuracy, information about microseconds and nanoseconds
  //!  are lost. The time is always rounded down to prevent the JS Date from being in the future.
  Q_INVOKABLE QDateTime toJSDate() const { return rosToQmlTime( time_ ); }

  //! The type of the time source. See Ros2ClockTypes
  Q_INVOKABLE quint32 clockType() const { return time_.get_clock_type(); }

  rclcpp::Time getTime() const { return time_; }

  qint32 sec() const { return builtin_interfaces::msg::Time( time_ ).sec; }

  void setSec( qint32 sec )
  {
    auto t = builtin_interfaces::msg::Time( time_ );
    t.sec = sec;
    time_ = rclcpp::Time( t );
  }

  quint32 nanosec() const { return builtin_interfaces::msg::Time( time_ ).nanosec; }

  void setNanosec( quint32 nanosec )
  {
    auto t = builtin_interfaces::msg::Time( time_ );
    t.nanosec = nanosec;
    time_ = rclcpp::Time( t );
  }

private:
  rclcpp::Time time_;
};

/*!
 * Represents a duration for a time source.
 *
 * Please note that this is a Q_GADGET and not a QObject which means there is no signal emitted
 * when properties change.
 */
class Duration
{
  Q_GADGET
  //! The duration in seconds as an integer.
  //! Use seconds() for floating point value including sub-seconds.
  Q_PROPERTY( qint32 sec READ sec WRITE setSec )
  //! The time in nanoseconds relative to the current second (as unsigned integer).
  //! Use nanoseconds() for the full duration in nanoseconds.
  Q_PROPERTY( quint32 nanosec READ nanosec WRITE setNanosec )
public:
  explicit Duration(
      const rclcpp::Duration &duration = rclcpp::Duration( std::chrono::nanoseconds( 0 ) ) )
      : duration_( duration )
  {
  }

  //! Get the seconds of the duration as floating point including sub-seconds.
  Q_INVOKABLE double seconds() const { return duration_.seconds(); }

  //! Get the full duration as nanosecond.
  Q_INVOKABLE qint64 nanoseconds() const { return duration_.nanoseconds(); }

  //! Whether the duration represented by this instance is zero.
  Q_INVOKABLE bool isZero() const { return nanoseconds() == 0; }

  //! A JS duration representing the value stored in this instance.
  //! JS measures differences between two Dates as a (floating point) number of milliseconds.
  Q_INVOKABLE double toJSDuration() const { return rosToQmlDuration( duration_ ); }

  rclcpp::Duration getDuration() const { return duration_; }

  qint32 sec() const { return builtin_interfaces::msg::Duration( duration_ ).sec; }

  void setSec( qint32 sec )
  {
    auto d = builtin_interfaces::msg::Duration( duration_ );
    d.sec = sec;
    duration_ = rclcpp::Duration( d );
  }

  quint32 nanosec() const { return builtin_interfaces::msg::Duration( duration_ ).nanosec; }

  void setNanosec( quint32 nanosec )
  {
    auto d = builtin_interfaces::msg::Duration( duration_ );
    d.nanosec = nanosec;
    duration_ = rclcpp::Duration( d );
  }

private:
  rclcpp::Duration duration_;
};
} // namespace qml_ros2_plugin

// Register Time types
Q_DECLARE_METATYPE( qml_ros2_plugin::Time )

Q_DECLARE_METATYPE( qml_ros2_plugin::Duration )

#endif // QML_ROS2_PLUGIN_TIME_HPP
