// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_LOGGER_HPP
#define QML_ROS2_PLUGIN_LOGGER_HPP

#include <QJSValue>
#include <QtCore>
#include <rclcpp/logging.hpp>

namespace qml_ros2_plugin
{

namespace ros2_logger_levels
{
Q_NAMESPACE

enum Ros2LoggerLevel {
  Unset = RCUTILS_LOG_SEVERITY_UNSET,
  Debug = RCUTILS_LOG_SEVERITY_DEBUG,
  Info = RCUTILS_LOG_SEVERITY_INFO,
  Warn = RCUTILS_LOG_SEVERITY_WARN,
  Error = RCUTILS_LOG_SEVERITY_ERROR,
  Fatal = RCUTILS_LOG_SEVERITY_FATAL
};

Q_ENUM_NS( Ros2LoggerLevel )
} // namespace ros2_logger_levels
using Ros2LoggerLevel = ros2_logger_levels::Ros2LoggerLevel;

class Logger : public QObject
{
  Q_OBJECT
  Q_PROPERTY( QJSValue debug READ debug CONSTANT )
  Q_PROPERTY( QJSValue info READ info CONSTANT )
  Q_PROPERTY( QJSValue warn READ warn CONSTANT )
  Q_PROPERTY( QJSValue error READ error CONSTANT )
  Q_PROPERTY( QJSValue fatal READ fatal CONSTANT )
public:
  explicit Logger( const rclcpp::Logger &logger );

  // Type needs to be explicit so QML will recognize it
  Q_INVOKABLE bool setLoggerLevel( qml_ros2_plugin::ros2_logger_levels::Ros2LoggerLevel level );

  /*!
   * Outputs a ROS debug message. The equivalent of calling ROS_DEBUG in C++.
   * The signature in QML is @c debug(msg).
   *
   * @b Note: The msg is not a format string.
   *
   * Example:
   * @code
   * Ros2.debug("The value of x is: " + x);
   * @endcode
   */
  QJSValue debug();

  /*!
   * Outputs a ROS info message. The equivalent of calling ROS_INFO in C++.
   * The signature in QML is @c info(msg).
   *
   * @b Note: The argument is not a format string.
   *
   * Example:
   * @code
   * Ros2.info("The value of x is: " + x);
   * @endcode
   */
  QJSValue info();

  /*!
   * Outputs a ROS warn message. The equivalent of calling ROS_WARN in C++.
   * The signature in QML is @c warn(msg).
   *
   * @b Note: The argument is not a format string.
   *
   * Example:
   * @code
   * Ros2.warn("The value of x is: " + x);
   * @endcode
   */
  QJSValue warn();

  /*!
   * Outputs a ROS error message. The equivalent of calling ROS_ERROR in C++.
   * The signature in QML is @c error(msg).
   *
   * @b Note: The argument is not a format string.
   *
   * Example:
   * @code
   * Ros2.error("The value of x is: " + x);
   * @endcode
   */
  QJSValue error();

  /*!
   * Outputs a ROS fatal message. The equivalent of calling ROS_FATAL in C++.
   * The signature in QML is @c fatal(msg).
   *
   * @b Note: The argument is not a format string.
   *
   * Example:
   * @code
   * Ros2.fatal("The value of x is: " + x);
   * @endcode
   */
  QJSValue fatal();

  Q_INVOKABLE void logInternal( int severity, const QString &function, const QString &file,
                                int line, const QString &msg ) const;

private:
  QJSValue createLogFunction( Ros2LoggerLevel level );

  rclcpp::Logger logger_;
  QJSValue log_function_;
  QJSValue debug_function_;
  QJSValue info_function_;
  QJSValue warn_function_;
  QJSValue error_function_;
  QJSValue fatal_function_;
};
} // namespace qml_ros2_plugin

Q_DECLARE_METATYPE( qml_ros2_plugin::ros2_logger_levels::Ros2LoggerLevel )

#endif // QML_ROS2_PLUGIN_LOGGER_HPP
