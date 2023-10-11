// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/logger.hpp"
#include "qml_ros2_plugin/helpers/logging.hpp"

#include <QJSEngine>

namespace qml_ros2_plugin
{

Logger::Logger( const rclcpp::Logger &logger ) : logger_( logger ) { }

bool Logger::setLoggerLevel( Ros2LoggerLevel level )
{
  // Using C API since set_level is not available for ROS2 Foxy and we don't really want to throw from this either.
  rcutils_ret_t result = rcutils_logging_set_logger_level(
      logger_.get_name(), static_cast<RCUTILS_LOG_SEVERITY>( level ) );
  if ( result != RCUTILS_RET_OK ) {
    std::string reason = rcutils_get_error_string().str;
    if ( reason.empty() )
      reason = "Unknown";
    QML_ROS2_PLUGIN_ERROR( "Failed to set logger level for '%s'! Reason: %s", logger_.get_name(),
                           reason.c_str() );
    rcutils_reset_error();
    return false;
  }
  return true;
}

QJSValue Logger::debug()
{
  if ( debug_function_.isCallable() )
    return debug_function_;
  return debug_function_ = createLogFunction( Ros2LoggerLevel::Debug );
}

QJSValue Logger::info()
{
  if ( info_function_.isCallable() )
    return info_function_;
  return info_function_ = createLogFunction( Ros2LoggerLevel::Info );
}

QJSValue Logger::warn()
{
  if ( warn_function_.isCallable() )
    return warn_function_;
  return warn_function_ = createLogFunction( Ros2LoggerLevel::Warn );
}

QJSValue Logger::error()
{
  if ( error_function_.isCallable() )
    return error_function_;
  return error_function_ = createLogFunction( Ros2LoggerLevel::Error );
}

QJSValue Logger::fatal()
{
  if ( fatal_function_.isCallable() )
    return fatal_function_;
  return fatal_function_ = createLogFunction( Ros2LoggerLevel::Fatal );
}

void Logger::logInternal( int severity, const QString &function, const QString &file, int line,
                          const QString &msg ) const
{
  const char *logger_name = logger_.get_name();
  RCUTILS_LOGGING_AUTOINIT;
  if ( rcutils_logging_logger_is_enabled_for( logger_name, severity ) ) {
    std::string function_std = function.toStdString();
    std::string file_std = file.toStdString();
    rcutils_log_location_t logging_location = { function_std.c_str(), file_std.c_str(),
                                                static_cast<size_t>( line ) };
    rcutils_log( &logging_location, severity, logger_name, "%s", msg.toStdString().c_str() );
  }
}

QJSValue Logger::createLogFunction( Ros2LoggerLevel level )
{
  auto engine = qjsEngine( this );
  if ( !engine ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to create logging function." );
    return {};
  }
  // This function extracts the file, method and number which called the log function in order to accurately report it.
  QJSValue func = engine->evaluate( R"js((function (__logger_instance) {
  return (function (msg) {
    var stack = new Error().stack.split('\n');
    if (stack && stack.length >= 2) {
      var call_info = stack[1].split('@');
      var method = 'unknown', file = 'unknown', line = 0;
      if (call_info && call_info.length >= 2) {
        method = call_info[0];
        var file_info = call_info[1].replace('file://', '');
        var line_sep = file_info.lastIndexOf(':');
        if (line_sep != -1) {
          file = file_info.substr(0, line_sep);
          line = Number(file_info.substr(line_sep + 1));
        }
      }
    }
    __logger_instance.logInternal()js" +
                                    QString::number( level ) + R"js(, method, file, Number(line), msg);
  });
}))js" );
  return func.call( { engine->newQObject( this ) } );
}
} // namespace qml_ros2_plugin
